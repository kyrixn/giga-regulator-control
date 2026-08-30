#include <Arduino.h>
#include <math.h>

// 兼容某些编译环境未定义 M_PI
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ==== 用户参数（按需修改）==== */
const uint8_t  PIN_RE_DE      = 2;         // RS485 DE&/RE 控制脚（HIGH=TX, LOW=RX）
const uint8_t  SLAVE_ID       = 1;         // 传感器 Modbus 地址
const uint32_t BAUD_485       = 115200;    // 传感器波特率（与设备一致）
const uint16_t REG_BASE_0380  = 0x0380;    // 单圈位置高位起始
const uint16_t WORDS_TO_READ  = 4;         // 0380..0383（位置32位 + 圈数32位）
const uint8_t  ENCODER_BITS_N = 21;        // 编码器位数 n（用于角度换算）

/* ==== 时序参数 ==== */
const uint16_t TX_SETTLE_US   = 15;
const uint16_t TX_HOLD_US     = 30;
const uint16_t RX_WINDOW_MS   = 30;
const uint16_t IDLE_GAP_MS    = 2;
const uint16_t LOOP_DELAY_MS  = 20;        // ≈50 Hz 读取

/* ==== 派生常量 ==== */
const uint32_t COUNTS_PER_REV = (1UL << ENCODER_BITS_N);

/* ==== 位移换算参数 & 方向（按需修改） ==== */
const double SPOOL_DIAMETER_MM = 13.7;   // 线轴直径，mm（需要微调时在这里改）
const int8_t DIR_SIGN          = +1;     // 方向系数：方向反了就设为 -1

/* ==== 零点与命令处理状态 ==== */
static double zero_deg = NAN;             // 零位角（度，已考虑方向），按 ‘r’ 键设置
static String cmdLine;                    // 串口命令缓冲

/* ==== 速度计算状态 ==== */
static bool     vel_init       = false;
static double   prev_disp_mm   = 0.0;
static uint32_t prev_t_ms      = 0;
static double   vel_ema_mm_s   = 0.0;
const  double   VEL_ALPHA      = 0.2;     // 速度一阶低通（0~1，越大越跟手，越小越平滑）

/* ==== 工具 ==== */
static inline void setTX(){ digitalWrite(PIN_RE_DE, HIGH); }
static inline void setRX(){ digitalWrite(PIN_RE_DE, LOW);  }

uint16_t crc16_modbus(const uint8_t* d, size_t n){
  uint16_t crc = 0xFFFF;
  for(size_t i=0;i<n;i++){
    crc ^= d[i];
    for(uint8_t b=0;b<8;b++){
      crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0xA001) : (uint16_t)(crc >> 1);
    }
  }
  return crc;
}

size_t modbus_read03(uint8_t slave_id, uint16_t startReg, uint16_t count, uint8_t* rx, size_t cap){
  uint8_t req[8];
  req[0]=slave_id; req[1]=0x03;
  req[2]=(uint8_t)(startReg>>8); req[3]=(uint8_t)(startReg&0xFF);
  req[4]=(uint8_t)(count>>8);    req[5]=(uint8_t)(count&0xFF);
  uint16_t crc = crc16_modbus(req,6);
  req[6]=(uint8_t)(crc & 0xFF); req[7]=(uint8_t)(crc >> 8);

  while(Serial1.available()) (void)Serial1.read();

  setTX(); delayMicroseconds(TX_SETTLE_US);
  Serial1.write(req, sizeof(req));
  Serial1.flush();
  delayMicroseconds(TX_HOLD_US);
  setRX();

  size_t rlen=0; uint32_t t0=millis(), last=t0;
  while(millis()-t0 < RX_WINDOW_MS){
    while(Serial1.available()){
      int b = Serial1.read();
      if(b>=0 && rlen<cap){ rx[rlen++]=(uint8_t)b; last=millis(); }
    }
    if(rlen>0 && (millis()-last) > IDLE_GAP_MS) break;
  }
  return rlen;
}

/* 解析 0380..0383：pos_u32（无符号）+ turns_i32（有符号补码） */
bool parse_pos_turns(uint8_t slave_id, const uint8_t* rx, size_t n, uint32_t& pos_u32, int32_t& turns_i32){
  // 期望帧: id(1) 03(1) byteCount(1=8) + data(8) + CRC(2) => 总长 13 字节
  if(n < 13) return false;
  uint16_t recv_crc = (uint16_t)rx[n-2] | ((uint16_t)rx[n-1] << 8);
  if(recv_crc != crc16_modbus(rx, n-2)) return false;
  if(rx[0] != slave_id || rx[1] != 0x03 || rx[2] != 0x08) return false;

  const uint8_t* p = rx + 3;
  // 单圈位置：无符号32位（大端）
  pos_u32 = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
  // 圈数计数：有符号32位（大端）
  uint32_t turns_u32 = ((uint32_t)p[4] << 24) | ((uint32_t)p[5] << 16) | ((uint32_t)p[6] << 8) | (uint32_t)p[7];
  turns_i32 = (int32_t)turns_u32;
  return true;
}

/* ==== 串口命令处理 ==== */
void handleSerialCommand(double current_deg_signed){
  while(Serial.available()){
    char ch = (char)Serial.read();
    if(ch=='\r' || ch=='\n'){
      if(cmdLine.length()){
        if(cmdLine.equalsIgnoreCase("r")){
          // 置零：把当前角度设为零位
          zero_deg = current_deg_signed;
          // 同步速度初值
          vel_init = false;
          Serial.println(F("# Zero set."));
        }else if(cmdLine.equals("?")){
          Serial.print(F("# dia="));
          Serial.print(SPOOL_DIAMETER_MM, 3);
          Serial.print(F(" mm, zero="));
          if(isnan(zero_deg)) Serial.print(F("unset"));
          else Serial.print(zero_deg, 3);
          Serial.print(F(" deg (signed), DIR_SIGN="));
          Serial.println((int)DIR_SIGN);
        }else{
          Serial.print(F("# Unknown cmd: "));
          Serial.println(cmdLine);
        }
        cmdLine = "";
      }
    }else{
      if(isPrintable(ch)) cmdLine += ch;
    }
  }

  // 上电第一次：把当前角度作为零位，避免初始位移很大
  if(isnan(zero_deg)){
    zero_deg = current_deg_signed;
  }
}

/* ==== Arduino 入口 ==== */
void setup(){
  pinMode(PIN_RE_DE, OUTPUT); setRX();
  Serial.begin(115200);          // 与PC通信（记录CSV）
  Serial1.begin(BAUD_485);       // 与传感器通信（RS485: TX1=18, RX1=19）

  Serial.println(F("# Ready. Commands: r (reset zero), ? (status)"));
  // CSV 表头：t_ms, 位移(mm), 速度(mm/s)
  Serial.println(F("t_ms,disp_mm,vel_mm_s"));
}

void loop(){
  uint8_t rx[32];
  uint32_t pos_u32 = 0;
  int32_t  turns_i32 = 0;

  size_t n = modbus_read03(SLAVE_ID, REG_BASE_0380, WORDS_TO_READ, rx, sizeof(rx));
  if(n){
    if(parse_pos_turns(SLAVE_ID, rx, n, pos_u32, turns_i32)){
      // 单圈角度（0..<360）
      uint32_t pos_mod = (COUNTS_PER_REV == 0) ? pos_u32 : (pos_u32 % COUNTS_PER_REV);
      double angle_single_deg = (double)pos_mod * 360.0 / (double)COUNTS_PER_REV;

      // 多圈展开角度
      double angle_unwrapped_deg = (double)turns_i32 * 360.0 + angle_single_deg;

      // 带方向符号
      double angle_signed_deg = (double)DIR_SIGN * angle_unwrapped_deg;

      // 处理串口命令（r / ?），并在首次上电时自动置零
      handleSerialCommand(angle_signed_deg);

      // 角度 -> 位移（mm）
      double mm_per_deg = (M_PI * SPOOL_DIAMETER_MM) / 360.0;
      double displacement_mm = (angle_signed_deg - zero_deg) * mm_per_deg;

      // 速度（mm/s）：相邻两点差分 + EMA 平滑
      unsigned long t_ms = millis();
      double vel_out = 0.0;
      if(!vel_init){
        vel_init = true;
        prev_disp_mm = displacement_mm;
        prev_t_ms    = t_ms;
        vel_ema_mm_s = 0.0;
        vel_out      = 0.0;
      }else{
        unsigned long dt_ms = t_ms - prev_t_ms;
        if(dt_ms > 0){
          double raw_v = (displacement_mm - prev_disp_mm) / ((double)dt_ms / 1000.0);
          vel_ema_mm_s = VEL_ALPHA * raw_v + (1.0 - VEL_ALPHA) * vel_ema_mm_s;
          vel_out = vel_ema_mm_s;
        }
        prev_disp_mm = displacement_mm;
        prev_t_ms    = t_ms;
      }

      // —— CSV 输出：t_ms,disp_mm,vel_mm_s —— （保留三位小数）
      Serial.print(t_ms);
      Serial.print(',');
      Serial.print(displacement_mm, 3);
      Serial.print(',');
      Serial.println(vel_out, 3);
    }
  }

  delay(LOOP_DELAY_MS);
}
