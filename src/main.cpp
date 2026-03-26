#include "Arduino.h"

// define pins
#define UART1_TX PA9
#define UART1_RX PA10

#define UART2_TX PA2
#define UART2_RX PA3

#define THRESHOLD PB5

#define IN1 PC13
#define IN2 PC14
#define IN3 PC15
#define IN4 PC0
#define IN5 PC1
#define IN6 PC2
#define IN7 PC3
#define IN8 PA0
#define IN9 PA1
#define IN10 PA4
#define IN11 PA5
#define IN12 PA6
#define IN13 PA7
#define IN14 PC4
#define IN15 PC5
#define IN16 PB0

#define RING_LINE 16

#define L1 PA11
#define L2 PA12
#define LEDCtrl PB4

// define classes
HardwareSerial SerialPC(UART1_RX, UART1_TX);
HardwareSerial SerialMain(UART2_RX, UART2_TX);
HardwareTimer *timer = nullptr;

// define valiables
#define LINE_SetThreshold 0xAB
#define LINE_SENSOR_INFO 0xAC
#define LINE_SENSOR_HEADER 0xAD
#define LINE_CARIBRATION_ERROR 0xAE
#define LINE_ANGLE_INFO 0xAF
#define LINE_RESET 0xB0

const float STEP = 360.0f / RING_LINE;

volatile uint8_t LineInfo[RING_LINE];

uint8_t brightness = 230;
uint8_t threshold = 120;

inline int fastReadIndex(int i, uint32_t A, uint32_t B, uint32_t C) {
  switch (i) {
    case 0:  return (C >> 13) & 1;
    case 1:  return (C >> 14) & 1;
    case 2:  return (C >> 15) & 1;
    case 3:  return (C >> 0) & 1;
    case 4:  return (C >> 1) & 1;
    case 5:  return (C >> 2) & 1;
    case 6:  return (C >> 3) & 1;
    case 7:  return (A >> 0) & 1;
    case 8:  return (A >> 1) & 1;
    case 9:  return (A >> 4) & 1;
    case 10: return (A >> 5) & 1;
    case 11: return (A >> 6) & 1;
    case 12: return (A >> 7) & 1;
    case 13: return (C >> 4) & 1;
    case 14: return (C >> 5) & 1;
    case 15: return (B >> 0) & 1;
  }
  return 0;
}

// define functions
void setupTimer();
void timerISR();
void sendInt16(int16_t val);
void sensorInfo();
uint8_t readThreshold();
int16_t calcEscapeAngleFromRing16();
bool calcLineTraceFromRing16(float radius,float *normalAngle,float *normalDist);
double wrapAngle360(double angle);
double wrapangle180(double angle);


void setup() {
  SerialPC.begin(115200);
  SerialMain.begin(115200);
  setupTimer();

  pinMode(THRESHOLD, OUTPUT);
  analogWrite(THRESHOLD, threshold);

  pinMode(LEDCtrl, OUTPUT);
  analogWrite(LEDCtrl, brightness);

  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  digitalWrite(L1, HIGH);
  digitalWrite(L2, HIGH);

  // 入力設定
  pinMode(IN1, INPUT_PULLUP);
  pinMode(IN2, INPUT_PULLUP);
  pinMode(IN3, INPUT_PULLUP);
  pinMode(IN4, INPUT_PULLUP);
  pinMode(IN5, INPUT_PULLUP);
  pinMode(IN6, INPUT_PULLUP);
  pinMode(IN7, INPUT_PULLUP);
  pinMode(IN8, INPUT_PULLUP);
  pinMode(IN9, INPUT_PULLUP);
  pinMode(IN10, INPUT_PULLUP);
  pinMode(IN11, INPUT_PULLUP);
  pinMode(IN12, INPUT_PULLUP);
  pinMode(IN13, INPUT_PULLUP);
  pinMode(IN14, INPUT_PULLUP);
  pinMode(IN15, INPUT_PULLUP);
  pinMode(IN16, INPUT_PULLUP);
}

void loop() {
  //tsts
  float lineangle = 0;
  float linedist = 0;
  bool calclinesuccess = calcLineTraceFromRing16(1.0, &lineangle, &linedist);
  if(calclinesuccess){
    SerialPC.print("Line Angle: ");
    SerialPC.println(lineangle);
    SerialPC.print(" Line Dist: ");
    SerialPC.println(linedist);
  }else{
    SerialPC.println("Line Not Detected");
  }

  /*
  int16_t angle = calcEscapeAngleFromRing16();

  if (SerialMain.available()) {
    uint8_t cmd = SerialMain.read();

    if (cmd == LINE_ANGLE_INFO) {
      digitalWrite(L1, !digitalRead(L1));
      sendInt16(angle);
    }
    else if (cmd == LINE_SENSOR_INFO) {
      sensorInfo();
    }
    else if (cmd == LINE_SetThreshold) {
      uint8_t val = readThreshold();
      if (val != 0xFF) {
        threshold = val;
        analogWrite(THRESHOLD, threshold);
      }
    }
    else if (cmd == LINE_RESET) {
      SerialPC.println("Reset Angle");
    }
  }
    */
}

void setupTimer() {
  timer = new HardwareTimer(TIM2);

  timer->setOverflow(2000, HERTZ_FORMAT); // 2kHz
  timer->attachInterrupt(timerISR);
  timer->resume();
}

void timerISR() {

  uint32_t A = GPIOA->IDR;
  uint32_t B = GPIOB->IDR;
  uint32_t C = GPIOC->IDR;

  for (int i = 0; i < RING_LINE; i++) {
    LineInfo[i] = fastReadIndex(i, A, B, C);
  }
}

void sensorInfo() {
  SerialMain.write(LINE_SENSOR_HEADER);

  uint32_t bits = 0;

  uint8_t buffer[RING_LINE];

  noInterrupts();
  memcpy(buffer, (const void*)LineInfo, RING_LINE);
  interrupts();

  for (int i = 0; i < RING_LINE; i++) {
    if (buffer[i]) bits |= (1UL << i);
  }

  SerialMain.write((bits >> 0) & 0xFF);
  SerialMain.write((bits >> 8) & 0xFF);
  SerialMain.write((bits >> 16) & 0x07);

  digitalWrite(L2, !digitalRead(L2));
}

int16_t calcEscapeAngleFromRing16() {

  float sumX = 0;
  float sumY = 0;
  int detected = 0;

  int first = -1;
  int last = -1;

  static float escapeAngle = -1;
  static int entryIndex = -1;
  static int lastIndex = -1;
  static bool escaping = false;

  int centerIndex = -1;

  // 安全にコピー
  uint8_t buf[RING_LINE];

  noInterrupts();
  memcpy(buf, (const void*)LineInfo, RING_LINE);
  interrupts();

  for (int i = 0; i < RING_LINE; i++) {

    int v = buf[i];

    if (v) {
      detected++;

      if (first == -1) first = i;
      last = i;

      float ang = (-i * STEP) * DEG_TO_RAD;

      sumX += cos(ang);
      sumY += sin(ang);
    }
  }

  if (detected > 1) {

    float lineAngle = atan2(sumY, sumX) * RAD_TO_DEG;
    if (lineAngle < 0) lineAngle += 360;

    int width = last - first;

    if (width < 8)
      centerIndex = (first + last) / 2;
    else
      centerIndex = ((first + last + RING_LINE) / 2) % RING_LINE;

    float newEscape = lineAngle + 180;
    if (newEscape >= 360) newEscape -= 360;

    if (!escaping) {
      escaping = true;
      entryIndex = centerIndex;
      escapeAngle = newEscape;
    }

    lastIndex = centerIndex;

    float diff = newEscape - escapeAngle;
    while (diff > 180) diff -= 360;
    while (diff < -180) diff += 360;

    diff = constrain(diff, -45, 45);

    float limited = escapeAngle + diff;
    if (limited < 0) limited += 360;
    if (limited >= 360) limited -= 360;

    return (int16_t)(limited + 0.5f);
  }

  else {
    if (escaping) {
      int d = abs(lastIndex - entryIndex);
      if (d > 8) d = 16 - d;

      if (d <= 6) {
        escaping = false;
        entryIndex = -1;
        lastIndex = -1;
        escapeAngle = -1;
        return -1;
      }
      else {
        return (int16_t)(escapeAngle + 0.5f);
      }
    }
    return -1;
  }
}

bool calcLineTraceFromRing16(float radius,float *normalAngle,float *normalDist){
  uint8_t buf[RING_LINE];

  noInterrupts();
  memcpy(buf, (const void*)LineInfo, RING_LINE);
  interrupts();

  int first = -1;
  int last = -1;
  int count = 0;

  // 反応のあるセンサの最初と最後を探す
  for(int i = 0; i < RING_LINE; i++){
    if(buf[i]){
      if(first == -1) first = i;
      last = i;
      count++;
    }
  }

  // 2点未満のときは不可
  if(count < 2){
    return false;
  }

  // 0番またぎ対応
  if(buf[0] && buf[RING_LINE - 1]){
    int t = RING_LINE - 1;
    while(t >= 0 && buf[t]) t--;
    first = t + 1;

    int h = 0;
    while(h < RING_LINE && buf[h]) h++;
    last = h - 1;
  }

  // センサ角度
  float a1 = -first * STEP * DEG_TO_RAD;
  float a2 = -last  * STEP * DEG_TO_RAD;

  // 円周上の端点
  float x1 = radius * cos(a1);
  float y1 = radius * sin(a1);
  float x2 = radius * cos(a2);
  float y2 = radius * sin(a2);

  // 白線方向ベクトル
  float dx = x2 - x1;
  float dy = y2 - y1;

  float len = sqrtf(dx*dx + dy*dy);
  if(len < 1e-6f) return false;


  // 機体中心から直線までの最短距離
  float cross = x1*y2 - y1*x2;
  *normalDist = fabsf(cross) / len;

  // -------------------------
  // 垂線方向
  // 白線に垂直なベクトルは2通りある
  // (-dy, dx) と (dy, -dx)
  // そのうち「白線のある側」を向く方を選ぶ
  // -------------------------
  float nx1 = -dy;
  float ny1 =  dx;

  float nx2 =  dy;
  float ny2 = -dx;

  // 白線の中点
  float mx = (x1 + x2) * 0.5f;
  float my = (y1 + y2) * 0.5f;

  // 中点方向と同じ側を向く法線を採用
  float dot1 = nx1*mx + ny1*my;

  float nx, ny;
  if(dot1 >= 0){
    nx = nx1;
    ny = ny1;
  }else{
    nx = nx2;
    ny = ny2;
  }

  // 正規化
  float nlen = sqrtf(nx*nx + ny*ny);
  if(nlen < 1e-6f) return false;

  nx /= nlen;
  ny /= nlen;

  float ang = -atan2f(ny, nx) * RAD_TO_DEG;
  while(ang < 0.0f) ang += 360.0f;
  while(ang >= 360.0f) ang -= 360.0f;

  *normalAngle = ang;

  return true;
}

void sendInt16(int16_t val){
  byte *data = (byte *)&val;
  SerialMain.write(data[0]);
  SerialMain.write(data[1]);
}

uint8_t readThreshold() {
  unsigned long start = millis();
  while (SerialMain.available() < 1) {
    if (millis() - start > 10) return 0xFF;
  }
  return SerialMain.read();
}

double wrapAngle360(double angle) {
  while (angle >= 360.0) angle -= 360.0;
  while (angle < 0.0) angle += 360.0;
  return angle;
}

double wrapangle180(double angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}