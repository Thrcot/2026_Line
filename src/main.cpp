#include "Arduino.h"

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
const int LINE[RING_LINE] = {IN1, IN2, IN3, IN4, IN5, IN6, IN7, IN8, IN9, IN10, IN11, IN12, IN13, IN14, IN15, IN16};

#define L1 PA11
#define L2 PA12

#define LEDCtrl PB4

HardwareSerial SerialPC(UART1_RX, UART1_TX);
HardwareSerial SerialMain(UART2_RX, UART2_TX);

uint8_t brightness = 230; //180
uint8_t threshold = 155; //95

#define LINE_SetThreshold 0xAB
#define LINE_SENSOR_INFO 0xAC
#define LINE_SENSOR_HEADER 0xAD
#define LINE_CARIBRATION_ERROR 0xAE
#define LINE_ANGLE_INFO 0xAF

const float STEP = 360.0f / RING_LINE;

void sendInt16(int16_t val);
void sensorInfo();
uint8_t readThreshold();
int16_t calcEscapeAngleFromRing16();

void setup() {
	SerialPC.begin(115200);
	SerialMain.begin(115200);
	pinMode(THRESHOLD, OUTPUT);
	analogWrite(THRESHOLD, threshold);
	pinMode(LEDCtrl, OUTPUT);
	analogWrite(LEDCtrl, brightness);
	pinMode(L1, OUTPUT);
	pinMode(L2, OUTPUT);
	digitalWrite(L1, HIGH);
	digitalWrite(L2, HIGH);

	for (int i = 0; i < RING_LINE; i++) {
		pinMode(LINE[i], INPUT_PULLUP);
	}
}

void loop() {
	int16_t angle = -1;

	//angle = calcEscapeAngleFromRing16();

	// 送受信処理
	if (SerialMain.available()) {
		uint8_t cmd = SerialMain.read();
		if(cmd == LINE_ANGLE_INFO){
			digitalWrite(L1, !digitalRead(L1));
			sendInt16(angle);
		}else if(cmd == LINE_SENSOR_INFO){
			sensorInfo();
		}else if(cmd == LINE_SetThreshold){
			uint8_t val = readThreshold();
			if(val != 0xFF){
				threshold = val;
				analogWrite(THRESHOLD, threshold);
				SerialPC.print("Set Threshold:");
				SerialPC.println(threshold);
			}
		}
		SerialPC.println(cmd, HEX);
	}
	delay(1);
}

void sendInt16(int16_t val){
  byte *data = (byte *)&val;
  for (int j = 0; j < 2; j++) {  // int16_t は 2 バイト
    SerialMain.write(data[j]);
  }
  digitalWrite(L2, !digitalRead(L2));

  /*
  SerialPC.print(data[0]);
  SerialPC.print(" ");
  SerialPC.println(data[1]);
  */
}

void sensorInfo() {
    SerialMain.write(LINE_SENSOR_HEADER); // ヘッダ送信

    uint32_t bits = 0; // 32bit 変数で19ビットをまとめる
    for (int i = 0; i < RING_LINE; i++) {
        int v = digitalRead(LINE[i]); // 1:白ライン, 0:黒
        if (v) bits |= (1UL << i);
    }

    // 19ビットを3バイトに分けて送信
    SerialMain.write((bits >> 0) & 0xFF);   // 下位8bit
    SerialMain.write((bits >> 8) & 0xFF);   // 中位8bit
    SerialMain.write((bits >> 16) & 0x07);  // 上位3bit

    // デバッグ用LED
    digitalWrite(L2, !digitalRead(L2));
}

uint8_t readThreshold() {
    unsigned long startTime = millis();
    while (SerialMain.available() < 1) {
        if (millis() - startTime > 10) {  // 10ms タイムアウト
            return 0xFF;  // 受信失敗
        }
    }
	digitalWrite(L2, !digitalRead(L2));
    return SerialMain.read();
}

int16_t calcEscapeAngleFromRing16() {
  static const float step = 360.0f / 16.0f;

  float sumX = 0.0f;
  float sumY = 0.0f;

  bool detected = false;

  for (int i = 0; i < RING_LINE; i++) {
    int v = digitalRead(LINE[i]);
    if (v) {
      detected = true;

      // ラインがある方向（センサの向き）
      float ang = (i * step) * DEG_TO_RAD;   // CCW
      sumX += cos(ang);
      sumY += sin(ang);
    }
  }

  if (!detected){
	return -1;
  }

  // ライン方向（0〜360）
  float lineAng = atan2(sumY, sumX) * RAD_TO_DEG;
  if (lineAng < 0){
	lineAng += 360.0f;
  }

  // 逃げ方向 = ライン方向反転
  float escapeAng = lineAng + 180.0f;
  if (escapeAng >= 360.0f){
	escapeAng -= 360.0f;
  }
  return (int16_t)(escapeAng + 0.5f); // 四捨五入
}
