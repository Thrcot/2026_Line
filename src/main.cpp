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
#define IN17 PB1
#define IN18 PB2
#define IN19 PB9

const int LINE[19] = {IN1, IN2, IN3, IN4, IN5, IN6, IN7, IN8, IN9, IN10, IN11, IN12, IN13, IN14, IN15, IN16, IN17, IN18, IN19};
#define RING_LINE 16
#define ALL_LINE 18

#define L1 PA11
#define L2 PA12

#define LEDCtrl PB4

HardwareSerial SerialPC(UART1_RX, UART1_TX);
HardwareSerial SerialMain(UART2_RX, UART2_TX);

int brightness = 230; //180
int threshold = 145; //95

#define LINE_INIT 0xAD
#define LINE_INFO 0xAE
#define LINE_OK 0xAF

const float STEP = 360.0f / RING_LINE;

float culcAngle();
void initLine();
void sendfloat(float f);

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

	for (int i = 0; i < 19; i++) {
		pinMode(LINE[i], INPUT_PULLUP);
	}
}

void loop() {
	float angle = 0.0;

	SerialPC.print(digitalRead(IN1));
	SerialPC.print(" ");
	SerialPC.print(digitalRead(IN2));
	SerialPC.print(" ");
	SerialPC.print(digitalRead(IN3));
	SerialPC.print(" ");
	SerialPC.print(digitalRead(IN4));
	SerialPC.print(" ");
	SerialPC.print(digitalRead(IN5));
	SerialPC.print(" ");
	SerialPC.print(digitalRead(IN6));
	SerialPC.print(" ");
	SerialPC.print(digitalRead(IN7));
	SerialPC.print(" ");
	SerialPC.print(digitalRead(IN8));
	SerialPC.print(" ");
	SerialPC.print(digitalRead(IN9));
	SerialPC.print(" ");
	SerialPC.print(digitalRead(IN10));
	SerialPC.print(" ");
	SerialPC.print(digitalRead(IN11));
	SerialPC.print(" ");
	SerialPC.print(digitalRead(IN12));
	SerialPC.print(" ");
	SerialPC.print(digitalRead(IN13));
	SerialPC.print(" ");
	SerialPC.print(digitalRead(IN14));
	SerialPC.print(" ");
	SerialPC.print(digitalRead(IN15));
	SerialPC.print(" ");
	SerialPC.print(digitalRead(IN16));
	SerialPC.print(" ");
	SerialPC.print(digitalRead(IN17));
	SerialPC.print(" ");
	SerialPC.print(digitalRead(IN18));
	SerialPC.print(" ");
	SerialPC.print(digitalRead(IN19));
	SerialPC.println();

	if (digitalRead(IN17) && digitalRead(IN19)) {
		angle = 180.0;
	} else if (digitalRead(IN17) && digitalRead(IN18)) {
		angle = 45.0;
	} else if (digitalRead(IN18) && digitalRead(IN19)) {
		angle = 315.0;
	} else if (digitalRead(IN17)) {
		angle = 90.0;
	} else if (digitalRead(IN18)) {
		angle = 0.0;
	} else if (digitalRead(IN19)) {
		angle = 270.0;
	} else if (digitalRead(IN1) || digitalRead(IN2) || digitalRead(IN3) || digitalRead(IN16) || digitalRead(IN15)) {
		angle = 180.0;
	} else {
		angle = -1.0;
	}
	SerialPC.println(angle);

	if (SerialMain.available()) {
		int cmd = SerialMain.read();
		SerialPC.println(cmd);
		digitalWrite(L1, !digitalRead(L1));
		sendfloat(angle);
	}
	delay(1);
}

void sendfloat(float f) {
	byte *data = (byte *)&f;
	for (int i = 0; i < 4; i++) {
		SerialMain.write(data[i]);
	}
	digitalWrite(L2, !digitalRead(L2));
	SerialPC.print(data[0]);
	SerialPC.print(" ");
	SerialPC.print(data[1]);
	SerialPC.print(" ");
	SerialPC.print(data[2]);
	SerialPC.print(" ");
	SerialPC.println(data[3]);
}

/*
float culcAngle() {
	int RINGLine[RING_LINE];
	int BrightLine[RING_LINE];
	int DarkLine[RING_LINE];
	int bright = 0;
	int dark = 0;

	for (int i = 0; i < RING_LINE; i++) {
		RINGLine[i] = digitalRead(LINE[i]);
		if (RINGLine[i] == 1) {
			BrightLine[bright] = i;
			bright++;
		} else {
			DarkLine[dark] = i;
			dark++;
		}
	}

	
	if (Line[16] == 1) {
		return 90.0;
	} else if (Line[17] == 1) {
		return 0.0;
	} else if (Line[18] == 1) {
		return 270.0;
	} else if ((Line[16] == 1) && (Line[18] == 1)) {
		return 180.0;
	}
	
}
*/

float culcAngle() {

    float sumX = 0;
    float sumY = 0;
    bool detected = false;

    // センサ読み取り
    for (int i = 0; i < RING_LINE; i++) {

        int v = digitalRead(LINE[i]);  // 白=1

        if (v == 1) {
            detected = true;

            float angle = i * STEP * DEG_TO_RAD; // CCW方向ラジアン
            sumX += cos(angle);
            sumY += sin(angle);
        }
    }

    // 白ラインが1つも無ければ -1 を返す
    if (!detected) return -1;

    // 重心角度（CCW）
    float angleCCW = atan2(sumY, sumX) * RAD_TO_DEG;
    if (angleCCW < 0) angleCCW += 360;

    // メインマイコン用にCWへ変換
    float angleCW = 360.0f - angleCCW;
    if (angleCW >= 360.0f) angleCW -= 360.0f;

    return angleCW;
}

void initLine() {
	for (int i = 0; i < 256; i++) {
		analogWrite(THRESHOLD, i);
		int val[ALL_LINE];
		for (int i = 0; i < ALL_LINE; i++) {
			val[i] = digitalRead(LINE[i]);
		}
	}
}