#include "UBTech.h"
SoftwareSerial ss14(14, 14, false, 256);

// UBTech servo(&ss14);  // Without debug
UBTech servo(&ss14, &Serial);  // Debug on Serial

int servoCnt = 0;

void setup() {
	delay(100);
	Serial.begin(115200);
	Serial.println(F("\n\n\nUBTech Servo Tester\n"));
	servo.init(16);
	servo.setDebug(false);  // Do not view servo detction here
	servo.begin();
	servo.setDebug(true);
	Serial.println(F("\n\nAvailable servo:"));
	servoCnt = 0;
	for (int id = 1; id <= 16; id++) {
		if (servo.exists(id)) {
			Serial.print(F("Servo ID="));
			Serial.print(id);
			Serial.println(F(" is avalable."));
			servo.setLedOff(id); // turn the led off (default on when power on)
			servoCnt++;
		}
	}
	Serial.println();
	if (servoCnt > 0) {
		Serial.print(servoCnt);
	} else {
		Serial.print(F("No "));
	}
	Serial.print(F(" servo detected."));
}

byte ledMode = 0;
byte action = 0;

void loop() {
	fx_checkServo();
	fx_action();
	delay(5000);
}

// Right hand only
void fx_action() {
	if (!(servo.exists(1) && servo.exists(2) && servo.exists(3))) return;
	for (action = 0; action < 4; action++) {
		switch (action) {
			case 0:
				servo.move(1, 0x1D, 50);
				servo.move(2, 0x86, 50);
				servo.move(3, 0x00, 50);
				delay(1000);
			break;
			case 1:
				servo.move(1, 0x0C, 50);
				servo.move(2, 0x00, 50);
				servo.move(3, 0x55, 50);
				delay(2000);
			break;
			case 2:
				servo.move(1, 0x0C, 100);
				servo.move(2, 0x00, 100);
				servo.move(3, 0x03, 100);
				delay(3000);
			break;
			case 3:
				servo.move(1, 0x5A, 100);
				servo.move(2, 0x11, 100);
				servo.move(3, 0x3D, 100);
				delay(3000);
			break;
		}
	}
}

void fx_checkServo() {
	if (!servoCnt) return;
	ledMode = 1 - ledMode;
	Serial.print("\nServo version / Position & LED = ");
	Serial.println(ledMode);
	byte *retBuffer;
	retBuffer = servo.retBuffer();
	for (int id = 1; id <= 16; id++) {
		if (servo.exists(id)) {
			Serial.print("Servo ");
			Serial.print(id);
			servo.getVersion(id);
			for (int i = 4; i <8; i++) {
				Serial.print(retBuffer[i] < 0x10 ? " 0" : " ");
				Serial.print(retBuffer[i], HEX);
			}
			Serial.print("  at ");
			servo.getPos(id);
			for (int i = 4; i <8; i++) {
				Serial.print(retBuffer[i] < 0x10 ? " 0" : " ");
				Serial.print(retBuffer[i], HEX);
			}
			servo.setLED(id, ledMode);
			Serial.println();
		}

	}
}