#include "UBTech.h"
#include "FS.h"

#define MAX_ACTION 26
#define MAX_POSES 30 
#define MAX_POSES_SIZE 40

#define MAX_COMBO 10
#define MAX_COMBO_SIZE 100

byte actionTable[MAX_ACTION][MAX_POSES][MAX_POSES_SIZE];
byte comboTable[MAX_COMBO][MAX_COMBO_SIZE];


SoftwareSerial ss14(12, 12, false, 256);

// UBTech servo(&ss14);  // Without debug
UBTech servo(&ss14, &Serial);  // Debug on Serial

int servoCnt = 0;
byte *retBuffer;

void setup() {
	delay(100);
	servo.setDebug(false);  // Disable servo debug first, enable it later if needed
	retBuffer = servo.retBuffer();
	Serial.begin(115200);
	Serial.println(F("\n\n\nUBTech Robot Control\n"));
	unsigned long actionSzie = sizeof(actionTable);
	servo.begin();
	// showServoInfo();
	// servo.setDebug(true);
	setupSampleData();
}


byte ledMode = 0;

byte ch, cmd;

void loop() {
	fx_remoteControl();
}

void fx_playDemo() {
	Serial.println("Go play demo");
	playAction(0);
}

void fx_remoteControl() {
	while (!Serial.available());
	cmd = Serial.read();
	delay(1);
	switch (cmd) {
		case 'C': // Check status
			showServoInfo();
			break;
		case 'M': // Move single servo
			fx_moveServo();
			break;
		case 'S': // Stop action (possible? or just go to standby)
			break;
		case 'U': // Unlock servo
			fx_unlockServo();
			break;
		case 'P': // Playback action Standard : 'A'-'Z', Combo : '0' - '9'
			fx_playAction();
			break;		
	}
	clearInputBuffer();
}

void clearInputBuffer() {
	while (Serial.available()) {
		Serial.read();
		delay(1);
	}
}

void showServoInfo() {
	Serial.println(F("\n\nAvailable servo:"));
	servoCnt = 0;
	for (int id = 1; id <= 16; id++) {
		if (id < 10) Serial.print('0');
		Serial.print(id);
		Serial.print(F(": "));
		if (servo.exists(id)) {
			servo.getPos(id);
			for (int i = 4; i <8; i++) {
				Serial.print(retBuffer[i] < 0x10 ? " 0" : " ");
				Serial.print(retBuffer[i], HEX);
			}
			Serial.print("  [");
			Serial.print(retBuffer[7], DEC);
			Serial.println("]");
			servoCnt++;
		} else {
			Serial.println("Missing");
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

void fx_moveServo() {
	int id = getServoId();
	switch (id) {
		case -1:
			Serial.print('M');
			Serial.write(0x01);
			break;
		case -2:
			Serial.print('M');
			Serial.write(0x02);
			break;
		case 0:
			Serial.print('M');
			Serial.write(0x03);
			break;
		default:
			int angle = -1;
			angle = Serial.parseInt();
			if ((angle >= 0) && (angle <= 240)) {
				servo.move(id, angle, 50);
				Serial.write('M');
				Serial.write(0x00);
				Serial.write(angle);
			} else {
				Serial.print('M');
				Serial.write(0x04);
			}
			break;
	}
}

void fx_unlockServo() {
	int id = getServoId();
	switch (id) {
		case -1:
			Serial.print('U');
			Serial.write(0x01);
			break;
		case -2:
			Serial.print('U');
			Serial.write(0x02);
			break;
		case 0:
			Serial.print('U');
			Serial.write(0x03);
			break;
		default:
			servo.getPos(id);
			Serial.print('U');
			Serial.write(0x00);
			Serial.write(retBuffer[7]);
			break;
	}
}


int getServoId() {
	if (!Serial.available()) return -1;
	int id = (byte) Serial.read();
	if ((id >= '0') && (id <= '9')) {
		id -= '0';
	} else if ((id >= 'A') && (id <= 'G')) {
		id = 10 + id - 'A';
	} else if ((id >= 'a') && (id <= 'g')) {
		id = 10 + id - 'a';
	} else {
		return -2;
	}
	return id;
}

void fx_playAction() {
	byte actionCode = 0;  // action 0, 'A' is standby action
	if (Serial.available()) {
		byte ch = Serial.read();
		if ((ch >= 'A') && (ch <= 'Z')) {
			actionCode = ch - 'A';
		} else {
			return; // Return for invalid action
		}
	}		
	playAction(actionCode);
}

void playAction(byte actionCode) {
	Serial.println("Play Action");
	servo.setDebug(true);
	for (int po = 0; po < MAX_POSES; po++) {
		int waitTime = actionTable[actionCode][po][32] * 256 + actionTable[actionCode][po][33];
		// End with all zero, so wait time at [32] [33] will be 0x00, 0x00
		if (waitTime == 0) break;
		for (int id = 1; id <= 16; id++) {\
			byte servoData = (id - 1) * 2;
			byte angle = actionTable[actionCode][po][servoData];
			byte time = actionTable[actionCode][po][servoData + 1];
			// max 240 degree, no action required if angle not changed, except first action
			if ((angle <= 0xf0) && (time > 0) &&
			    ((po == 0) || (angle != actionTable[actionCode][po-1][servoData]))) {
				servo.move(id, angle, time);
			}
		}
		delay(waitTime);
	}
	servo.setDebug(false);
}

void setupSampleData() {
	memset(actionTable, 0, sizeof(actionTable));

	/*
    A - standby  
	B - forward
	C - left
	D - right
	E - backword
	F - action
	*/

	byte poseArray[][16] = { {0x5a, 0x10, 0x3d, 
	                          0x5a, 0xa7, 0x72, 
	                          0x58, 0x3c, 0x48, 0x6e, 0x5f, 
							  0x5C, 0x78, 0x69, 0x46, 0x55}
						 };
	setFullPoses(0,0,poseArray[0], 50, 1000);

	setSamplePos(5,0,0x1D, 0x86, 0x00, 50, 1000);
	setSamplePos(5,1,0x0C, 0x00, 0x55, 30, 2000);
	setSamplePos(5,2,0x0C, 0x00, 0x03, 100, 3000);
	setSamplePos(5,3,0x0C, 0x00, 0x55, 100, 2000);
	setSamplePos(5,4,0x5A, 0x11, 0x3D, 100, 3000);

	byte poseG[][16] = { {0x5a, 0x10, 0x3d, 
						  0x5a, 0xa7, 0x72, 
						  0x58, 0x3c, 0x48, 0x6e, 0x5f, 
						  0x5C, 0x78, 0x69, 0x46, 0x55},
					     {0x26, 0x00, 0x1e,
	                      0x8b, 0xb1, 0x98,
						  0x5d, 0x3f, 0x00, 0xb4, 0x5c,
						  0x62, 0x77, 0xb4, 0x00, 0x53},
						 {0x5a, 0x10, 0x3d, 
						  0x5a, 0xa7, 0x72, 
						  0x58, 0x3c, 0x48, 0x6e, 0x5f, 
						  0x5C, 0x78, 0x69, 0x46, 0x55} 
	                   };

	setFullPoses(6,0,poseG[0], 150, 4000);
	setFullPoses(6,1,poseG[1], 150, 5000);
	setFullPoses(6,2,poseG[2], 150, 4000);

	setFullPoses(7,0,poseG[1], 150, 5000);

}

void setFullPoses(byte act, byte pos, byte angle[], byte time, int waitTime) {
	for (int i = 0; i < 16; i++) {
		actionTable[act][pos][2 * i] = angle[i];
		actionTable[act][pos][2 * i + 1] = time;
	}
	actionTable[act][pos][32] = (waitTime / 256);
	actionTable[act][pos][33] = (waitTime % 256);
}

void setSamplePos(byte act, byte pos, byte a1, byte a2, byte a3, byte time, int waitTime) {
	actionTable[act][pos][0] = a1;
	actionTable[act][pos][1] = time;
	actionTable[act][pos][2] = a2;
	actionTable[act][pos][3] = time;
	actionTable[act][pos][4] = a3;
	actionTable[act][pos][5] = time;
	actionTable[act][pos][32] = (waitTime / 256);
	actionTable[act][pos][33] = (waitTime % 256);
}
