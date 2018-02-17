#include "UBTech.h"
#include "FS.h"

#define MAX_ACTION 26
#define MAX_POSES 30 
// #define MAX_POSES_SIZE 40
#define MAX_POSES_SIZE 20
// PoseInfo (20 bytes)
// 0 : enabled
// 1~16 : 16 servo angle, 0xFF = no change
// 17 : Time in servo command
// 18~19 : waiting time in millis

#define ENABLE_FLAG    0
#define ID_OFFSET      0
#define EXECUTE_TIME   17
#define WAIT_TIME_HIGH 18
#define WAIT_TIME_LOW  19


#define MAX_COMBO 10
#define MAX_COMBO_SIZE 100

byte actionTable[MAX_ACTION][MAX_POSES][MAX_POSES_SIZE];
byte comboTable[MAX_COMBO][MAX_COMBO_SIZE];


// SoftwareSerial ss14(14, 14, false, 256);
SoftwareSerial ss12(12, 12, false, 256);

// UBTech servo(&ss14);  // Without debug
UBTech servo(&ss12, &Serial);  // Debug on Serial

int servoCnt = 0;
byte *retBuffer;

void setup() {
	// Delay 2s to wait for all servo started
	analogWrite(13, 32);
	delay(2000);
	servo.setDebug(false);  // Disable servo debug first, enable it later if needed
	retBuffer = servo.retBuffer();
	Serial.begin(115200);
	Serial.println(F("\n\n\nUBTech Robot Control\n"));
	unsigned long actionSzie = sizeof(actionTable);
	servo.begin();
	// showServoInfo();
	// servo.setDebug(true);
	setupSampleData();
	analogWrite(13, 255);
	Serial.println("Control board ready");
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
		case 'A': // Get Angle
			cmd_GetServoAngleHex();
			break;
		case 'a': // Get Angle
			cmd_GetServoAngle();
			break;
	

		case 'B':
			servo.setDebug(true);
			break;
		case 'b':
			servo.setDebug(false);
			break;

		case 'D': // Lock servo
			cmd_DetectServoHex();
			break;
		case 'd': // Lock servo
			cmd_DetectServo();
			break;

		case 'G':
			cmd_GetActionDataHex();
			break;			

		case 'L': // Lock servo
			cmd_LockServoHex(true);
			break;

		case 'l': // Lock servo
			cmd_LockServo(true);
			break;

		case 'M': // Move single servo
			cmd_MoveServoHex();
			break;
		case 'm': // Move single servo
			cmd_moveServo();
			break;
		case 'R': // Read servo angle without changing the locking
			fx_readServo();
			break;
		case 'S': // Stop action (possible? or just go to standby)
			break;
		case 'U': // Unlock servo
			cmd_LockServoHex(false);
			break;

		case 'u': // Lock servo
			cmd_LockServo(false);
			break;

		case 'P': // Playback action Standard : 'A'-'Z', Combo : '0' - '9'
			fx_playAction();
			break;		
		case 'X': 
			fx_resetConnection();
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

void serialPrintByte(byte data) {
	if (data < 0x10) Serial.print("0");
	Serial.print(data, HEX);
}


void cmd_GetServoAngleHex() {
	byte outBuffer[32];
	for (int id = 1; id <= 16; id++) {
		int pos = 2 * (id - 1);
		if (servo.exists(id)) {
			outBuffer[pos] = servo.getPos(id);
			outBuffer[pos+1] = (servo.isLocked(id) ? 1 : 0);
		} else {
			outBuffer[pos] = 0xFF;
			outBuffer[pos+1] = 0;
		}
	}
	Serial.write(outBuffer, 32);
}

void cmd_GetServoAngle() {
	Serial.println(F("\nServo Angle:\n"));
	for (int id = 1; id <= 16; id++) {
		Serial.print(F("Servo "));
		Serial.print(id);
		Serial.print(": ");
		if (servo.exists(id)) {
			byte angle = servo.getPos(id);
			Serial.print(angle, DEC);
			Serial.print(" [");
			serialPrintByte(angle);
			Serial.print("]  ");
			if (servo.isLocked(id)) {
				Serial.print(" Locked");
			}
			Serial.println();
		} else {
			Serial.println("Not Available");
		}
	}
}

void cmd_DetectServoHex() {
	servo.detectServo(1,16);
	cmd_GetServoAngleHex();
}

void cmd_DetectServo() {
	servo.detectServo(1,16);
}


// Return: 
//   0 - L/U
//   1 - log count
//   (2n)   - servo id
//   (2n+1) - angle / 0xFF for error
void cmd_LockServoHex(bool goLock) {
	byte result[40];
	byte cnt = 0;
	char mode = (goLock ? 'L' : 'U');
	result[0] = mode;
	while (Serial.available()) {
		byte id = Serial.read();
		if ((cnt == 0) && (id == 0)) {
			// Only 1 paramter 0x00 for lock/unlock all
			if (!Serial.available()) {
				for (int servoId = 1; servoId <= 16; servoId++)  {
					result[2*servoId] = servoId;
					if (servo.exists(servoId)) {
						result[2*servoId+1] = servo.getPos(servoId, goLock);
					} else {
						result[2*servoId+1] = 0xFF;
					}
				}
				cnt = 16;
			} 
			break;
		}
		cnt++;
		result[2*cnt] = id;
		if ((id >= 1) && (id <= 16) && servo.exists(id)) {
			result[2*cnt+1] = servo.getPos(id, goLock);
		} else {
			result[2*cnt+1] = 0xFF;
		}
		// For safety, not reasonable to have more than MAC servo
		if (cnt >= 16) break;
	}
	result[1] = cnt;
	cnt = 2 * (cnt + 1);
	Serial.write(result, cnt);
}

void cmd_LockServo(bool goLock) {
	int id = getServoId();
	char mode = (goLock ? 'l' : 'u');
	switch (id) {
		case -1:
			Serial.print(mode);
			Serial.write(0x01);
			break;
		case -2:
			Serial.print(mode);
			Serial.write(0x02);
			break;
		case 0:
			Serial.print(mode);
			Serial.write(0x00);
			Serial.write(0x00);
			for (int servoId = 1; servoId <= 16; servoId++)  {
				if (servo.exists(servoId)) servo.getPos(servoId, goLock);
			}
			break;
		default:
			Serial.print(mode);
			Serial.write(0x00);
			Serial.write(id);
			byte angle;
			if ((id >= 1) && (id <= 16) && servo.exists(id)) {
				angle = servo.getPos(id, goLock);
			} else {
				angle = 0xFF;
			}
			Serial.write(angle);
			break;
	}
}



void cmd_GetActionDataHex() {
	byte *ptr = (byte *)actionTable;
	long size = MAX_POSES * MAX_POSES_SIZE;
	for(int action = 0; action < MAX_ACTION; action ++) {
		Serial.write(ptr, size);
		ptr += size;
		delay(1);  // add 1 ms delay for processing the data
	}
}

#define MOVE_OK 				0x00
#define MOVE_ERR_PARM_CNT		0x01
#define MOVE_ERR_PARM_VALUE     0x02
#define MOVE_ERR_PARM_CONTENT   0x03
#define MOVE_ERR_PARM_END		0x04
#define MOVE_ERR_PARM_ALL_CNT	0x11
#define MOVE_ERR_PARM_ALL_ANGLE	0x12
#define MOVE_ERR_PARM_ONE_ID	0x21
#define MOVE_ERR_PARM_ONE_ANGLE	0x22
#define MOVE_ERR_PARM_DUP_ID	0x23



// Input id, angle, time
// Output - action, status, # servo, id
void cmd_MoveServoHex() {
	byte result[20];
	int moveCount = goMoveServoHex(result);
	Serial.write(result, moveCount + 3);
}

int goMoveServoHex(byte *result) {
	result[0] = 'M';
	result[1] = 0x00;
	result[2] = 0x00;
	byte inBuffer[51];  // Max 16 servo + end mark: 17 * 3 = 51
	byte inCount = 0;
	byte moveData[16][3];
	byte moveCnt = 0;
	while (Serial.available()) {
		if (inCount >= 51) {
			result[1] = MOVE_ERR_PARM_CNT;
			return 0;	
		}
		inBuffer[inCount++] = (byte) Serial.read();
	}
	return moveMultiServo(inCount, inBuffer, result);
}


void cmd_moveServo() {
	byte result[20];
	int moveCount = goMoveServo(result);
	Serial.print("\nMove - ");
	if (result[1]) {
		Serial.print(" Error : ");
		serialPrintByte(result[1]);
		Serial.println();
		return;
	}
	Serial.print(moveCount);
	Serial.print(" servo moved: ");
	for (int i = 0; i < moveCount; i++) {
		Serial.print(" ");
		Serial.print(result[ 3 + i]);
	}
	Serial.println();
}

int SerialParseInt() {
	if (!Serial.available()) return -1;
	ch = Serial.peek();
	if ((ch < '0') || (ch > '9')) return -2;
	int data = Serial.parseInt();
	return data;
}

int goMoveServo(byte *result)
{
	result[0] = 'M';
	result[1] = 0x00;
	result[2] = 0x00;
	byte inBuffer[51];  // Max 16 servo + end mark: 17 * 3 = 51
	byte inCount = 0;
	byte moveData[16][3];
	byte moveCnt = 0;
	while (Serial.available()) {
		if (inCount >= 51) {
			result[1] = MOVE_ERR_PARM_CNT;
			return 0;	
		}
		int value = SerialParseInt();
		if ((value < 0) || (value > 255)) {
			result[1] = MOVE_ERR_PARM_VALUE;
			return 0;	
		}
		inBuffer[inCount++] = (byte) value;

		if (Serial.available()) {
			if ((Serial.peek() == 0x0A) || (Serial.peek() == 0x0D)) {
				break;
			}
			if (Serial.peek() != ',') {
				result[1] = MOVE_ERR_PARM_CONTENT;
				return 0;	
			}
			// Read the ',' separator
			Serial.read();  
		} 
	}
	moveMultiServo(inCount, inBuffer, result);
}

int moveMultiServo(int inCount, byte* inBuffer, byte *result) {
	if ((inCount < 6) || (inCount % 3 != 0)) {
		result[1] = MOVE_ERR_PARM_CNT;
		return 0;
	}
	if ((inBuffer[inCount-1] != 0x00) || (inBuffer[inCount-2] != 0x00) || (inBuffer[inCount-3] != 0x00)) {
		result[1] = MOVE_ERR_PARM_END;
		return 0;
	}

	byte moveData[16][3];
	byte moveCnt = 0;
	if (inBuffer[0] == 0) {
		if (inCount != 6) {
			result[1] = MOVE_ERR_PARM_ALL_CNT;
			return 0;
		}
		if (inBuffer[1] > 240) {
			result[1] = MOVE_ERR_PARM_ALL_ANGLE;
			return 0;
		}
		moveCnt = 0;
		for (int id = 1; id <= 16; id++) {
			if (servo.exists(id)) {
				moveData[moveCnt][0] = id;
				moveData[moveCnt][1] = inBuffer[1];
				moveData[moveCnt][2] = inBuffer[2];
				moveCnt++;
			}
		}
	} else {
		moveCnt = 0;
		for (int i = 0; i < inCount - 3; i += 3) {
			int id = inBuffer[i];
			if ((id == 0) || (id > 16)) {
				result[1] = MOVE_ERR_PARM_ONE_ID;
				return 0;
			}
			if (inBuffer[i + 1] > 240) {
				result[1] = MOVE_ERR_PARM_ONE_ANGLE;
				return false;
			}
			bool servoFound = false;
			for (int j = 0; j < moveCnt; j++) {
				if (moveData[j][0] == id) {
					result[1] = MOVE_ERR_PARM_DUP_ID;
					return false;
				}
			}
			if (servo.exists(id)) {
				moveData[moveCnt][0] = id;
				moveData[moveCnt][1] = inBuffer[i + 1];
				moveData[moveCnt][2] = inBuffer[i + 2];
				moveCnt++;
			}
		}
	}
	result[1] = 0x00;
	result[2] = moveCnt;
	if (!moveCnt) return 0;

	for (int i = 0; i < moveCnt; i++) {
		servo.move(moveData[i][0], moveData[i][1], moveData[i][2]);
		result[i+3] = moveData[i][0];
	}
	return moveCnt;
}

void showServoInfoHex() {
	servoCnt = 0;
	byte outBuffer[16];
	for (int id = 1; id <= 16; id++) {
		if (servo.exists(id)) {
			outBuffer[id-1] = servo.getPos(id);
		} else {
			outBuffer[id-1] = 0xFF;
		}
	}
	Serial.write(outBuffer, 16);
}

void showServoInfo() {
	Serial.println(F("\n\nAvailable servo:"));
	servoCnt = 0;
	for (int id = 1; id <= 16; id++) {
		if (id < 10) Serial.print('0');
		Serial.print(id);
		Serial.print(F(": "));
		if (servo.exists(id)) {
			byte angle = servo.getPos(id);
			for (int i = 4; i <8; i++) {
				Serial.print(retBuffer[i] < 0x10 ? " 0" : " ");
				Serial.print(retBuffer[i], HEX);
			}
			Serial.print("  [");
			Serial.print(angle, DEC);
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

void fx_detectServoHex() {
	servo.detectServo(1,16);
	showServoInfoHex();
}

void fx_detectServo() {
	Serial.println("Re-detect servo, please wait for few seconds");
	servo.detectServo(1,16);
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

void fx_readServo() {
	int id = getServoId();
	switch (id) {
		case -1:
			Serial.print('R');
			Serial.write(0x01);
			break;
		case -2:
			Serial.print('R');
			Serial.write(0x02);
			break;
		case 0:
			Serial.print('R');
			Serial.write(0x03);
			break;
		default:
			byte angle = servo.getPos(id);
			Serial.print('R');
			Serial.write(0x00);
			Serial.write(id);
			Serial.println(angle);
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

/*
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
*/
void playAction(byte actionCode) {
	Serial.println("Play Action");
	servo.setDebug(true);
	for (int po = 0; po < MAX_POSES; po++) {
		int waitTime = actionTable[actionCode][po][WAIT_TIME_HIGH] * 256 + actionTable[actionCode][po][WAIT_TIME_LOW];
		// End with all zero, so wait time will be 0x00, 0x00
		if (waitTime == 0) break;
		byte time = actionTable[actionCode][po][EXECUTE_TIME];
		if (time > 0) {
			for (int id = 1; id <= 16; id++) {
				byte angle = actionTable[actionCode][po][ID_OFFSET + id];
				// max 240 degree, no action required if angle not changed, except first action
				if ((angle <= 0xf0) && 
					((po == 0) || (angle != actionTable[actionCode][po-1][ID_OFFSET + id]))) {
					servo.move(id, angle, time);
				}
			}
		}
		delay(waitTime);
	}
	servo.setDebug(false);
}

void fx_resetConnection() {
	Serial.println("Reset servo connection");
	servo.end();
	delay(100);
	servo.begin();
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
/*
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
*/
void setFullPoses(byte act, byte pos, byte angle[], byte time, int waitTime) {
	byte *ptr = &actionTable[act][pos][0];
	memset(ptr, 0xFF, MAX_POSES_SIZE);
	actionTable[act][pos][ENABLE_FLAG] = 0x01;
	for (int i = 1; i <= 16; i++) {
		actionTable[act][pos][ID_OFFSET + i] = angle[i-1];
	}
	actionTable[act][pos][EXECUTE_TIME] = time;
	actionTable[act][pos][WAIT_TIME_HIGH] = (waitTime / 256);
	actionTable[act][pos][WAIT_TIME_LOW] = (waitTime % 256);
}

void setSamplePos(byte act, byte pos, byte a1, byte a2, byte a3, byte time, int waitTime) {
	byte *ptr = &actionTable[act][pos][0];
	memset(ptr, 0xFF, MAX_POSES_SIZE);
	actionTable[act][pos][ENABLE_FLAG] = 0x01;
	actionTable[act][pos][ID_OFFSET + 1] = a1;
	actionTable[act][pos][ID_OFFSET + 2] = a2;
	actionTable[act][pos][ID_OFFSET + 3] = a3;
	actionTable[act][pos][EXECUTE_TIME] = time;
	actionTable[act][pos][WAIT_TIME_HIGH] = (waitTime / 256);
	actionTable[act][pos][WAIT_TIME_LOW] = (waitTime % 256);
}
