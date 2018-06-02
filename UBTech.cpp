#include "UBTech.h"

UBTech::UBTech(SoftwareSerial *ssData) 
{
    initObject(ssData, NULL);
}

UBTech::UBTech(SoftwareSerial *ssData, HardwareSerial *hsDebug) 
{
	initObject(ssData, hsDebug);
}

void UBTech::initObject(SoftwareSerial *ssData, HardwareSerial *hsDebug) {
    _ss = ssData;
	_dbg = hsDebug;
	_enableDebug = (_dbg != NULL);
	_arrayReady = false;
}

UBTech::~UBTech() {
    _ss = NULL;
	_dbg = NULL;
}

bool UBTech::setDebug(bool debug) {
	if (_dbg == NULL) return false;
	_enableDebug = debug;
	return _enableDebug;
}

void UBTech::init(byte max_id, byte maxCommandWaitMs, byte maxCommandRetry, byte maxDetectRetry) {
	// Avoid duplicate array creation
	if (!_arrayReady) {
		_max_id = max_id;
		setRetry(maxCommandWaitMs, maxCommandRetry, maxDetectRetry);
		int arraySize = max_id + 1;
		_servo = new bool[arraySize];
		_led = new byte[arraySize];
		 _isLocked = new bool[arraySize];
		_lastAngle = new byte[arraySize];
		_isServo = new bool[arraySize];
		_adjAngle = new uint16[arraySize];
		_servoCnt = 0;
		_arrayReady = true;
	}
}

void UBTech::setRetry(byte maxCommandWaitMs, byte maxCommandRetry, byte maxDetectRetry) {
	// max wait ms should be atleast 1ms
	_maxCommandWaitMs = (maxCommandWaitMs == 0 ?  DEFAULT_MAX_COMMAND_WAIT_MS : maxCommandWaitMs);
	_maxCommandRetry = maxCommandRetry;
	_maxTry = maxCommandRetry + 1;
	_maxDetectRetry = maxDetectRetry;
}

void UBTech::begin() {
    _ss->begin(SERVO_BAUD);
	delay(100);
	detectServo();
}

void UBTech::end() {
    _ss->end();
	for (int id =1; id <= _max_id; id++) {
		_servo[id] = false;
		_led[id] = 0;
		_isServo[id] = true;
		_isLocked[id] = false;
		_lastAngle[id] = 0xFF;
		_adjAngle[id] = 0x7F7F;
	}	
}

void UBTech::detectServo(byte min, byte max) {
	memset(_servo, 0, _max_id + 1);
	byte expCnt = max - min + 1;
	_servoCnt = 0;
	for (int id = min; id <= max; id++) {
		delay(1);
		getVersion(id);
		if ((_retCnt > 0) && (_retBuf[2] == id) && (_retBuf[3] == 0xAA)) {
			// new servo detected, assume unlock
			if (!_servo[id]) {
				_servo[id] = true;
				_servo[id] = true;
				_isServo[id] = true;
				_isLocked[id] = false;
				_lastAngle[id] = 0xFF;
				_adjAngle[id] = 0x7F7F;
				_led[id] = 0;
				_servoCnt++;
			}
		} else {
			_servo[id] = false;
		}
	}
	// Retry for missing servo
	int retryCnt = 0;
	while ((_servoCnt < expCnt) && (retryCnt < _maxDetectRetry)) {
		retryCnt++;			
		for (int id = min; id <= max; id++) {
			if (!_servo[id]) {
				delay(1);
				getVersion(id);
				if ((_retCnt > 0) && (_retBuf[2] == id) && (_retBuf[3] == 0xAA)) {
					// new servo detected, assume unlock
					if (!_servo[id]) {
						_servo[id] = true;
						_servo[id] = true;
						_isServo[id] = true;
						_isLocked[id] = false;
						_lastAngle[id] = 0xFF;
						_adjAngle[id] = 0x7F7F;
						_led[id] = 0;
						_servoCnt++;
					}
				}
			}
		}
	}

	if (_enableDebug) _dbg->printf("%d servo detected.\n", _servoCnt);
}

bool UBTech::exists(byte id) {
	if (id > _max_id) return false;
	return _servo[id];
}

void UBTech::showCommand()  {
    if (!_enableDebug) return;
    _dbg->printf("%08ld OUT>>", millis());
    for (int i = 0; i < 10; i++) {
        _dbg->print( (_buf[i] < 0x10 ? " 0" : " "));
        _dbg->print(_buf[i], HEX);
    }
    _dbg->println();
}

bool UBTech::sendCommand(bool expectReturn) {
	byte sum = 0;
	for (int i = 2; i < 8; i++) {
		sum += _buf[i];
	}
	_buf[8] = sum;
	if (_enableDebug) showCommand();

	_ss->flush();
	_ss->enableTx(true);
	_ss->write(_buf, 10);
	_ss->enableTx(false);
	if (expectReturn) return checkReturn();
	return true;
}

bool UBTech::checkReturn() {
    // unsigned long startMs = millis();
    // while ( ((millis() - startMs) < COMMAND_WAIT_TIME) && (!_ss->available()) ) ;
    unsigned long endMs = millis() + _maxCommandWaitMs;
    while ( (millis() < endMs) && (!_ss->available()) ) ;
    if (!_ss->available()) return false;

    resetReturnBuffer();
    byte ch;

    if (_enableDebug) {
        _dbg->printf("%08ld IN>>>", millis());
    }
    while (_ss->available()) {
        ch =  (byte) _ss->read();
        _retBuf[_retCnt++] = ch;
        if (_enableDebug) {
			_dbg->printf(" %02X", ch);
        }
    }
	// TODO: Think about any better solution to initiate the bus.
	// Special handling for missing frist byte.
	// In some situation, espeically right after reset, the first return byte will be missing.
	// This is a temporary solution to handle FC / FA return with missing byte.
	if ((_retCnt == 9) && (_retBuf[8]==0xED)) {
		byte b1 = 0;

		// Now, only handle those 0x?F
		if ((_retBuf[0] % 0x10) == 0x0F) {
			b1 = 0xF0 + (_retBuf[0] >> 4);
		}
		// add handling for other code  if needed

		if (b1) {
			for (int i = _retCnt; i > 0; i--) {
				_retBuf[i] = _retBuf[i-1];
			}
			_retBuf[0] = b1;
			_retCnt++;
			if (_enableDebug) _dbg->printf("  **Missing byte added: [%02X]",b1);
		}
	}

    if (_enableDebug) _dbg->println();
    return true;
}

// FC CF {id} 01 00 00 00 00 {sum} ED
// FC CF {id} {V-1} {V-2} {V-3} {V-4} {sum} ED
void UBTech::getVersion(byte id) {
	resetCommandBuffer();
	_buf[0] = 0xFC;
	_buf[1] = 0xCF;
	_buf[2] = id;
	_buf[3] = 0x01;
	sendCommand();
}

// FA AF {id} 01 {angle} {time} {T-1} {T-2} {sum} ED
// {AA + id}
bool UBTech::move(byte id, byte angle, byte time) {
	if (!exists(id)) return false;

	int tryCnt = 0;

	while (tryCnt++ < _maxTry) {
		resetCommandBuffer();
		_buf[2] = id;
		_buf[3] = 0x01;
		_buf[4] = angle;
		_buf[5] = time;
		_buf[6] = 0x00;
		_buf[7] = time;  

		sendCommand(true);

		if ((_retCnt == 1) && (_retBuf[0] == (0xAA + id))) {
			_isLocked[id] = true;
			_lastAngle[id] = angle;
			return true;
		}
	}

	// Failed even after retry
	if (_enableDebug) {
		_dbg->printf("Move failed: id=%d ; angle=%d ; time=%d ; MaxTry=%d\n", id, angle, time, _maxTry);
	}
	// servo will be locked after fire a move command
	return false;
}


// FA AF {id} 01 00 00 00 00 {sum} ED
// FA AF {id} 00 {angle} 00 {real} {sum} ED
byte UBTech::getPos(byte id, bool lockAfterGet) {
	if (!exists(id)) return 0xFF;
	int tryCnt = 0;
	while (tryCnt++ < _maxTry) {
		resetCommandBuffer();
		_buf[2] = id;
		_buf[3] = 0x02;
		sendCommand();
		if (_retCnt == 10) break;
	}
	if (_retCnt != 10) {
		// What can I do if it has not return the position
		return 0xFF;
	}

	byte angle = _retBuf[7];

	if (lockAfterGet) {
		move(id, angle, 0);
	} else {
		// servo will be unlocked after fire a get position command
		_isLocked[id] = false;
	}
	return angle;
}


// FA AF {id} D4 00 00 00 00 {sum} ED
// FA AF {AA + id} D4 00 00 {A-1} {A-2} {sum} ED
uint16 UBTech::getAdjAngle(byte id) {
	if (!exists(id)) return 0x7F7F;
	int tryCnt = 0;
	while (tryCnt++ < _maxTry) {
		resetCommandBuffer();
		_buf[2] = id;
		_buf[3] = 0xD4;
		sendCommand();
		if (_retCnt == 10) break;
	}
	if (_retCnt != 10) {
		// What can I do if it has not return the position
		return 0x7F7F;
	}
	_adjAngle[id] = _retBuf[6] * 256 + _retBuf[7];
	return _adjAngle[id];
}

// FA AF {id} D2 00 00 {A-1} {A-2} {sum} ED
// FA AF {AA + id} 00 00 00 00 {sum} ED
uint16 UBTech::setAdjAngle(byte id, uint16 adjValue) {
	if (!exists(id)) return 0x7F7F;
	int tryCnt = 0;
	while (tryCnt++ < _maxTry) {
		resetCommandBuffer();
		_buf[2] = id;
		_buf[3] = 0xD2;
		_buf[6] = adjValue / 256;
		_buf[7] = adjValue % 256;
		sendCommand();
		if (_retCnt == 10) break;
	}
	return getAdjAngle(id);
}

// FA AF {id} 04 {0/1} 00 00 00 {sum} ED
// {AA + id}
void UBTech::setLED(byte id, byte mode) {
	if ((id) && !exists(id)) return;
	resetCommandBuffer();
	_buf[2] = id;
	_buf[3] = 0x04;
	_buf[4] = mode;
	sendCommand();
	if (id) {
		if ((_retCnt > 0) && (_retBuf[0] == (0xAA + id))) {
			_led[id] = mode;
		}
	} else {
		// Ignore return and assume all OK
		for (int id =1; id <= _max_id; id++) {
			if (exists(id)) _led[id] = mode;
		}
	}
}

int UBTech::execute(byte cmd[], byte result[]) {
	resetCommandBuffer();
	memcpy(_buf, cmd, 8);
	sendCommand();
	if (!_retCnt) return false;
	// Checking for Angle / Lock / LED
	// Ignore for id = 0 ?
	if ((cmd[0] == 0xFA) && (cmd[1] == 0xAF)) {
		byte code = cmd[3];
		switch (code) {
			case 0x01:
				if ((cmd[2] != 0) && (_retBuf[0] == (0xAA + cmd[2]))) {
					_lastAngle[cmd[2]] = cmd[4];
					_isLocked[cmd[2]] = true;
				}
				break;
			case 0x02:
				if ((cmd[2] != 0) && (_retCnt == 10)) {
					_isLocked[cmd[2]] = false;
				}
				break;
			case 0x04:
				if ((cmd[2] != 0) && (_retBuf[0] == (0xAA + cmd[2]))) {
					_led[cmd[2]] = cmd[4];
				}
				break;
		}

	}

	memcpy(result, _retBuf, _retCnt);
	return _retCnt;
}

