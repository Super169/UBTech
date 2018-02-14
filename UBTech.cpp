#include "UBTech.h"

UBTech::UBTech(SoftwareSerial *ssData) 
{
    _hs = NULL;
    _enableDebug = false;
    initObject(ssData);
}

UBTech::UBTech(SoftwareSerial *ssData, HardwareSerial *hsDebug) 
{
    _hs = hsDebug;
    _enableDebug = true;
	initObject(ssData);
}

void UBTech::initObject(SoftwareSerial *ssData) {
    _ss = ssData;
}

UBTech::~UBTech() {
    _ss = NULL;
}

bool UBTech::setDebug(bool debug) {
	if (_hs == NULL) return false;
	_enableDebug = debug;
	return _enableDebug;
}

void UBTech::begin() {
    _ss->begin(SERVO_BAUD);
	delay(100);
	detectServo();
}


void UBTech::end() {
    _ss->end();
	for (int id =1; id <= MAX_SERVO_ID; id++) {
		_servo[id] = false;
		_isServo[id] = true;
		_isLocked[id] = false;
		_lastAngle[id] = 0xFF;
	}	
}

void UBTech::detectServo(byte min, byte max) {
	memset(_servo, 0, MAX_SERVO_ID + 1);
	for (int id = min; id <= max; id++) {
		getVersion(id);
		if ((_retCnt > 0) && (_retBuf[2] == id) && (_retBuf[3] == 0xAA)) {
			// new servo detected, assume unlock
			if (!_servo[id]) {
				_servo[id] = true;
				_isServo[id] = true;
				_isLocked[id] = false;
				_lastAngle[id] = 0xFF;
			}
		} else {
			_servo[id] = false;
		}
	}
}

bool UBTech::exists(byte id) {
	if (id > MAX_SERVO_ID) return false;
	return _servo[id];
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

void UBTech::showCommand()  {
    if (!_enableDebug) return;
    _hs->print(millis());
    _hs->print(" OUT>>");
    for (int i = 0; i < 10; i++) {
        _hs->print( (_buf[i] < 0x10 ? " 0" : " "));
        _hs->print(_buf[i], HEX);
    }
    _hs->println();
}

bool UBTech::checkReturn() {
    unsigned long startMs = millis();
    resetReurnBuffer();
    byte ch;
    while ( ((millis() - startMs) < 600) && (!_ss->available()) ) ;
    if (!_ss->available()) return false;
    if (_enableDebug) {
        _hs->print(millis());
        _hs->print(" IN>>>");
    }
    while (_ss->available()) {
        ch =  (byte) _ss->read();
        _retBuf[_retCnt++] = ch;
        if (_enableDebug) {
            _hs->print((ch < 0x10 ? " 0" : " "));
            _hs->print(ch, HEX);
        }
		// extra delay to make sure transaction completed 
		// ToDo: check data end
		if (!_ss->available()) delay(1);
    }
    if (_enableDebug) _hs->println();
    return true;
}

void UBTech::getVersion(byte id) {
	resetCommandBuffer();
	_buf[0] = 0xFC;
	_buf[1] = 0xCF;
	_buf[2] = id;
	_buf[3] = 0x01;
	sendCommand();
}

void UBTech::move(byte id, byte angle, byte time) {
	resetCommandBuffer();
	_buf[2] = id;
	_buf[3] = 0x01;
	_buf[4] = angle;
	_buf[5] = time;
	_buf[6] = 0x00;
	_buf[7] = time;  
	sendCommand();
	// servo will be locked after fire a move command
	_isLocked[id] = true;
	_lastAngle[id] = angle;
}

byte UBTech::getPos(byte id, bool lockAfterGet) {
	int tryCnt = 0;
	while (tryCnt++ < 3) {
		resetCommandBuffer();
		_buf[2] = id;
		_buf[3] = 0x02;
		sendCommand();
		if (_retCnt == 10) break;
	}
	if (!_retCnt == 0x00) {
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

void UBTech::setLED(byte id, byte mode) {
	resetCommandBuffer();
	_buf[2] = id;
	_buf[3] = 0x04;
	_buf[4] = mode;
	sendCommand();
}

