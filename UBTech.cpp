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
	// How to get current servo status?
	// Assume all unlocked as it is unlocked during power-off
	for (int id = 0; id <= MAX_SERVO_ID; id++ ) _servoLocked[id] = false;
}

void UBTech::detectServo(byte min, byte max) {
	memset(_servo, 0, MAX_SERVO_ID + 1);
	for (int i = min; i <= max; i++) {
		getVersion(i);
		if ((_retCnt > 0) && (_retBuf[2] == i) && (_retBuf[3] == 0xAA)) {
			_servo[i] = true;
		}
	}
}

bool UBTech::exists(byte id) {
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
    while ( ((millis() - startMs) < 500) && (!_ss->available()) ) ;
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
	_servoLocked[id] = true;
}

void UBTech::getPos(byte id) {
	resetCommandBuffer();
	_buf[2] = id;
	_buf[3] = 0x02;
	sendCommand();
	// servo will be unlocked after fire a get position command
	_servoLocked[id] = false;
}

void UBTech::setLED(byte id, byte mode) {
	resetCommandBuffer();
	_buf[2] = id;
	_buf[3] = 0x04;
	_buf[4] = mode;
	sendCommand();
}

