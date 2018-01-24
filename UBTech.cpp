#include "ubtech.h"
UBTech::UBTech(byte dataPin) 
{
    _hsDebug = NULL;
    _serialDebug = false;
    initObject(dataPin);
}

UBTech::UBTech(byte dataPin, HardwareSerial *hsDebug) 
{
    _hsDebug = hsDebug;
    _serialDebug = true;
	initObject(dataPin);
}

void UBTech::initObject(byte dataPin) {
    _dataPin = dataPin;
    SoftwareSerial ss(dataPin, dataPin, false, 256);
    _ss = &ss;
}

UBTech::~UBTech() {
    _ss = NULL;
}

void UBTech::begin() {
    _ss->begin(SERVO_BAUD);
	detectServo();
}

void UBTech::detectServo() {
	memset(_servo, 0, MAX_SERVO_ID + 1);
	for (int i = 1; i <= MAX_SERVO_ID; i++) {
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
	if (_serialDebug) showCommand();
	_ss->flush();
	_ss->enableTx(true);
	_ss->write(_buf, 10);
	_ss->enableTx(false);
	if (expectReturn) return checkReturn();
	return true;
}

void UBTech::showCommand()  {
    if (!_serialDebug) return;
    _hsDebug->print(millis());
    _hsDebug->print(" OUT>>");
    for (int i = 0; i < 10; i++) {
        _hsDebug->print( (_buf[i] < 0x10 ? " 0" : " "));
        _hsDebug->print(_buf[i], HEX);
    }
    _hsDebug->println();
}

bool UBTech::checkReturn() {
    unsigned long startMs = millis();
    resetReurnBuffer();
    byte ch;
    while ( ((millis() - startMs) < 500) && (!_ss->available()) ) ;
    if (!_ss->available()) return false;
    if (_serialDebug) {
        _hsDebug->print(millis());
        _hsDebug->print(" IN>>>");
    }
    while (_ss->available()) {
        ch =  (byte) _ss->read();
        _retBuf[_retCnt++] = ch;
        if (_serialDebug) {
            _hsDebug->print((ch < 0x10 ? " 0" : " "));
            _hsDebug->print(ch, HEX);
        }
    }
    if (_serialDebug) _hsDebug->println();
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

void UBTech::goAngle(byte id, byte angle, byte time) {
	resetCommandBuffer();
	_buf[2] = id;
	_buf[3] = 0x01;
	_buf[4] = angle;
	_buf[5] = time;
	_buf[6] = 0x00;
	_buf[7] = time;  
	sendCommand();
}

void UBTech::getPos(byte id) {
	resetCommandBuffer();
	_buf[2] = id;
	_buf[3] = 0x02;
	sendCommand();
}

void UBTech::setLED(byte id, byte mode) {
	resetCommandBuffer();
	_buf[2] = id;
	_buf[3] = 0x04;
	_buf[4] = mode;
	sendCommand();
}

