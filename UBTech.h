#ifndef _UBTECH_H_
#define _UBTECH_H_

#include <ESP8266WiFi.h>
#include "HardwareSerial.h"
#include "SoftwareSerial.h"

#define SERVO_BAUD 115200

#define COMMAND_BUFFER_SIZE 10
#define RETURN_BUFFER_SIZE 	20  // Actually, 10 is enough, just for saftey

// #define DEFAULT_MAX_TRY_GETPOS		10
#define DEFAULT_MAX_DETECT_RETRY		2
#define DEFAULT_MAX_COMMAND_WAIT_MS		2
#define DEFAULT_MAX_COMMAND_RETRY       10

const byte SERVO_CMD[] = {0xFA, 0xAF,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 0xED};
const byte JIMU_VERSION[] = {0xFC, 0xCF,0x00,0xAA,0x41, 0x16, 0x51, 0x01, 0x00, 0xED};

class UBTech {
    public:
        UBTech(SoftwareSerial *ssData);
		UBTech(SoftwareSerial *ssData, HardwareSerial *hsDebug);
        ~UBTech();
		bool setDebug(bool debug);
        void init(byte max_id) { return init(max_id, DEFAULT_MAX_COMMAND_WAIT_MS, DEFAULT_MAX_COMMAND_RETRY, DEFAULT_MAX_DETECT_RETRY); }
        void init(byte max_id, byte maxCommandWaitMs, byte maxCommandRetry, byte maxDetectRetry);
		void setRetry(byte maxCommandWaitMs, byte maxCommandRetry, byte maxDetectRetry);
        void begin();
        void end();
        void getVersion(byte id);
        bool move(byte id, byte angle, byte time);
		byte lock(byte id) { return getPos(id, true); }
		void lockAll() { for (int id = 1; id <= _max_id; id++) getPos(id, true); }
		byte unlock(byte id) { return getPos(id, false); }
        byte getPos(byte id) { return getPos(id, _isLocked[id]); }
		byte getPos(byte id, bool lockAfterGet); 
		byte getPos(byte id, bool lockAfterGet, int retryCount);
        void setLED(byte id, byte mode);
		inline void setLedOn(byte id) { setLED(id, 0); }
		inline void setLedOff(byte id) { setLED(id, 1); }
		byte getLedMode(byte id) { return _led[id]; }
		inline void detectServo() { detectServo(1, _max_id); }
		inline void detectServo(byte max) { detectServo(1, max); }
		void detectServo(byte min, byte max);
		bool exists(byte id);
		byte* retBuffer() { return _retBuf; }
		byte retCount() { return _retCnt; }
		bool isLocked(byte id) { return _isLocked[id]; }
		byte lastAngle(byte id) { return (_isLocked ? _lastAngle[id] : 0xFF); }
		bool isServo(byte id) { return _isServo[id]; }
		int execute(byte cmd[], byte result[]);
		uint16 getAdjAngle(byte id);
		uint16 setAdjAngle(byte id, uint16 adjValue);

    private:
        void initObject(SoftwareSerial *ssData, HardwareSerial *hsDebug);
        inline bool sendCommand() { return sendCommand(true); }
        bool sendCommand(bool expectReturn);
        void showCommand();
        bool checkReturn();
        inline void resetCommandBuffer() { memcpy(_buf, SERVO_CMD, COMMAND_BUFFER_SIZE); }
        inline void resetReturnBuffer() { memset(_retBuf, 0, RETURN_BUFFER_SIZE); _retCnt = 0; }

        SoftwareSerial *_ss;
        HardwareSerial *_dbg;
        bool _enableDebug;

        byte _buf[COMMAND_BUFFER_SIZE];
        byte _retBuf[RETURN_BUFFER_SIZE];  
        byte _retCnt;

		bool _arrayReady;
		byte _max_id;
		bool* _servo;
		byte* _led;
		bool* _isLocked;
		byte* _lastAngle;
		bool* _isServo;
		uint16* _adjAngle;
		byte _servoCnt;
		byte _maxDetectRetry;
		byte _maxCommandWaitMs;
		byte _maxCommandRetry;
		byte _maxTry;

};

#endif
