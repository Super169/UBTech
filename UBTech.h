#ifndef _UBTECH_H_
#define _UBTECH_H_

#include <ESP8266WiFi.h>
#include "HardwareSerial.h"
#include "SoftwareSerial.h"

#define SERVO_BAUD 115200

#define COMMAND_BUFFER_SIZE 10
#define RETURN_BUFFER_SIZE 20  // Actually, 10 is enough, just for saftey

const byte SERVO_CMD[] = {0xFA, 0xAF,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 0xED};
const byte JIMU_VERSION[] = {0xFC, 0xCF,0x00,0xAA,0x41, 0x16, 0x51, 0x01, 0x00, 0xED};

class UBTech {
    public:
        UBTech(byte dataPin);
        UBTech(byte dataPin, HardwareSerial *hsDebug);
        ~UBTech();
        void begin();
        void getVersion(byte id);
        void goAngle(byte id, byte angle, byte time);
        void getPos(byte id);
        void setLED(byte id, byte mode);

    private:
        void initObject(byte dataPin);
        inline bool sendCommand() { sendCommand(true); }
        bool sendCommand(bool expectReturn);
        void showCommand();
        bool checkReturn();
        inline void resetCommandBuffer() { memcpy(_buf, SERVO_CMD, COMMAND_BUFFER_SIZE); }
        inline void resetReurnBuffer() { memset(_retBuf, 0, RETURN_BUFFER_SIZE); _retCnt = 0; }

        SoftwareSerial *_ss;
        HardwareSerial *_hsDebug;
        bool _serialDebug;
        byte _dataPin;
        byte _buf[COMMAND_BUFFER_SIZE];

        byte _retBuf[RETURN_BUFFER_SIZE];  
        byte _retCnt;

};

#endif
