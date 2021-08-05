#ifndef DEFIBRILATOR_H
#define DEFIBRILATOR_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <FirebaseESP32.h>

#define RELAY_PIN 13
#define LED_HIJAU 33
#define LED_MERAH 12
#define LED_KUNING 32
#define BUZZER 25

#define threshold 620
#define ARITMIA_UPPER_THRESHOLD 100
#define ARITMIA_LOWER_THRESHOLD 60

#define SSID "sc19"
#define PASS "4444333221"

#define DEBUG true

#define MPU_ADDR 0x68

extern int beatIndex;
extern int RRindex;
extern int beat_old;
extern boolean fall;     // true when fall has occurred
extern boolean trigger1; // true when accelerometer value exceed lower threshold
extern boolean trigger2; // true when accelerometer value exceed upper threshold
extern boolean trigger3; // true when orientation change has occurred
extern byte trigger;
extern byte trigger;
extern byte trigger;
extern int angleCh;
extern int lowerT;
extern int higherTh;
extern int angleLowerTh;
extern int angleHigherThr;

class Defibrilator{
    private:
    Adafruit_MPU6050 MPU;
    uint8_t _datapin;
    uint16_t beats[4] = {0,0,0,0};
    float RRInterval[5] = {0,0,0,0,0};
    float RRArray[3] = {0,0,0};
    float RRintervalnow = 0;
    uint16_t HBmean = 0;
    bool belowThreshold = true;
    uint8_t cnt=0;
    bool firsttimeon = true;
    int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
    unsigned long _defibrilateTime = 0;
    bool _relaystate = LOW;
    bool _Defon = false;

    private:
    float mean(float a, float b);

    public:
    String RRDetection = "NORMAL";
    float ax,ay,az,gx,gy,gz;
    String trigger;
   
    public:
    Defibrilator(uint8_t pin);
    void begin();
    void GetECGSignal();
    uint16_t GetHeartbeat();
    float GetRRInterval();
    bool IsAritmia();
    void SendData();
    void Falldetection(float ax, float ay, float az, float gx, float gy, float gz);
    void IsBodyFall();
    void GetMPUdata();
    void Defibrilate();
    void NoDefibrilate();
};

#endif