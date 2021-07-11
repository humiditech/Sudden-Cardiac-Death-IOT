#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SPI.h>
#include "protocentral_Max30003.h"

// MPU6050 Variables
const int MPU_address = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
boolean fall = false;     // true when fall has occurred
boolean trigger1 = false; // true when accelerometer value exceed lower threshold
boolean trigger2 = false; // true when accelerometer value exceed upper threshold
boolean trigger3 = false; // true when orientation change has occurred
byte trigger1cnt = 0;
byte trigger2cnt = 0;
byte trigger3cnt = 0;
int angleChange = 0;
int lowerThres = 2;
int higherThres = 12;
int angleLowerThres = 30;
int angleHigherThres = 400;

// Timer variables
unsigned long lastTime = 0;
unsigned long lastTimeTemperature = 0;
unsigned long lastTimeAcc = 0;
unsigned long gyroDelay = 10;
unsigned long temperatureDelay = 1000;
unsigned long accelerometerDelay = 50;

// Create a sensor object
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

float gyroX, gyroY, gyroZ;
float accX, accY, accZ;
float temperature;

//Gyroscope sensor deviation
float gyroXerror = 0.07;
float gyroYerror = 0.03;
float gyroZerror = 0.01;

#define DEBUG true
#define RELAY_PIN 13
#define LED_HIJAU 33
#define LED_MERAH 12
#define LED_KUNING 32

// ------------------ MAX30003 ECG Module ------------------
#define INT_PIN 26
MAX30003 max30003;
bool rtorIntrFlag = false;
uint8_t statusReg[3];

// ------------------- Function Declaration --------------------
void initMPU6050();
void mpu_read();
void fallDetection(float ax, float ay, float az, float gx, float gy, float gz);
void initMAX30003();
void rtorInterruptHandler();
void enableInterruptPin();

void setup()
{
  Serial.begin(115200);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(MAX30003_CS_PIN, OUTPUT);
  digitalWrite(MAX30003_CS_PIN, HIGH);
  initMAX30003();
  initMPU6050();
}

void loop()
{
  // mpu_read();
  // ax = (AcX - 2050) / 16384.00;
  // ay = (AcY - 77) / 16384.00;
  // az = (AcZ - 1947) / 16384.00;
  // gx = (GyX + 270) / 131.07;
  // gy = (GyY - 351) / 131.07;
  // gz = (GyZ + 136) / 131.07;
  // fallDetection(ax, ay, az, gx, gy, gz);
  // delay(100);

  // max30003.getEcgSamples(); //It reads the ecg sample and stores it to max30003.ecgdata .

  // Serial.println(max30003.ecgdata);
  // delay(8);
  if (rtorIntrFlag)
  {
    rtorIntrFlag = false;
    max30003.max30003RegRead(STATUS, statusReg);

    if (statusReg[1] & RTOR_INTR_MASK)
    {
      max30003.getHRandRR(); // get heart rate and rr interval
      if (DEBUG)
      {
        Serial.print(F("Heart Rate =  "));
        Serial.println(max30003.heartRate);

        Serial.print(F("RR Interval = "));
        Serial.println(max30003.RRinterval);
      }
    }
  }
}

void initMPU6050()
{
  Wire.begin();
  Wire.beginTransmission(MPU_address);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

void mpu_read()
{
  Wire.beginTransmission(MPU_address);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_address, 14, true); // request total of 14 registers
  AcX = Wire.read() << 8 | Wire.read();    // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read();    // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read();    // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read();    // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read();    // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read();    // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read();    // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void fallDetection(float ax, float ay, float az, float gx, float gy, float gz)
{
  // calculating Amplitute vactor for 3 axis
  float Raw_Amp = pow(pow(ax, 2) + pow(ay, 2) + pow(az, 2), 0.5);
  int Amp = Raw_Amp * 10; // Mulitiplied by 10 bcz values are between 0 to 1
  // Serial.println(Amp);
  if (Amp <= lowerThres && trigger2 == false)
  { //if AM breaks lower threshold (0.4g)
    trigger1 = true;
    if (DEBUG)
      Serial.println(F("TRIGGER 1 ACTIVATED"));
  }
  if (trigger1 == true)
  {
    trigger1cnt++;
    if (Amp >= higherThres)
    { //if AM breaks upper threshold (3g)
      trigger2 = true;
      if (DEBUG)
        Serial.println(F("TRIGGER 2 ACTIVATED"));
      trigger1 = false;
      trigger1cnt = 0;
    }
  }
  if (trigger2 == true)
  {
    trigger2cnt++;
    angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
    Serial.println(angleChange);
    if (angleChange >= angleLowerThres && angleChange <= angleHigherThres)
    { //if orientation changes by between 80-100 degrees
      trigger3 = true;
      trigger2 = false;
      trigger2cnt = 0;
      if (DEBUG)
      {
        Serial.println(angleChange);
        Serial.println(F("TRIGGER 3 ACTIVATED"));
      }
    }
  }
  if (trigger3 == true)
  {
    trigger3cnt++;
    if (trigger3cnt >= 10)
    {
      angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
      delay(10);
      Serial.println(angleChange);
      if ((angleChange >= 0) && (angleChange <= 10))
      { //if orientation changes remains between 0-10 degrees
        if (DEBUG)
          Serial.println(F("Jatuhh"));
        fall = true;
        trigger3 = false;
        trigger3cnt = 0;
      }
      else
      { //user regained normal orientation
        trigger3 = false;
        trigger3cnt = 0;
        if (DEBUG)
          Serial.println(F("TRIGGER 3 DEACTIVATED"));
      }
    }
  }
  if (fall == true)
  { //in event of a fall detection
    if (DEBUG)
      Serial.println(F("FALL DETECTED"));
    // Send notification, fall detected
    fall = false;
  }
  if (trigger2cnt >= 6)
  { //allow 0.5s for orientation change
    trigger2 = false;
    trigger2cnt = 0;
    if (DEBUG)
      Serial.println(F("TRIGGER 2 DECACTIVATED"));
  }
  if (trigger1cnt >= 6)
  { //allow 0.5s for AM to break upper threshold
    trigger1 = false;
    trigger1cnt = 0;
    if (DEBUG)
      Serial.println(F("TRIGGER 1 DECACTIVATED"));
  }
}

void initMAX30003()
{
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);

  bool ret = max30003.max30003ReadInfo();
  if (ret)
  {
    if (DEBUG)
      Serial.println(F("Max30003 ID Success"));
  }
  else
  {
    while (!ret)
    {
      ret = max30003.max30003ReadInfo();
      if (DEBUG)
        Serial.println(F("Failed to read ID"));
      delay(5000);
    }
  }

  if (DEBUG)
    Serial.println("Initializing the chip ....");
  max30003.max30003BeginRtorMode();
  enableInterruptPin();
  max30003.max30003RegRead(STATUS, statusReg);
}

void rtorInterruptHandler()
{
  rtorIntrFlag = true;
}

void enableInterruptPin()
{
  pinMode(INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), rtorInterruptHandler, CHANGE);
}
