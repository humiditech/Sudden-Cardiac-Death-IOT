#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// MPU6050 Fall detection
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
void mpu_read();

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

void initMPU()
{
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
}

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  // initMPU();
  Wire.beginTransmission(MPU_address);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

void loop()
{
  mpu_read();

  ax = (AcX - 2050) / 16384.00;
  ay = (AcY - 77) / 16384.00;
  az = (AcZ - 1947) / 16384.00;
  gx = (GyX + 270) / 131.07;
  gy = (GyY - 351) / 131.07;
  gz = (GyZ + 136) / 131.07;

  // calculating Amplitute vactor for 3 axis
  float Raw_Amp = pow(pow(ax, 2) + pow(ay, 2) + pow(az, 2), 0.5);
  int Amp = Raw_Amp * 10; // Mulitiplied by 10 bcz values are between 0 to 1
  // Serial.println(Amp);
  if (Amp <= lowerThres && trigger2 == false)
  { //if AM breaks lower threshold (0.4g)
    trigger1 = true;
    Serial.println("TRIGGER 1 ACTIVATED");
  }
  if (trigger1 == true)
  {
    trigger1cnt++;
    if (Amp >= higherThres)
    { //if AM breaks upper threshold (3g)
      trigger2 = true;
      Serial.println("TRIGGER 2 ACTIVATED");
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
      Serial.println(angleChange);
      Serial.println("TRIGGER 3 ACTIVATED");
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
        Serial.println("Jatuhh");
        fall = true;
        trigger3 = false;
        trigger3cnt = 0;
      }
      else
      { //user regained normal orientation
        trigger3 = false;
        trigger3cnt = 0;
        Serial.println("TRIGGER 3 DEACTIVATED");
      }
    }
  }
  if (fall == true)
  { //in event of a fall detection
    Serial.println("FALL DETECTED");
    fall = false;
  }
  if (trigger2cnt >= 6)
  { //allow 0.5s for orientation change
    trigger2 = false;
    trigger2cnt = 0;
    Serial.println("TRIGGER 2 DECACTIVATED");
  }
  if (trigger1cnt >= 6)
  { //allow 0.5s for AM to break upper threshold
    trigger1 = false;
    trigger1cnt = 0;
    Serial.println("TRIGGER 1 DECACTIVATED");
  }
  delay(100);
}

void mpu_read()
{
  // sensors_event_t a, g, temp;
  // mpu.getEvent(&a, &g, &temp);

  // if ((millis() - lastTimeAcc) > accelerometerDelay)
  // {

  //   /* Print out the values */
  //   Serial.print("Acc X: ");
  //   Serial.print(a.acceleration.x);
  //   Serial.print("Acc Y: ");
  //   Serial.print(a.acceleration.y);
  //   Serial.print("Acc Z: ");
  //   Serial.println(a.acceleration.z);

  //   lastTimeAcc = millis();
  // }
  // if ((millis() - lastTime) > gyroDelay)
  // {
  //   Serial.print("Gyro X: ");
  //   Serial.print(g.gyro.x);
  //   Serial.print("Gyro Y: ");
  //   Serial.print(g.gyro.y);
  //   Serial.print("Gyro Z: ");
  //   Serial.println(g.gyro.z);
  //   Serial.println("");

  //   lastTime = millis();
  // }

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