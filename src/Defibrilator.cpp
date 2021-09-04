#include <Defibrilator.h>

int beatIndex=0;
int RRindex = 0;
unsigned long beat_old = 0;
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
uint8_t dataindex = 0;

Defibrilator::Defibrilator(HardwareSerial *HW){
  this->EcgSerial = HW;
}

void Defibrilator::begin(){
    this->EcgSerial->begin(9600, SERIAL_8N1,16,17);
    // pinMode(this->_datapin, INPUT);
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(LED_HIJAU, OUTPUT);
    pinMode(LED_KUNING, OUTPUT);
    pinMode(LED_MERAH, OUTPUT);
    pinMode(BUZZER, OUTPUT);

    Wire.begin();
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0);    // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
}

uint16_t Defibrilator::mean(uint16_t a, uint16_t b){
    return (a+b)/2;
}

void Defibrilator::GetECGSignal(){
  String data="";
  if(this->EcgSerial->available()){
    data = this->EcgSerial->readStringUntil('\n');

    int index1 = data.indexOf(',');
    int index2 = data.indexOf(',',index1+1);

    this->Analogdata = data.substring(0,index1).toInt();
  if(
    ((this->Analogarray[0] < ECGOnDetect_Upper)  &&  Analogarray[0] > ECGOnDetect_Lower) 
    && 
    ((this->Analogarray[1] < ECGOnDetect_Upper)  &&  Analogarray[1] > ECGOnDetect_Lower) 
    &&
    ((this->Analogarray[2] < ECGOnDetect_Upper)  &&  Analogarray[2] > ECGOnDetect_Lower) 
    && 
    ((this->Analogarray[3] < ECGOnDetect_Upper)  &&  Analogarray[3] > ECGOnDetect_Lower) 
    &&
    ((this->Analogarray[4] < ECGOnDetect_Upper)  &&  Analogarray[4] > ECGOnDetect_Lower) 
    && 
    ((this->Analogarray[5] < ECGOnDetect_Upper)  &&  Analogarray[5] > ECGOnDetect_Lower) 
    &&
    ((this->Analogarray[6] < ECGOnDetect_Upper)  &&  Analogarray[6] > ECGOnDetect_Lower) 
    && 
    ((this->Analogarray[7] < ECGOnDetect_Upper)  &&  Analogarray[7] > ECGOnDetect_Lower) 
    &&
    ((this->Analogarray[8] < ECGOnDetect_Upper)  &&  Analogarray[8] > ECGOnDetect_Lower) 
    && 
    ((this->Analogarray[9] < ECGOnDetect_Upper)  &&  Analogarray[9] > ECGOnDetect_Lower)
  ){
    this->HBmean = 0;
    this->RRintervalnow = 0;    
  }
  else{
    this->HBmean = data.substring(index1 + 1, index2).toInt();
    this->RRintervalnow = data.substring(index2+1).toInt();
  }

  
    // for(int i=1;i<100;i++){
    //   Analogarray[i] = Analogarray[i-1];
    // }
    // Analogarray[0] = Analogdata;

    // for(int i=0;i<100;i++){
    //   if(Analogarray[i] > ECGOnDetect_Upper && Analogarray[i] < ECGOnDetect_Lower){
    //     count_detect++;
    //   }
    //   else{
    //     break;
    //   }
    // }

    // Serial.println(count_detect);

    // if(count_detect == 100){
    //   count_detect = 0;
    //   _HBdetected = false;
    //   this->HBmean = 0;
    //   this->RRintervalnow = 0;
    // }
    // else{
    //   _HBdetected = true;
    //   this->HBmean = data.substring(index1 + 1, index2).toInt();
    //   this->RRintervalnow = data.substring(index2+1).toInt();

    //   for(int i=1;i<10;i++){
    //   beats[i] = beats[i-1];
    //   }
    //   beats[0] = HBmean;

    //   this->RRArray[2] = this->RRArray[1];
    //   this->RRArray[1] = this->RRArray[0];
    //   this->RRArray[0] = this->RRintervalnow;
    //   count_detect = 0;
    // }
  }
}

bool Defibrilator::IsAritmia(){
  bool isaritmia = false;
  if(
    ((beats[0] < ARITMIA_LOWER_THRESHOLD && beats[0] > 0)  || beats[0] > ARITMIA_UPPER_THRESHOLD) 
    && 
    ((beats[1] < ARITMIA_LOWER_THRESHOLD && beats[1] > 0)  || beats[1] > ARITMIA_UPPER_THRESHOLD) 
    &&
    ((beats[2] < ARITMIA_LOWER_THRESHOLD && beats[2] > 0)  || beats[2] > ARITMIA_UPPER_THRESHOLD) 
    && 
    ((beats[3] < ARITMIA_LOWER_THRESHOLD && beats[3] > 0)  || beats[3] > ARITMIA_UPPER_THRESHOLD) 
    &&
    ((beats[4] < ARITMIA_LOWER_THRESHOLD && beats[4] > 0)  || beats[4] > ARITMIA_UPPER_THRESHOLD) 
    && 
    ((beats[5] < ARITMIA_LOWER_THRESHOLD && beats[5] > 0)  || beats[5] > ARITMIA_UPPER_THRESHOLD) 
    /*&&
    ((beats[6] < ARITMIA_LOWER_THRESHOLD && beats[6] > 0)  || beats[6] > ARITMIA_UPPER_THRESHOLD) 
    && 
    ((beats[7] < ARITMIA_LOWER_THRESHOLD && beats[7] > 0)  || beats[7] > ARITMIA_UPPER_THRESHOLD) 
    &&
    ((beats[8] < ARITMIA_LOWER_THRESHOLD && beats[8] > 0)  || beats[8] > ARITMIA_UPPER_THRESHOLD) 
    && 
    ((beats[9] < ARITMIA_LOWER_THRESHOLD && beats[9] > 0)  || beats[9] > ARITMIA_UPPER_THRESHOLD)*/
  ){
    _HBdetected = true;
    // return true;
  }
  else{
    _HBdetected = false;
  }

  if(_HBdetected){
    Serial.println("masuk pengecekan");
    _HBdetected = false;
    if((RRArray[1] < 600) && (RRArray[1] < RRArray[2])){
      Serial.println(1);
      isaritmia = true;
    }
    else{
      Serial.println(2);
      isaritmia = false;
    }
    // if((RRArray[1] < 600) && ((1.8*RRArray[1]) < RRArray[0])){
    // Serial.println(1);
    // isaritmia = true; 
    // }
    // else if(
    //     (((1.15*RRArray[1]) < RRArray[0]) && ((1.15*RRArray[1]) < RRArray[2]))
    //                                         ||
    //     ((abs(RRArray[0] - RRArray[1]) < 300) && ((RRArray[0] < 800) && (RRArray[1] < 800)) 
    //     && (RRArray[2] > (1.2*this->mean(RRArray[0], RRArray[1]))))
    //                                         ||
    //     ((abs(RRArray[1] - RRArray[2]) < 300) && ((RRArray[1] < 800) && (RRArray[2] < 800)) 
    //     && (RRArray[0] > (1.2*this->mean(RRArray[1], RRArray[2]))))
    // ){
    //   Serial.println(2);
    //   isaritmia = true;
    // }
    // else if((RRArray[1] > 2200 && RRArray[1] < 3000)
    //                             && 
    // ((abs(RRArray[0]-RRArray[1]) < 200) || (abs(RRArray[1]-RRArray[2]) < 200))
    // ){
    //   Serial.println(3);
    //   isaritmia = true;
    // }
    // else{
    //   Serial.println(4);
    //   isaritmia = false;
    // }
    // Serial.println(isaritmia);
    return isaritmia;
  }
  else{
    // Serial.println(5);
    return false;
  }
}

void Defibrilator::Defibrilate(){
    if(!_Defon)_Defon = true;

    if(_Defon)
    {    
        if((micros() - this->_defibrilateTime) > 10000){
        this->_defibrilateTime = micros();
        this->_relaystate = !this->_relaystate;
        }
        digitalWrite(RELAY_PIN, this->_relaystate);

        if(_Defon && this->_relaystate == LOW){
            _Defon = false;
            RRDetection = "NORMAL";
        }
    }
    
}

void Defibrilator::NoDefibrilate(){
  digitalWrite(RELAY_PIN, LOW);
}

void Defibrilator::GetMPUdata()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); // request total of 14 registers
  AcX = Wire.read() << 8 | Wire.read();    // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read();    // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read();    // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read();    // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read();    // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read();    // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read();    // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  ax = (AcX - 2050) / 16384.00;
  ay = (AcY - 77) / 16384.00;
  az = (AcZ - 1947) / 16384.00;
  gx = (GyX + 270) / 131.07;
  gy = (GyY - 351) / 131.07;
  gz = (GyZ + 136) / 131.07;
}

void Defibrilator::Falldetection(float ax, float ay, float az, float gx, float gy, float gz){
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

void Defibrilator::IsBodyFall(){
    this->GetMPUdata();
    this->Falldetection(ax, ay, az, gx, gy, gz);
}

void Defibrilator::SetMaxHR(int age){
  this->_patienAge = age;
  this->_MaxHR = 220 - this->_patienAge;
  if(DEBUG){
    Serial.println("MAX HR :" + String(this->_MaxHR));
  }
}

void Defibrilator::Ledact(uint16_t HB){
  uint16_t percentage = ((float)(HB/this->_MaxHR))*100;

  if(percentage <= 63){
    digitalWrite(LED_HIJAU, HIGH);
    digitalWrite(LED_MERAH, LOW);
    digitalWrite(LED_KUNING, LOW);
    digitalWrite(BUZZER, HIGH); 
  }
  else if(percentage >= 64 && percentage <= 76){
    digitalWrite(LED_HIJAU, LOW);
    digitalWrite(LED_MERAH, LOW);
    digitalWrite(LED_KUNING, HIGH);
    digitalWrite(BUZZER, HIGH); 
  }
  else if(percentage >= 77 && percentage <= 93){
    digitalWrite(LED_HIJAU, LOW);
    digitalWrite(LED_MERAH, HIGH);
    digitalWrite(LED_KUNING, LOW);
    digitalWrite(BUZZER, HIGH); 
  }
  else{
    digitalWrite(LED_HIJAU, LOW);
    digitalWrite(LED_MERAH, HIGH);
    digitalWrite(LED_KUNING, LOW);
    digitalWrite(BUZZER, HIGH);    
  }
}

void Defibrilator::ShiftArray(){
  this->RRArray[2] = this->RRArray[1];
  this->RRArray[1] = this->RRArray[0];
  this->RRArray[0] = this->RRintervalnow; 

  // beats[9] = beats[8];
  // beats[8] = beats[7];
  // beats[7] = beats[6];
  // beats[6] = beats[5];
  beats[5] = beats[4];
  beats[4] = beats[3];
  beats[3] = beats[2];
  beats[2] = beats[1];
  beats[1] = beats[0];
  beats[0] = HBmean;

  Analogarray[9] = Analogarray[8];
  Analogarray[8] = Analogarray[7];
  Analogarray[7] = Analogarray[6];
  Analogarray[6] = Analogarray[5];
  Analogarray[5] = Analogarray[4];
  Analogarray[4] = Analogarray[3];
  Analogarray[3] = Analogarray[2];
  Analogarray[2] = Analogarray[1];
  Analogarray[1] = Analogarray[0];
  Analogarray[0] = this->Analogdata;

  String a = String(RRArray[0]) + "," + String(RRArray[1]) + "," + String(RRArray[2]);
  String d = String(beats[0]) + "," + String(beats[1]) + "," +String(beats[2]) + "," + String(beats[3]) + "," 
  + String(beats[4]) + "," + String(beats[5]) /*+ "," + String(beats[6]) + "," + String(beats[7]) + "," + String(beats[8]) 
  + "," + String(beats[9])*/;
  String c = String(Analogarray[0]) + "," + String(Analogarray[1]) + "," +String(Analogarray[2]) + "," + String(Analogarray[3]) + "," 
  + String(Analogarray[4]) + "," + String(Analogarray[5]) + "," + String(Analogarray[6]) + "," + String(Analogarray[7]) + "," + String(Analogarray[8]) 
  + "," + String(Analogarray[9]);
  Serial.println(a);
  Serial.println(d);
  Serial.println(c); 
}

uint16_t Defibrilator::GetHeartbeat(){
  return this->HBmean;
}

uint16_t Defibrilator::GetRRInterval(){
  return this->RRintervalnow;
}

uint16_t Defibrilator::GetAnalogData(){
  return this->Analogdata;
}

bool Defibrilator::GetStatus(){
  return this->ECGPlaced;
}

