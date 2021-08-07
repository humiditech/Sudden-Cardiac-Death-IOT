#include <Defibrilator.h>

int beatIndex = 0;
int RRindex = 0;
int beat_old = 0;
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

Defibrilator::Defibrilator(uint8_t pin){
    this->_datapin = pin;
}

void Defibrilator::begin(){
    pinMode(this->_datapin, INPUT);
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

    WiFi.begin(SSID, PASS);
    if(DEBUG){
        Serial.print("Connecting to Wi-Fi");
    }
    while (WiFi.status() != WL_CONNECTED){
         if(DEBUG){
            Serial.print(".");
        }
        delay(200);
    }
}

float Defibrilator::mean(float a, float b){
    return (a+b)/2;
}

void Defibrilator::GetECGSignal(){
    
  if ((analogRead(this->_datapin) > threshold) && (this->belowThreshold == true)){
    int beat_new = millis();    // get the current millisecond
    int diff = beat_new - beat_old;    // find the time between the last two beats
    int currentBPM = 60000/ diff;    // convert to beats per minute

    this->beats[0] = this->beats[1];
    this->beats[1] = this->beats[2];
    this->beats[2] = this->beats[3];
    this->beats[3] = currentBPM;  // store to array to convert the average

    this->RRInterval[0] = this->RRInterval[1];
    this->RRInterval[1] = this->RRInterval[2];
    this->RRInterval[2] = diff;

    int total = 0;

    for (int i = 0; i < 4; i++){
        total += beats[i];
    }

    if(firsttimeon)
    {
        this->cnt++;
        if(this->cnt >= 4){
        this->firsttimeon = false;
        this->cnt = 0;
        }
    }
    else{
        this->HBmean = (int)(total / 4);
        this->RRintervalnow = diff;
        if(DEBUG){
            String data = "HR : " + String(this->HBmean) + " BPM" + "\n" + "RR : " + String(this->RRintervalnow,2) + " ms" +"\n\n"; 
            Serial.println(data);
        }
        beat_old = beat_new;
        belowThreshold = false;
    }
  }

  else if ((analogRead(this->_datapin)) < threshold){
    belowThreshold = true;
  }
}

bool Defibrilator::IsAritmia(){
    if((RRInterval[1] < 0.6) && ((1.8*RRInterval[1]) < RRInterval[0])){
      return true;  
    }
    else if(
        (((1.15*RRInterval[1]) < RRInterval[0]) && ((1.15*RRInterval[1]) < RRInterval[2]))
                                            ||
        ((fabsf(RRInterval[0] - RRInterval[1]) < 0.3) && ((RRInterval[0] < 0.8) && (RRInterval[1] < 0.8)) 
        && (RRInterval[2] > (1.2*this->mean(RRInterval[0], RRInterval[1]))))
                                            ||
        ((fabsf(RRInterval[1] - RRInterval[2]) < 0.3) && ((RRInterval[1] < 0.8) && (RRInterval[2] < 0.8)) 
        && (RRInterval[0] > (1.2*this->mean(RRInterval[1], RRInterval[2]))))
    ){
      return true;
    }
    else if((RRInterval[1] > 2.2 && RRInterval[1] < 3.0)
                                 && 
    ((fabsf(RRInterval[0]-RRInterval[1]) < 0.2) || (fabsf(RRInterval[1]-RRInterval[2]) < 0.2))

    ){
        return true;
    }
    else{
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

void Defibrilator::SetMaxHR(String age){
  this->_patienAge = age;
  this->_MaxHR = 220 - age.toInt();
  if(DEBUG){
    Serial.println("MAX HR :" + String(this->_MaxHR));
  }
}

void Defibrilator::Ledact(uint16_t HB){
  uint8_t percentage = (HB/this->_MaxHR)*100;

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

uint16_t Defibrilator::GetHeartbeat(){
    return this->HBmean;
}

float Defibrilator::GetRRInterval(){
    return this->RRintervalnow;
}

