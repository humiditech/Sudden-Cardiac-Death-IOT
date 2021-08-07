#include <Arduino.h>
#include <Defibrilator.h>

#define ECG_OUTPUT 27

Defibrilator defibrilator(ECG_OUTPUT);
FirebaseData defibrilatorFBdata;

#define FIREBASE_HOST "https://sisca-app-67dfd-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "iEZgfjTk3CnbsGTIKGPakZta4W6zprRsob7jAm7I"

TaskHandle_t ECGTask;
TaskHandle_t Defibrilation;

String age;

unsigned long lastTime = 0;
unsigned long lastTimeTemperature = 0;
unsigned long lastTimeAcc = 0;
unsigned long gyroDelay = 10;
unsigned long temperatureDelay = 1000;
unsigned long accelerometerDelay = 50;
unsigned long millisfirst = 0;
bool getfirsttime = false;
bool aritmia = false;

float gyroXerror = 0.07;
float gyroYerror = 0.03;
float gyroZerror = 0.01;

void Def(void * parameter){
  for(;;){
    if(defibrilator.trigger == "OFF"){
      Firebase.getString(defibrilatorFBdata, "/Relay/relayState", defibrilator.trigger);
    }
    if(defibrilator.RRDetection == "ABNORMAL" && defibrilator.trigger == "OFF"){
      if(!getfirsttime){
        millisfirst = millis();
        getfirsttime = false;
        if(DEBUG){
          Serial.println("ABNORMAL, wait for confirmation!");
        }
      }
      else if(millis() - millisfirst >= 900000){
        if(DEBUG){
          Serial.println("Defibrilate Warning!");
        }
        defibrilator.trigger == "ON";
        for(int i=0; 1<4; i++){
          digitalWrite(BUZZER, HIGH);
          delay(300);
          digitalWrite(BUZZER, LOW);
        }
      }
      
    }
    else if(defibrilator.RRDetection == "ABNORMAL" && defibrilator.trigger == "ON"){
      if(DEBUG){
        Serial.println("Defibrilate!!!");
      }
      defibrilator.Defibrilate();
    }
    else if(defibrilator.RRDetection == "NORMAL" && defibrilator.trigger == "ON"){
      if(DEBUG){
        Serial.println("Defibrilate Done!!!");
      }
      defibrilator.NoDefibrilate();
      defibrilator.trigger = "OFF";
      Firebase.setString(defibrilatorFBdata, "/Relay/relayState", "OFF");
      digitalWrite(BUZZER, LOW);
    }
    vTaskDelay(1);
  }
}

void ECGtask(void * parameter){
  for(;;){ 
    defibrilator.GetECGSignal();
    if(defibrilator.IsAritmia()){
      defibrilator.RRDetection = "ABNORMAL";
      aritmia = true;
      if(DEBUG){
        Serial.println("Aritmia GESS!!!");
      }
    }
    else{
      defibrilator.RRDetection = "NORMAL";
      aritmia = false;
      digitalWrite(BUZZER, LOW);
    }
    Firebase.setInt(defibrilatorFBdata,"/Sensor/ecgBPM",defibrilator.GetHeartbeat());
    Firebase.setString(defibrilatorFBdata,"/Sensor/rrDetection",defibrilator.RRDetection);
    vTaskDelay(1);
  }
}

void setup()
{
  Serial.begin(115200);
  defibrilator.begin();
  xTaskCreatePinnedToCore(ECGtask,"ECG",10000,NULL,0,&ECGTask,0);
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.setString(defibrilatorFBdata, "/Relay/relayState", "OFF");
  Firebase.setString(defibrilatorFBdata,"/Sensor/imuSensorStatus","IDLE");
  Firebase.getString(defibrilatorFBdata, "/PatientParams/age", age);
  if(DEBUG){
    Serial.println("AGE :" + age);
  }
  defibrilator.SetMaxHR(age);
  delay(300);
}

void loop()
{
  defibrilator.GetMPUdata();
  defibrilator.Falldetection(defibrilator.ax,defibrilator.ay,defibrilator.az,defibrilator.gx,defibrilator.gy,defibrilator.gz);
  if(fall){
    Firebase.setString(defibrilatorFBdata,"/Sensor/imuSensorStatus","FALL");
    if(DEBUG){
      Serial.println("Jatuh GESS!!!");
    }
  }
  else{
    Firebase.setString(defibrilatorFBdata,"/Sensor/imuSensorStatus","IDLE");
  }
}
