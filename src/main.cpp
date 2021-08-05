#include <Arduino.h>
#include <Defibrilator.h>

#define ECG_OUTPUT 27

Defibrilator defibrilator(ECG_OUTPUT);
FirebaseData defibrilatorFBdata;

#define FIREBASE_HOST "https://sisca-app-67dfd-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "iEZgfjTk3CnbsGTIKGPakZta4W6zprRsob7jAm7I"

TaskHandle_t ECGTask;
TaskHandle_t Defibrilation;

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
    if(defibrilator.RRDetection == "ARITMIA" && defibrilator.trigger == "OFF"){
      if(!getfirsttime){
        millisfirst = millis();
        getfirsttime = false;
      }
      else if(millis() - millisfirst >= 900000){
        defibrilator.trigger == "ON";
        digitalWrite(BUZZER, HIGH);
      }
    }
    else if(defibrilator.RRDetection == "ARITMIA" && defibrilator.trigger == "ON"){
      defibrilator.Defibrilate();
    }
    else if(defibrilator.RRDetection == "NORMAL" && defibrilator.trigger == "ON"){
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
      defibrilator.RRDetection = "ARITMIA";
      aritmia = true;
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
  delay(300);
}

void loop()
{
  defibrilator.GetMPUdata();
  defibrilator.Falldetection(defibrilator.ax,defibrilator.ay,defibrilator.az,defibrilator.gx,defibrilator.gy,defibrilator.gz);
  Firebase.setString(defibrilatorFBdata,"/Sensor/imuSensorStatus","IDLE");
}
