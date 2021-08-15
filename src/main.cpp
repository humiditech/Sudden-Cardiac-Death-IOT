#include <Arduino.h>
#include <Defibrilator.h>

#define ECG_OUTPUT 39

Defibrilator defibrilator(ECG_OUTPUT);
FirebaseData defibrilatorFBdata;

#define FIREBASE_HOST "https://sisca-app-67dfd-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "iEZgfjTk3CnbsGTIKGPakZta4W6zprRsob7jAm7I"

TaskHandle_t Data;
TaskHandle_t Defibrilation;

String age;

const char* ssid = "sc19";
const char* password =  "4444333221";

unsigned long lastTime = 0;
unsigned long lastTimeTemperature = 0;
unsigned long lastTimeAcc = 0;
unsigned long gyroDelay = 10;
unsigned long temperatureDelay = 1000;
unsigned long accelerometerDelay = 50;
unsigned long millisfirst = 0;
unsigned long printmillis = 0;
unsigned long getecgtime = 0;
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

void DataSend(void * parameter){
  for(;;){ 
    Firebase.setInt(defibrilatorFBdata,"/Sensor/ecgBPM",defibrilator.GetHeartbeat());
    Firebase.setString(defibrilatorFBdata,"/Sensor/rrDetection",defibrilator.RRDetection);
    Firebase.setString(defibrilatorFBdata,"/Sensor/imuSensorStatus","FALL");
    // vTaskDelay(100);
  }
}

void setup()
{
  Serial.begin(115200);
  defibrilator.begin();
    WiFi.begin(ssid, password);
    if(DEBUG){
        Serial.print("Connecting to Wi-Fi");
    }
    while (WiFi.status() != WL_CONNECTED){
         if(DEBUG){
            Serial.print(".");
        }
        delay(200);
    }
    Serial.println("Connected");
    delay(1000);
    
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.setString(defibrilatorFBdata, "/Relay/relayState", "OFF");
  Firebase.setString(defibrilatorFBdata,"/Sensor/imuSensorStatus","IDLE");
  Firebase.getString(defibrilatorFBdata, "/PatientParams/age", age);

  defibrilator.SetMaxHR(age);
  
  xTaskCreatePinnedToCore(DataSend,"ECG",10000,NULL,1,&Data,1);
  // xTaskCreatePinnedToCore(Def,"Defibliation",10000,NULL,1,&Defibrilation,0);
  
  delay(300);
}

void loop()
{
  defibrilator.GetECGSignal();
  
  if(millis() - printmillis >= 1000){
    printmillis = millis();
    Serial.print(defibrilator.GetHeartbeat());
    Serial.println(" BPM");
  }

  // if(defibrilator.IsAritmia()){
  // defibrilator.RRDetection = "ABNORMAL";
  // aritmia = true;
  // if(DEBUG){
  //     Serial.println("Aritmia GESS!!!");
  // }
  // }
  // else{
  //   defibrilator.RRDetection = "NORMAL";
  //   aritmia = false;
  //   digitalWrite(BUZZER, LOW);
  // }

  // defibrilator.GetMPUdata();
  // defibrilator.Falldetection(defibrilator.ax,defibrilator.ay,defibrilator.az,defibrilator.gx,defibrilator.gy,defibrilator.gz);
  // if(fall){
  //   if(DEBUG){
  //     Serial.println("Jatuh GESS!!!");
  //   }
  // }
}
