#include <Arduino.h>
#include <Defibrilator.h>
#include <BluetoothSerial.h>

HardwareSerial ECGSerial(1);

Defibrilator defibrilator(&ECGSerial);
FirebaseData defibrilatorFBdata;
BluetoothSerial EcgBluetooth;

#define FIREBASE_HOST "https://sisca-app-67dfd-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "iEZgfjTk3CnbsGTIKGPakZta4W6zprRsob7jAm7I"

TaskHandle_t Data;
TaskHandle_t Defibrilation;

int age;

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
    if(defibrilator.RRDetection == "ARITMIA" && defibrilator.trigger == "OFF"){
      if(!getfirsttime){
        millisfirst = millis();
        getfirsttime = false;
        if(DEBUG){
          Serial.println("ARITMIA, wait for confirmation!");
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
    else if(defibrilator.RRDetection == "ARITMIA" && defibrilator.trigger == "ON"){
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
    Firebase.setInt(defibrilatorFBdata,"/Sensor/ecgSignal",defibrilator.GetAnalogData());
    Firebase.setString(defibrilatorFBdata,"/Sensor/rrDetection",defibrilator.RRDetection);
    if(fall){
      Firebase.setString(defibrilatorFBdata,"/Sensor/imuSensorStatus","FALL");
    }
    else{
      Firebase.setString(defibrilatorFBdata,"/Sensor/imuSensorStatus","IDLE");
    }
    ECGSerial.println(defibrilator.GetAnalogData());
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
  EcgBluetooth.begin("Si-SCA");
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.setString(defibrilatorFBdata, "/Relay/relayState", "OFF");
  Firebase.setString(defibrilatorFBdata,"/Sensor/imuSensorStatus","IDLE");
  Firebase.getInt(defibrilatorFBdata, "/Relay/relayState", age);
  defibrilator.SetMaxHR(age);
  xTaskCreatePinnedToCore(DataSend,"ECG",10000,NULL,1,&Data,1);
  // xTaskCreatePinnedToCore(Def,"Defibliation",10000,NULL,1,&Defibrilation,0);
  
  delay(300);
}

void loop()
{
  defibrilator.GetECGSignal();
  if(millis() - getecgtime > 500){
    getecgtime = millis();
    // Serial.println(data);
    Serial.print(defibrilator.GetRRInterval());
    Serial.println(" ms");
    Serial.print(defibrilator.GetHeartbeat());
    Serial.println(" Bpm");
    Serial.println(defibrilator.GetAnalogData());
    defibrilator.ShiftArray();
    defibrilator.Ledact(defibrilator.GetHeartbeat());
    Serial.println();   
  // }

  if(defibrilator.IsAritmia()){
    defibrilator.RRDetection = "ARITMIA";
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
  }


  // defibrilator.GetMPUdata();
  // defibrilator.Falldetection(defibrilator.ax,defibrilator.ay,defibrilator.az,defibrilator.gx,defibrilator.gy,defibrilator.gz);
  // if(fall){
  //   if(DEBUG){
  //     Serial.println("Jatuh GESS!!!");
  //   }
  // }
}
