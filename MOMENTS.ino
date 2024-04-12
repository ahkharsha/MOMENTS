#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Firebase_ESP_Client.h>
#include <WiFiClient.h>

#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

#define ON_Board_LED 2

const char* ssid = "FBI-XAE";
const char* password = "123456789";

#define API_KEY "AIzaSyCIyZs-9c60seTg9T2ueu3h2dIAAXx3Hyc"

#define DATABASE_URL "https://pregathi-69-default-rtdb.asia-southeast1.firebasedatabase.app/" 

FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;
int count = 0;
bool signupOK = false;

unsigned long previousMillisGetHR = 0;
unsigned long previousMillisHR = 0;

const long intervalGetHR = 10;
const long intervalHR = 10000;

const int PulseSensorHRWire = A0;
const int LED_D1 = D1;
int Threshold = 600;
int Signal;

int cntHB = 0;
boolean ThresholdStat = true;
int BPMval = 0;

void GetHeartRate() {
  unsigned long currentMillisGetHR = millis();

  if (currentMillisGetHR - previousMillisGetHR >= intervalGetHR) {
    previousMillisGetHR = currentMillisGetHR;

    int PulseSensorHRVal = analogRead(PulseSensorHRWire);

    if (PulseSensorHRVal > Threshold && ThresholdStat == true) {
      cntHB++;
      ThresholdStat = false;
      digitalWrite(LED_D1,HIGH);
    }

    if (PulseSensorHRVal < Threshold) {
      ThresholdStat = true;
      digitalWrite(LED_D1,LOW);
    }
  }

  unsigned long currentMillisHR = millis();

  if (currentMillisHR - previousMillisHR >= intervalHR) {
    previousMillisHR = currentMillisHR;

    BPMval = cntHB * 6;
    Serial.print("BPM : ");
    Serial.println(BPMval);
    Firebase.RTDB.setInt(&fbdo, "sensors/heart/BPM", BPMval);
    
    cntHB = 0;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
    
  pinMode(ON_Board_LED,OUTPUT);
  digitalWrite(ON_Board_LED, HIGH);

  pinMode(LED_D1,OUTPUT);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");

  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");

    digitalWrite(ON_Board_LED, LOW);
    delay(250);
    digitalWrite(ON_Board_LED, HIGH);
    delay(250);
  }
  digitalWrite(ON_Board_LED, HIGH);
  Serial.println("");
  Serial.print("Successfully connected to : ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  config.api_key = API_KEY;

  config.database_url = DATABASE_URL;

  if (Firebase.signUp(&config, &auth, "", "")){
    Serial.println("ok");
    signupOK = true;
  }
  else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }
  config.token_status_callback = tokenStatusCallback;
  
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  Serial.println();
  Serial.println("Please wait 10 seconds to get the BPM Value");
}

void loop() {
  Signal = analogRead(PulseSensorHRWire);

  Serial.println(Signal);
  Firebase.RTDB.setInt(&fbdo, "sensors/heart/Svalue", Signal);

  if(Signal > Threshold){
    digitalWrite(LED_D1,HIGH);
  } else {
    digitalWrite(LED_D1,LOW);
  }

  GetHeartRate();
}