#include <Wire.h>
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Firebase_ESP_Client.h>
#include <WiFiClient.h>

#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

#define ON_Board_LED 2

const char *ssid = "FBI-XAE";
const char *password = "123456789";

#define API_KEY "AIzaSyCIyZs-9c60seTg9T2ueu3h2dIAAXx3Hyc"

#define DATABASE_URL "https://pregathi-69-default-rtdb.asia-southeast1.firebasedatabase.app/"

FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

//--Heart sensor--
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
      digitalWrite(LED_D1, HIGH);
    }

    if (PulseSensorHRVal < Threshold) {
      ThresholdStat = true;
      digitalWrite(LED_D1, LOW);
    }
  }

  unsigned long currentMillisHR = millis();

  if (currentMillisHR - previousMillisHR >= intervalHR) {
    previousMillisHR = currentMillisHR;

    BPMval = cntHB * 6;

    Serial.print("BPM : ");
    Serial.println(BPMval);
    Firebase.RTDB.setInt(&fbdo, "data/user/BPM", BPMval);

    cntHB = 0;
  }
}

//--Gyro sensor--
const int MPU_addr = 0x68;  // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
boolean fall = false;      //stores if a fall has occurred
boolean trigger1 = false;  //stores if first trigger (lower threshold) has occurred
boolean trigger2 = false;  //stores if second trigger (upper threshold) has occurred
boolean trigger3 = false;  //stores if third trigger (orientation change) has occurred
byte trigger1count = 0;    //stores the counts past since trigger 1 was set true
byte trigger2count = 0;    //stores the counts past since trigger 2 was set true
byte trigger3count = 0;    //stores the counts past since trigger 3 was set true
int angleChange = 0;

void send_event(const char *event);
const char *host = "maker.ifttt.com";
const char *privateKey = "xxx-xxx-xxx-xxx-xxx-xxx-xxx";

void setup() {
  Serial.begin(115200);
  delay(500);

  //--Gyro sensor
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.println("Wrote to IMU");

  //--Heart sensor--
  pinMode(ON_Board_LED, OUTPUT);
  digitalWrite(ON_Board_LED, HIGH);

  pinMode(LED_D1, OUTPUT);

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

  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("ok");
    signupOK = true;
  } else {
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }
  config.token_status_callback = tokenStatusCallback;

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  Serial.println();
  Serial.println("Please wait 10 seconds to get the BPM Value");
}

void loop() {
  //--Gyro sensor--
  mpu_read();
  ax = (AcX - 2050) / 16384.00;
  ay = (AcY - 77) / 16384.00;
  az = (AcZ - 1947) / 16384.00;
  gx = (GyX + 270) / 131.07;
  gy = (GyY - 351) / 131.07;
  gz = (GyZ + 136) / 131.07;
  // calculating Amplitute vactor for 3 axis
  float Raw_Amp = pow(pow(ax, 2) + pow(ay, 2) + pow(az, 2), 0.5);
  int Amp = Raw_Amp * 10;  // Mulitiplied by 10 bcz values are between 0 to 1
  Serial.println(Amp);
  if (Amp <= 2 && trigger2 == false) {  //if AM breaks lower threshold (0.4g)
    trigger1 = true;
    Firebase.RTDB.setString(&fbdo, "data/user/Trigger-1", "true");
    Serial.println("TRIGGER 1 ACTIVATED");
  }
  if (trigger1 == true) {
    trigger1count++;
    if (Amp >= 12) {  //if AM breaks upper threshold (3g)
      trigger2 = true;
      Firebase.RTDB.setString(&fbdo, "data/user/Trigger-2", "true");
      Serial.println("TRIGGER 2 ACTIVATED");
      trigger1 = false;
      Firebase.RTDB.setString(&fbdo, "data/user/Trigger-1", "false");
      trigger1count = 0;
    }
  }
  if (trigger2 == true) {
    trigger2count++;
    angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
    Serial.println(angleChange);
    if (angleChange >= 30 && angleChange <= 400) {  //if orientation changes by between 80-100 degrees
      trigger3 = true;
      Firebase.RTDB.setString(&fbdo, "data/user/Trigger-3", "true");
      trigger2 = false;
      Firebase.RTDB.setString(&fbdo, "data/user/Trigger-2", "false");
      trigger2count = 0;
      Serial.println(angleChange);
      Serial.println("TRIGGER 3 ACTIVATED");
    }
  }
  if (trigger3 == true) {
    trigger3count++;
    if (trigger3count >= 10) {
      angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
      //delay(10);
      Serial.println(angleChange);
      if ((angleChange >= 0) && (angleChange <= 10)) {  //if orientation changes remains between 0-10 degrees
        fall = true;
        Firebase.RTDB.setString(&fbdo, "data/user/Fall", "true");
        trigger3 = false;
        Firebase.RTDB.setString(&fbdo, "data/user/Trigger-3", "false");
        trigger3count = 0;
        Serial.println(angleChange);
      } else {  //user regained normal orientation
        trigger3 = false;
        Firebase.RTDB.setString(&fbdo, "data/user/Trigger-3", "false");
        trigger3count = 0;
        Serial.println("TRIGGER 3 DEACTIVATED");
      }
    }
  }
  if (fall == true) {  //in event of a fall detection
    Serial.println("FALL DETECTED");
    send_event("fall_detect");
    fall = false;
    Firebase.RTDB.setString(&fbdo, "data/user/Fall", "false");
  }
  if (trigger2count >= 6) {  //allow 0.5s for orientation change
    trigger2 = false;
    Firebase.RTDB.setString(&fbdo, "data/user/Trigger-2", "false");
    trigger2count = 0;
    Serial.println("TRIGGER 2 DECACTIVATED");
  }
  if (trigger1count >= 6) {  //allow 0.5s for AM to break upper threshold
    trigger1 = false;
    Firebase.RTDB.setString(&fbdo, "data/user/Trigger-1", "false");
    trigger1count = 0;
    Serial.println("TRIGGER 1 DECACTIVATED");
  }

  //--Heart sensor--
  Signal = analogRead(PulseSensorHRWire);

  Serial.println(Signal);
  Firebase.RTDB.setInt(&fbdo, "data/user/Svalue", Signal);

  if (Signal > Threshold) {
    digitalWrite(LED_D1, HIGH);
  } else {
    digitalWrite(LED_D1, LOW);
  }

  GetHeartRate();
}

void mpu_read() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);  // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}
void send_event(const char *event) {
  Serial.print("Connecting to ");
  Serial.println(host);
  // Use WiFiClient class to create TCP connections
  WiFiClient client;
  const int httpPort = 80;
  if (!client.connect(host, httpPort)) {
    Serial.println("Connection failed");
    return;
  }
  // We now create a URI for the request
  String url = "/trigger/";
  url += event;
  url += "/with/key/";
  url += privateKey;
  Serial.print("Requesting URL: ");
  Serial.println(url);
  // This will send the request to the server
  client.print(String("GET ") + url + " HTTP/1.1\r\n" + "Host: " + host + "\r\n" + "Connection: close\r\n\r\n");
  while (client.connected()) {
    if (client.available()) {
      String line = client.readStringUntil('\r');
      Serial.print(line);
    }
  }
  Serial.println();
  Serial.println("closing connection");
  client.stop();
}
