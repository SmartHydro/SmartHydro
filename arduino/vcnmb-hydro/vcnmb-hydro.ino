#include <WiFiEsp.h>
#include <WiFiEspClient.h>
#include <WiFiEspServer.h>
#include <DHTesp.h>
#include <ArduinoJson.h>
#include <DFRobot_EC10.h>
#include <DFRobot_PH.h>

#define STATION_SSID "beef hotpot"
#define STATION_PASS "tastyNetwork"
#define AP_SSID "VCNMB-Hydro"
#define AP_PASS "hydro123"
#define PUMP_1_PIN 7
#define PUMP_2_PIN 8
#define PUMP_3_PIN 9
#define FAN_1_PIN 10
#define FAN_2_PIN 11
#define LIGHT_1_PIN 12
#define TEMP_HUMID_SENS_PIN 50
#define FLOW_SENS_PIN 52
#define AMB_LIGHT_SENS_PIN A0
#define PH_SENS_PIN A1
#define EC_SENS_PIN A2
#define PH_CAL 0.00
#define FLOW_CAL 0.00
#define EC_CAL 0.00
#define BUZZER_PIN 48

WiFiEspServer WebServer(80);
DHTesp TempHumid;
DFRobot_EC10 EC10;
DFRobot_PH PH;
volatile int Pulse_Count;
unsigned long Current_Time, Loop_Time;

void setup() {
  // USB Serial Connection
  Serial.begin(115200);

  // DHT22 Temp/Humidity Sensor
  TempHumid.setup(TEMP_HUMID_SENS_PIN, 'AUTO_DETECT');

  // YF-B6 Flow Sensor
  pinMode(FLOW_SENS_PIN, INPUT);
  attachInterrupt(0, Detect_Rising_Edge_Flow, FALLING);
  Current_Time = millis();
  Loop_Time = Current_Time;

  // Ambient Light Sensor
  pinMode(AMB_LIGHT_SENS_PIN, INPUT);

  // SEN0161 pH Sensor
  pinMode(PH_SENS_PIN, INPUT);
  PH.begin();

  // EC10 Current Sensor
  pinMode(EC_SENS_PIN, INPUT);

  // ESP8266 WiFi Connection
  Serial1.begin(115200);
  WiFi.init(&Serial1);
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.print("no wifi shield, panicking!!!");
    while (true)
      ;
  }
  IPAddress SysIP(192, 168, 1, 10);

  //WiFi.begin(STATION_SSID, STATION_PASS);
  //Serial.print("Connecting to ");
  //Serial.print(STATION_SSID);
  //int wait = 0;
  //while (WiFi.status() != WL_CONNECTED)
  //{
  //  delay(100);
  //  Serial.print(".");
  //  wait += 1;
  // if (wait >= 10) {
  //    Serial.print("couldn't find network. panicking!!!");
  //    while (true);
  //  }
  //}

  WiFi.configAP(SysIP);
  WiFi.beginAP(AP_SSID, 13, AP_PASS, ENC_TYPE_WPA2_PSK, false);
  WebServer.begin();

  // Relay PWM Pins, 7-12
  for (int i = 6; i <= 12; i++) {
    pinMode(i, OUTPUT);
  }
}

void loop() {
  WiFiEspClient WebClient = WebServer.available();
  String RequestHeader = "";
  if (WebClient) {
    Serial.println("client connection started");
    while (WebClient.connected()) {
      if (WebClient.available()) {
        char c = WebClient.read();
        Serial.print(c);
        RequestHeader += c;
        if (c == '\n') {
          if (RequestHeader.indexOf("GET /hardware.json") >= 0) {
            serializeJsonPretty(HardwareToJson(), WebClient);
            break;
          }
          if (RequestHeader.indexOf("GET /sensor.json") >= 0) {
            serializeJsonPretty(SensorToJson(), WebClient);
            break;
          }
          if (RequestHeader.indexOf("GET /pump1") >= 0) {
            TogglePin(PUMP_1_PIN);
            serializeJsonPretty(HardwareToJson(), WebClient);
            break;
          }
          if (RequestHeader.indexOf("GET /pump2") >= 0) {
            TogglePin(PUMP_2_PIN);
            serializeJsonPretty(HardwareToJson(), WebClient);
            break;
          }
          if (RequestHeader.indexOf("GET /pump3") >= 0) {
            TogglePin(PUMP_3_PIN);
            serializeJsonPretty(HardwareToJson(), WebClient);
            break;
          }
          if (RequestHeader.indexOf("GET /fan1") >= 0) {
            TogglePin(FAN_1_PIN);
            serializeJsonPretty(HardwareToJson(), WebClient);
            break;
          }
          if (RequestHeader.indexOf("GET /fan2") >= 0) {
            TogglePin(FAN_2_PIN);
            serializeJsonPretty(HardwareToJson(), WebClient);
            break;
          }
          if (RequestHeader.indexOf("GET /light1") >= 0) {
            TogglePin(LIGHT_1_PIN);
            TogglePin(BUZZER_PIN);
            serializeJsonPretty(HardwareToJson(), WebClient);
            break;
          }
        }
      }
    }
    WebClient.stop();
    RequestHeader = "";
    Serial.println("client connection terminated");
  }
}

void Detect_Rising_Edge_Flow() {
  Pulse_Count++;
}

void TogglePin(int Pin) {
  digitalWrite(Pin, !digitalRead(Pin));
}

DynamicJsonDocument HardwareToJson() {
  DynamicJsonDocument temp(128);
  temp["Pump1"] = digitalRead(PUMP_1_PIN);
  temp["Pump2"] = digitalRead(PUMP_2_PIN);
  temp["Pump3"] = digitalRead(PUMP_3_PIN);
  temp["Fan1"] = digitalRead(FAN_1_PIN);
  temp["Fan2"] = digitalRead(FAN_2_PIN);
  temp["Light1"] = digitalRead(LIGHT_1_PIN);
  temp["WiFi_Module"] = WiFi.status();
  temp["IP_Address"] = WiFi.localIP();
  return temp;
}

DynamicJsonDocument SensorToJson() {
  DynamicJsonDocument temp(128);
  TempAndHumidity measurement = TempHumid.getTempAndHumidity();
  temp["Temperature"] = measurement.temperature, 1;
  temp["Humidity"] = measurement.humidity, 0;
  temp["LightLevel"] = analogRead(AMB_LIGHT_SENS_PIN);
  Current_Time = millis();
  if (Current_Time >= (Loop_Time + 1000)) {
    temp["FlowRate"] = (Pulse_Count * 60 / 7.5) + FLOW_CAL, 2;
  }
  float PH_VOLT = analogRead(PH_SENS_PIN) / 1024.0 * 5000;
  float PH_LEVEL = PH.readPH(PH_VOLT, measurement.temperature);
  temp["pH"] = PH_LEVEL;
  float EC_VOLT = analogRead(EC_SENS_PIN) / 1024.0 * 5000;
  float EC_TEMP = measurement.temperature;
  float EC_VALUE = EC10.readEC(EC_VOLT, EC_TEMP);
  temp["EC"] = EC_VALUE;
  EC10.calibration(EC_VOLT, EC_TEMP);
  PH.calibration(PH_VOLT, measurement.temperature);
  return temp;
}
