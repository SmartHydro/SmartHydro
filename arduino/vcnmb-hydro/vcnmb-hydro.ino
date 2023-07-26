#include <SPI.h>
#include <WiFiEsp.h>
#include <WiFiEspClient.h>
#include <WiFiEspServer.h>
#include <DHTesp.h>
#include <ArduinoJson.h>

WiFiEspServer WebServer(80);
DHTesp TempHumid;
volatile int  Pulse_Count;
unsigned long Current_Time, Loop_Time;

#define STATION_SSID "beef hotpot"
#define STATION_PASS "tastyNetwork"
#define AP_SSID "VCNMB-Hydro"
#define AP_PASS "hydro123"

const unsigned int PUMP_1_PIN = 8;
const unsigned int PUMP_2_PIN = 9;
const unsigned int PUMP_3_PIN = 10;
const unsigned int FAN_1_PIN = 11;
const unsigned int FAN_2_PIN = 12;
const unsigned int LIGHT_1_PIN = 13;
const unsigned int TEMP_HUMID_SENS_PIN = 50;
const unsigned int FLOW_SENS_PIN = 51;
const unsigned int AMB_LIGHT_SENS_PIN = 0;

void setup() {
  // serial connection 
  Serial.begin(115200);

  // DHT22 initialisation 
  TempHumid.setup(TEMP_HUMID_SENS_PIN, 'AUTO_DETECT');

  // flow rate
  pinMode(FLOW_SENS_PIN, INPUT);
  attachInterrupt(0, Detect_Rising_Edge, FALLING);
  Current_Time = millis();
  Loop_Time = Current_Time;

  // Begin WiFi
  Serial1.begin(115200);
  WiFi.init(&Serial1);
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.print("no wifi shield, panicking!");
    while (true);
  }

  IPAddress SysIP(192, 168, 1, 10);
  //WiFi.begin(STATION_SSID, STATION_PASS);
  //Serial.print("Connecting to ");
  //Serial.print(STATION_SSID);
  //while (WiFi.status() != WL_CONNECTED)
  //{
  //delay(100);
  //Serial.print(".");
  //}
  WiFi.configAP(SysIP);
  WiFi.beginAP(AP_SSID, 13, AP_PASS, ENC_TYPE_WPA2_PSK, false);
  Serial.println();
  Serial.print(AP_SSID);
  //Serial.print(STATION_SSID);
  Serial.print(" IP Address: ");
  Serial.print(WiFi.localIP());
  Serial.println();
  WebServer.begin();

  // initialise relay pins to output mode
  for(int i = 8; i <= 13; i++) {
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
        }
      }
    }
    WebClient.stop();
    RequestHeader = "";
    Serial.println("client connection terminated");
  }
}

void Detect_Rising_Edge ()
{
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
  return temp;
}

DynamicJsonDocument SensorToJson() {
  DynamicJsonDocument temp(128);
  TempAndHumidity measurement = TempHumid.getTempAndHumidity();
  temp["Temperature"] = measurement.temperature;
  temp["Humidity"] = measurement.humidity;
  temp["LightLevel"] = analogRead(AMB_LIGHT_SENS_PIN);
  Current_Time = millis();
  if (Current_Time >= (Loop_Time + 1000)) {
    temp["FlowRate"] = (Pulse_Count * 60 / 7.5), DEC;
  }
  temp["pH"] = "N/A";
  temp["EC"] = "N/A";
  return temp;
}
