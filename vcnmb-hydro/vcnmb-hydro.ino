#include <WiFiEsp.h>
#include <WiFiEspClient.h>
#include <WiFiEspServer.h>
#include <DHTesp.h>
#include <ArduinoJson.h>
#include <DFRobot_EC10.h>
#include <DFRobot_PH.h>

#define AP_SSID "VCNMB-Hydro"
#define AP_PASS "hydro123"
#define EC_OUT_PIN 5
#define EC_IN_PIN 6
#define PH_OUT_PIN 7
#define PH_IN_PIN 12
#define CIRC_PUMP_PIN 9
#define FAN_1_PIN 10
#define FAN_2_PIN 11
#define LIGHT_1_PIN 8
#define TEMP_HUMID_SENS_PIN 50
#define FLOW_SENS_PIN 52
#define AMB_LIGHT_SENS_PIN A2
#define PH_SENS_PIN A3
#define EC_SENS_PIN A4
#define FLOW_CAL 0.00

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

  WiFi.configAP(SysIP);
  WiFi.beginAP(AP_SSID, 13, AP_PASS, ENC_TYPE_WPA2_PSK, false);
  WebServer.begin();

  // Relay PWM Pins, 5-12
  for (int i = 5; i <= 12; i++) {
    pinMode(i, OUTPUT);
    TogglePin(i);
  }
  // default on
  TogglePin(LIGHT_1_PIN);
  TogglePin(FAN_2_PIN);
  TogglePin(CIRC_PUMP_PIN);
}

void loop() {
  WiFiEspClient client = WebServer.available();
  if (!client) { return; }
  client.setTimeout(5000);
  String req = client.readStringUntil('\r');
  Serial.println(F("request: "));
  Serial.println(req);

  while (client.available()) {
    client.read();
  }
  client.print(
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: application/json\r\n"
    "Connection: close\r\n"
    "\r\n");

  if (req.indexOf("GET /hardware.json") >= 0) {
    serializeJsonPretty(HardwareToJson(), client);
  }
  if (req.indexOf("GET /sensor.json") >= 0) {
    serializeJsonPretty(SensorToJson(), client);
  }
  if (req.indexOf("GET /ph_in.json") >= 0) {
    TogglePin(PH_IN_PIN);
  }
  if (req.indexOf("GET /ph_out.json") >= 0) {
    TogglePin(PH_OUT_PIN);
  }
  if (req.indexOf("GET /circ_pump.json") >= 0) {
    TogglePin(CIRC_PUMP_PIN);
  }
  if (req.indexOf("GET /ec_in.json") >= 0) {
    TogglePin(EC_IN_PIN);
  }
  if (req.indexOf("GET /ec_out.json") >= 0) {
    TogglePin(EC_OUT_PIN);
  }
  if (req.indexOf("GET /fan_circ.json") >= 0) {
    TogglePin(FAN_2_PIN);
  }
  if (req.indexOf("GET /fan_extractor.json") >= 0) {
    TogglePin(FAN_1_PIN);
  }
  if (req.indexOf("GET /light.json") >= 0) {
    TogglePin(LIGHT_1_PIN);
  }
  client.stop();
  Serial.println(F("Disconnecting from client"));
}

void Detect_Rising_Edge_Flow() {
  Pulse_Count++;
}

void TogglePin(int Pin) {
  digitalWrite(Pin, !digitalRead(Pin));
}

DynamicJsonDocument HardwareToJson() {
  DynamicJsonDocument temp(128);
  temp["pH_In_Pump"] = digitalRead(PH_IN_PIN);
  temp["pH_Out_Pump"] = digitalRead(PH_OUT_PIN);
  temp["EC_In_Pump"] = digitalRead(EC_IN_PIN);
  temp["EC_Out_Pump"] = digitalRead(EC_OUT_PIN);
  temp["Circulation_Pump"] = digitalRead(CIRC_PUMP_PIN);
  temp["Fan_Extractor"] = digitalRead(FAN_1_PIN);
  temp["Fan_Tent"] = digitalRead(FAN_2_PIN);
  temp["Light"] = digitalRead(LIGHT_1_PIN);
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