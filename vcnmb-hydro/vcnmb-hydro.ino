#include <WiFiEsp.h>
#include <WiFiEspClient.h>
#include <WiFiEspServer.h>
#include <DHTesp.h>
#include <DFRobot_EC10.h>
#include <DFRobot_PH.h>

#define AP_SSID "VCNMB-Hydro"
#define AP_PASS "hydro123"
#define EC_OUT_PIN 8
#define EC_IN_PIN 7
#define PH_OUT_PIN 6
#define PH_IN_PIN 5
#define CIRC_PUMP_PIN 9
#define FAN_1_PIN 10
#define FAN_2_PIN 11
#define LIGHT_1_PIN 12
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
  attachInterrupt(0, Detect_Rising_Edge_Flow, RISING);
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
  client.setTimeout(10000);
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
    client.println(HardwareToJson());
    Serial.println(HardwareToJson());
  }
  if (req.indexOf("GET /sensor.json") >= 0) {
    client.println(SensorToJson());
    Serial.println(SensorToJson());
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

String HardwareToJson() {
  return "{\n \"pH_In_Pump\" : \"" + String(digitalRead(PH_IN_PIN)) + "\",\n \"pH_Out_Pump\" : \"" + String(digitalRead(PH_OUT_PIN)) + "\",\n \"EC_In_Pump\" : \"" + String(digitalRead(EC_IN_PIN)) + "\",\n \"EC_Out_Pump\" : \"" + String(digitalRead(EC_OUT_PIN)) + "\",\n \"Circulation_Pump\" : \"" + String(digitalRead(CIRC_PUMP_PIN)) + "\",\n \"Fan_Extractor\" : \"" + String(digitalRead(FAN_1_PIN)) + "\",\n \"Fan_Tent\" : \"" + String(digitalRead(FAN_2_PIN)) + "\",\n \"Light\" : \"" + String(digitalRead(LIGHT_1_PIN)) + "\"\n}";
}

String SensorToJson() {
  String returnString = "{\n";
  TempAndHumidity measurement = TempHumid.getTempAndHumidity();
  returnString += " \"Temperature\" : \"" + String(measurement.temperature) + "\",\n \"Humidity\" : \"" + String(measurement.humidity) + "\",\n \"LightLevel\" : \"" + String(analogRead(AMB_LIGHT_SENS_PIN)) + "\",\n";
  Current_Time = millis();
  if (Current_Time >= (Loop_Time + 1000)) {
    returnString += " \"FlowRate\" : \"" + String(((Pulse_Count * 60 / 7.5) + FLOW_CAL)) + "\",\n";
  }
  float PH_VOLT = analogRead(PH_SENS_PIN) / 1024.0 * 5000;
  float EC_VOLT = analogRead(EC_SENS_PIN) / 1024.0 * 5000;
  returnString += " \"pH\" : \"" + String(PH.readPH(PH_VOLT, measurement.temperature)) + "\",\n \"EC\" : \"" + String(EC10.readEC(EC_VOLT, measurement.temperature)) + "\"\n}";
  PH.calibration(PH_VOLT, measurement.temperature);
  EC10.calibration(EC_VOLT, measurement.temperature);
  return returnString;
}

void Detect_Rising_Edge_Flow() {
  Pulse_Count++;
}

void TogglePin(int Pin) {
  digitalWrite(Pin, !digitalRead(Pin));
}