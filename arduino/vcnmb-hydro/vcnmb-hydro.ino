#include <SPI.h>
#include <WiFiEsp.h>
#include <WiFiEspClient.h>
#include <WiFiEspServer.h>
#include <DHTesp.h>

WiFiEspServer WebServer(80);
DHTesp TempHumid;
volatile int  Pulse_Count;
unsigned long Current_Time, Loop_Time;

#define STATION_SSID "beef hotpot"
#define STATION_PASS "tastyNetwork"
#define AP_SSID "VCNMB-Hydro"
#define AP_PASS "hydro123"

void setup() {
  // serial for logging
  Serial.begin(115200);

  // temp and humidity initialisation, pin 40
  TempHumid.setup(50, 'AUTO_DETECT');

  // flow rate
  pinMode(51, INPUT);
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
  WiFi.begin(STATION_SSID, STATION_PASS);
  Serial.print("Connecting to ");
  Serial.print(STATION_SSID);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
    Serial.print(".");
  }
  //WiFi.configAP(SysIP);
  WiFi.beginAP(AP_SSID, 13, AP_PASS, ENC_TYPE_WPA2_PSK, false);
  Serial.println();
  //Serial.print(AP_SSID);
  Serial.print(STATION_SSID);
  Serial.print(" IP Address: ");
  Serial.print(WiFi.localIP());
  Serial.println();
  WebServer.begin();

  // relay test
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
}

void loop() {
  WiFiEspClient WebClient = WebServer.available();
  TempAndHumidity measurement = TempHumid.getTempAndHumidity();
  String RequestHeader = "";
  if (WebClient) {
    Serial.println("client connection started");
    while (WebClient.connected()) {
      if (WebClient.available()) {
        char c = WebClient.read();
        Serial.print(c);
        RequestHeader += c;
        if (c == '\n') {
          WebClient.println("HTTP/1.1 200 OK");
          WebClient.println("Content-type : text/html");
          WebClient.println("Connection: close");
          WebClient.println();
          WebClient.println("<!DOCTYPE HTML>");
          WebClient.println("<html>");
          WebClient.println("<h1>Test Web Server</h1>");

          if (RequestHeader.indexOf("GET /poll") >= 0) {
            Serial.println("polling all sensors");
            WebClient.print("<p>Current Temperature: ");
            WebClient.print(measurement.temperature);
            WebClient.println("</br>");
            WebClient.print("Current Humidity: ");
            WebClient.print(measurement.humidity);
            WebClient.print("</br>");
            WebClient.print("Current Ambient Light: ");
            WebClient.print(analogRead(0), DEC);
            WebClient.print("</br>");
            Current_Time = millis();
            if (Current_Time >= (Loop_Time + 1000)) {
              Loop_Time = Current_Time;
              Pulse_Count = 0;
              WebClient.print("Flow Rate (L/hour): ");
              WebClient.print((Pulse_Count * 60 / 7.5), DEC);
              WebClient.print( "</br></p>");
            }
          }

          if (RequestHeader.indexOf("GET /relay") >= 0) {
            Serial.print("toggling relay");
            for (int i = 8; i <= 13; i++) {
              digitalWrite(i, !digitalRead(i));
              delay(500);
            }
          }


          WebClient.println("</html>");
          break;
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
