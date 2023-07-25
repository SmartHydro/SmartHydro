#include <SPI.h>
#include <WiFiEsp.h>
#include <WiFiEspClient.h>
#include <WiFiEspServer.h>
#include <DHTesp.h>

WiFiEspServer WebServer(80);
DHTesp TempHumid;
volatile int  Pulse_Count;
unsigned int  Liter_per_hour;
unsigned long Current_Time, Loop_Time;

void setup() {
  // serial for logging
  Serial.begin(9600);
  // temp and humidity initialisation, pin 50
  TempHumid.setup(50, 'AUTO_DETECT');
  // flow rate setup, pin 51
  pinMode(51, INPUT);
  attachInterrupt(0, Detect_Rising_Edge, RISING);
  Current_Time = millis();
  Loop_Time = Current_Time;
  // wifi initialisation, pin rx1 & tx1
  Serial1.begin(115200);
  WiFi.init(&Serial1);
  // if no module connected, hang
  if (WiFi.status() == WL_NO_SHIELD) {
    while (true);
  }
  // if module connected, continue
  IPAddress SysIP(192, 168, 1, 10);
  WiFi.configAP(SysIP);
  // ssid, channel, pasword, security type
  WiFi.beginAP("VCNMB-Hydro", 13, "hydro123", ENC_TYPE_WPA2_PSK);
  WebServer.begin();
  pinMode(2, OUTPUT);

}

void loop() {
  digitalWrite(2, HIGH);
  WiFiEspClient WebClient = WebServer.available();
  String RequestHeader = "";
  TempAndHumidity measurement = TempHumid.getTempAndHumidity();

  if (WebClient) {
    Serial.println("client connection started");
    boolean currentLineIsBlank = true;
    String currentLine = "";
    while (WebClient.connected()) {
      if (WebClient.available()) {
        char c = WebClient.read();
        Serial.print(c);
        RequestHeader += c;
        if (c == '\n') {
          WebClient.println("HTTP/1.1 200 OK");
          WebClient.println("Content-type:text/html");
          WebClient.println("Connection: close");
          WebClient.println();
          WebClient.println("<!DOCTYPE html><html>");
          WebClient.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
          WebClient.println("<link rel=\"icon\" href=\"data:,\">");
          WebClient.println("<body><h2>VCNMB-Hydroponics</h2>");
          if (RequestHeader.indexOf("GET /temp") >= 0) {
            Serial.println("/temp requested");
            WebClient.print("<p>Current Temperature: ");
            WebClient.print(measurement.temperature);
            WebClient.print("</p>");
          } else if (RequestHeader.indexOf("GET /humid") >= 0) {
            Serial.println("/humid requested");
            WebClient.print("<p>Current Humidity: ");
            WebClient.print(measurement.humidity);
            WebClient.print("</p>");
          } else if (RequestHeader.indexOf("GET /light") >= 0) {
            Serial.println("/light requested");
            WebClient.print("<p>Current Ambient Light: ");
            WebClient.print(analogRead(0), DEC);
            WebClient.print("</p>");
          } else if (RequestHeader.indexOf("GET /flow") >= 0) {
            Serial.println("/flow requested");
            Current_Time = millis();
            if (Current_Time >= (Loop_Time + 1000)) {
              Loop_Time = Current_Time;
              Liter_per_hour = (Pulse_Count * 60 / 7.5);
              Pulse_Count = 0;
              WebClient.print("<p>Flow Rate (L/hour): ");
              WebClient.print(Liter_per_hour, DEC);
              WebClient.println("</p>");
            }
          } else if (RequestHeader.indexOf("GET /status") >= 0) {
            Serial.println("/status recieved");
            WebClient.print("<p>Current Temperature: ");
            WebClient.print(measurement.temperature);
            WebClient.print("</p></br><p>Current Humidity: ");
            WebClient.print(measurement.humidity);
            WebClient.print("</p></br><p>Current Ambient Light: ");
            WebClient.print(analogRead(0), DEC);
            Current_Time = millis();
            if (Current_Time >= (Loop_Time + 1000)) {
              Loop_Time = Current_Time;
              Liter_per_hour = (Pulse_Count * 60 / 7.5);
              Pulse_Count = 0;
              WebClient.print("</p></br><p>Flow Rate (L/hour): ");
              WebClient.print(Liter_per_hour, DEC);
            }    WebClient.print("</p></br><p>WiFi Status: ");
            WebClient.print("blah</p>");
          } else if (RequestHeader.indexOf("GET /ping") >= 0) {
            Serial.println("/ping requested");
            WebClient.println("<p>pong</p>");
          }
        }
        WebClient.println("</body></html>");
        WebClient.println();
        break;
      }
    }
  }
  WebClient.stop();
  RequestHeader = "";
  Serial.println("client connection terminated");
}

void Detect_Rising_Edge ()
{
  Pulse_Count++;
}
