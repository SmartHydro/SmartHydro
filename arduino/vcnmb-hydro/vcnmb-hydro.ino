#include <SPI.h>
#include <WiFiEsp.h>
#include <WiFiEspClient.h>
#include <WiFiEspServer.h>
#include <DHTesp.h>

WiFiEspServer WebServer(80);
DHTesp TempHumid;

void setup() {
  // serial for logging
  Serial.begin(9600);
  // temp and humidity initialisation, pin 40
  TempHumid.setup(50, 'AUTO_DETECT');
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
}

void loop() {
  WiFiEspClient WebClient = WebServer.available();
  String RequestHeader = "";
  TempAndHumidity measurement = TempHumid.getTempAndHumidity();
  Serial.print("Temperature: ");
  Serial.print(measurement.temperature);
  Serial.println();
  delay(1000);
  if (WebClient) {
    Serial.println("client connection started");
    String currentLine = "";
    while (WebClient.connected()) {
      if (WebClient.available()) {
        char c = WebClient.read();
        Serial.print(c);
        RequestHeader += c;
        if (c == '\n') {
          if (currentLine.length() == 0) {
            WebClient.println("HTTP/1.1 200 OK");
            WebClient.println("Content-type:text/html");
            WebClient.println("Connection: close");
            WebClient.println();
            WebClient.println("<!DOCTYPE html><html>");
            WebClient.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            WebClient.println("<link rel=\"icon\" href=\"data:,\">");
            WebClient.println("<body><h1>Test Web Server</h1>");

            if (RequestHeader.indexOf("GET /temp") >= 0) {
              Serial.println("/temp requested");
              WebClient.print("<p>Current Temperature: ");
              WebClient.print(measurement.temperature);
              WebClient.print("</p>");
            }
            else if (RequestHeader.indexOf("GET /humid") >= 0) {
              Serial.println("/humid requested");
              WebClient.print("<p>Current Humidity: ");
              WebClient.print(measurement.humidity);
              WebClient.print("</p>");
            }
            else if (RequestHeader.indexOf("GET /light") >= 0) {
              Serial.println("/light requested");
              WebClient.print("<p>Current Ambient Light: ");
              WebClient.print(measurement.humidity);
              WebClient.print("</p>");
            }
            else if (RequestHeader.indexOf("GET /ping") >= 0) {
              Serial.println("/ping requested");
              WebClient.println("<p>pong</p>");
            }
            WebClient.println("</body></html>");
            WebClient.println();
            break;
          } else {
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }
    WebClient.stop();
    RequestHeader = "";
    Serial.println("client connection terminated");

  }
}
