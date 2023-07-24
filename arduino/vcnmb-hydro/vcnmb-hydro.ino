#include <SPI.h>
#include <WiFiEsp.h>
#include <WiFiEspClient.h>
#include <WiFiEspServer.h>

WiFiEspServer WebServer(80);

void setup() {
  // serial for logging
  Serial.begin(9600);
  // wifi initialisation
  Serial1.begin(115200);
  WiFi.init(&Serial1);
  // if no module connected, hang
  if (WiFi.status() == WL_NO_SHIELD) {
    while(true);
  }
  // if module connected, continue
  IPAddress SysIP(192, 168, 1, 10);
  WiFi.configAP(SysIP);
  // ssid, channel, pasword, security type
  WiFi.beginAP("VCNMB-Hydro", 13, "hydro123", ENC_TYPE_WPA2_PSK);
  WebServer.begin();
}

void loop() {
}
