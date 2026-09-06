#include <ESP8266WiFi.h>

void setup() {
  Serial.begin(115200);
  
  // Increase this delay to 3-5 seconds to ensure the connection 
  // is fully established before printing.
  delay(3000); 

  WiFi.mode(WIFI_STA);

  Serial.println("\n");
  Serial.print("Full MAC Address: ");
  Serial.println(WiFi.macAddress());
}

void loop() {}
