#include <SPI.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include "DHT.h"
#include "MQ135.h"

#define DHTPIN 2
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);
HTTPClient http;

//const char *ssid = "#";
//const char *password = "#";
byte ledPin = 2;
char ssid[] = "#";               // SSID of your home WiFi
char password[] = "#";               // password of your home WiFi

IPAddress server(192,168,0,80);       // the fix IP address of the server
WiFiClient client;

void setup() {
  Serial.begin(115200);
  dht.begin();
  
  connectToWifiNetwork();
}

void loop() {
  
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  StaticJsonBuffer<300> jsonBuffer;
  JsonObject& jsonRoot = jsonBuffer.createObject();
  
  
  JsonArray& data = jsonRoot.createNestedArray("data");

  JsonObject& humidityObject = jsonBuffer.createObject();
  humidityObject["key"] = "humidity";
  humidityObject["value"] = humidity;

  JsonObject& temperatureObject = jsonBuffer.createObject();
  temperatureObject["key"] = "temperature";
  temperatureObject["value"] = temperature;

  data.add(humidityObject);
  data.add(temperatureObject);
  
  String dataToSend;
  jsonRoot.printTo(dataToSend);

  Serial.println(dataToSend);

  client.connect(server, 80);   // Connection to the server
  digitalWrite(ledPin, LOW);    // to show the communication only (inverted logic)
  Serial.println(".");
  client.println(dataToSend);  // sends the message to the server
  String answer = client.readStringUntil('\r');   // receives the answer from the sever
  Serial.println("from server: " + answer);
  client.flush();
  digitalWrite(ledPin, HIGH);
   
  delay(5000);
}

void connectToWifiNetwork() {
  WiFi.begin(ssid, password);             // connects to the WiFi router
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("Connected to wifi");
  Serial.print("Status: "); Serial.println(WiFi.status());    // Network parameters
  Serial.print("IP: ");     Serial.println(WiFi.localIP());
  Serial.print("Subnet: "); Serial.println(WiFi.subnetMask());
  Serial.print("Gateway: "); Serial.println(WiFi.gatewayIP());
  Serial.print("SSID: "); Serial.println(WiFi.SSID());
  Serial.print("Signal: "); Serial.println(WiFi.RSSI());
  pinMode(ledPin, OUTPUT);
}


