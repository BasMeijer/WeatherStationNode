// ----------------
// LIBRARY INCLUDES
// ----------------
#include <Wire.h>
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "DHT.h"
#include "MQ135.h"


// ----------------
// PIN DEFINITIONS
// ----------------
// DHT PINS
#define DHTPIN 2
#define DHTTYPE DHT22

// BMP PINS
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11 
#define BMP_CS 10

// RAIN PINS
int nRainIn = A0;
int nRainDigitalIn = D6;
int nRainVal;
boolean bIsRaining = false;
String strRaining;

// HALL PINS
int hallSensorPin = D7;
int state = 0;

// Wind speed
int windspeed;
int rounds;

// Loop Counter
int loopCount = 0;

Adafruit_BMP280 bme; // I2C
DHT dht(DHTPIN, DHTTYPE);

// ----------------
// WIFI SETTINGS 
// ----------------

byte ledPin = 2;
char ssid[] = "PinguHotspot";               // SSID of your home WiFi
char password[] = "basbas1337";               // password of your home WiFi

IPAddress server(192, 168, 43, 198);       // the fix IP address of the server
WiFiClient client;

// ----------------
// DATA TYPES
// ----------------
// SENSOR READINGS
float humidity;
float temperature_DHT;
float temperature_BMP;
float airPressure;
float pressureAltitude;
float airspeed = 5;
// json string
String dataToSend;

// Runs at startup, initialze sensors and connect to wifi network.
void setup() {
  Serial.begin(115200);
  dht.begin();
  pinMode(2,INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  connectToWifiNetwork();
}

void loop() {

  if (!bme.begin()) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }

  // Read sensor values
  humidity = dht.readHumidity();
  temperature_DHT = dht.readTemperature();
  temperature_BMP = bme.readTemperature();
  airPressure = bme.readPressure();
  pressureAltitude = bme.readAltitude(1015);
  bIsRaining = !(digitalRead(nRainDigitalIn));

  // Build the json format 
  dataToSend = buildDataFormat();
  Serial.println(dataToSend); 

  state = digitalRead(hallSensorPin);

//  if (state == LOW) {
//    Serial.println("low");
//  } 
//  else {
//    Serial.println("high");
//  }


  if(loopCount > 200){
    Serial.println("Haaai");
    loopCount = 0;  
  }else{
    loopCount += 1;
    }
  sendDataToHub();

  // Delay at which to send data to main hub.
  delay(10000);
}

void calculateWindSpeed() {

  // FIX ROUNDS
  int fixed_rounds = rounds;
  
  windspeed = (2*3,14*60)*fixed_rounds; // mm/min (2*pi*straal)
  windspeed = windspeed * 10; // from 6 seconds to 60 seconds
  windspeed = windspeed/1000000; // km/min
  windspeed = windspeed*60; // km/hour
  
  if (windspeed > 130) {
    windspeed = 0;
  }

  Serial.print("Read wind speed, speed=");
  Serial.println(windspeed); 
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

void sendDataToHub(){
  client.connect(server, 80);   // Connection to the server
  digitalWrite(ledPin, LOW);    // to show the communication only (inverted logic)
  client.println(dataToSend);  // sends the message to the server
  String answer = client.readStringUntil('\r');   // receives the answer from the sever
  Serial.println("from server: " + answer);
  client.flush();
  digitalWrite(ledPin, HIGH);
}

String buildDataFormat(){
  String dataToSend = String(humidity) + ";" + String((temperature_DHT + temperature_BMP) / 2)+ ";" + String(airPressure) + ";" + String(bIsRaining)+ ";" + String(airspeed);

  Serial.println("------ Formatting Data ----------");
  
  Serial.println("Humidity");
  Serial.println(humidity);

  Serial.println("Temperature DHT");
  Serial.println(temperature_DHT);

  Serial.println("Temperature BMP");
  Serial.println(temperature_BMP);

  Serial.println("Temperature Average");
  Serial.println((temperature_DHT + temperature_BMP) / 2);

  Serial.println("Is Raining");
  Serial.println(bIsRaining);

  Serial.println("Airspeed");
  Serial.println(airspeed);

  Serial.println("Airpressure");
  Serial.println(airPressure);

  Serial.println("---------------------------------");
  
  return dataToSend;
}



