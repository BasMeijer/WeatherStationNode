// ----------------------------------------------------
// LIB IMPORTS
// ----------------------------------------------------

#include <Wire.h>
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "DHT.h"
#include "MQ135.h"

// ----------------------------------------------------
// PIN & SENSOR DEFINITIONS 
// ----------------------------------------------------

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

// HALL PINS
int hallSensorPin = D7;


Adafruit_BMP280 bme; // I2C
DHT dht(DHTPIN, DHTTYPE);

// ----------------------------------------------------
// WIFI & NETWORK SETTINGS
// ----------------------------------------------------

byte ledPin = 2;
char ssid[] = "PinguHotspot";               // SSID of your home WiFi
char password[] = "basbas1337";               // password of your home WiFi

IPAddress server(192, 168, 43, 198);       // the fix IP address of the server
WiFiClient client;

// ----------------------------------------------------
// DATA VARIABLES & SENSOR  READINGS
// ----------------------------------------------------

float humidity;
float temperature_DHT;
float temperature_BMP;
float airPressure;
float pressureAltitude;
// Rain Sensor
int nRainVal;
boolean bIsRaining = false;
String strRaining;
// Hall Sensor
int state = 0;
// Air Speed
float airspeed  = 0;
int rounds = 0;
int windReading = 0;
// json string
String dataToSend;

// MAIN LOOP INTERVAL // CURRENTLY 1 MIN
const unsigned long intervalTime = 1 * 60 * 1000UL;
static unsigned long lastSampleTime = 0 - intervalTime;  // initialize such that a reading is due the first time through loop()


// ----------------------------------------------------
// HOMESTATION OUTSIDE STARTUP FUNCTIONS
// ----------------------------------------------------

void setup() {
  Serial.begin(115200);
  
  dht.begin();
  
  pinMode(2,INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  connectToWifiNetwork();

  delay(2000);
}

// ----------------------------------------------------
// MAIN LOOP FUNCTIONALITY
// ----------------------------------------------------

void loop() {

  if (!bme.begin()) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
  
  // Hall / AirSpeed RPM calculations
  state = digitalRead(hallSensorPin);

  if (state == LOW) {
    if(windReading != 1){
      rounds +=1;
      Serial.println("New anemometer round");
      Serial.println(rounds);
    }
    windReading = 1;
  } 
  else {
     windReading = 0;
  }

  // RUNS EVERY MINUTE DEFINED BY THE INTERVALTIME
  unsigned long now = millis();
   if (now - lastSampleTime >= intervalTime)
   {
    lastSampleTime += intervalTime;
      
    Serial.println("Sending Data");
  
    // Read sensor values
    humidity = dht.readHumidity();
    temperature_DHT = dht.readTemperature();
    temperature_BMP = bme.readTemperature();
    airPressure = bme.readPressure();
    pressureAltitude = bme.readAltitude(1015);
    bIsRaining = !(digitalRead(nRainDigitalIn));
    calculateAirSpeed();

    delay(1000);
    
    // Build the json format 
    dataToSend = buildDataFormat();
    Serial.println(dataToSend); 
  
    sendDataToHub();
    delay(5000);
   }
}

void calculateAirSpeed() {
  // reset the airspeed before calculating
  airspeed = 0;
  
  Serial.println("Calculating wind speed");
  Serial.print("RPM=");
  Serial.println(rounds);
  
  airspeed = (2*3,14*140)*rounds; // mm/min (2*pi*straal)

  airspeed = airspeed/1000000; // bereken km/min

  airspeed = airspeed*60; // km/hour

  Serial.print("Read airspeed, speed=");
  Serial.println(airspeed);

  // reset the rounds for next readings.
  rounds = 0;
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



