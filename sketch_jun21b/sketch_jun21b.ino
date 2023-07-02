/*
  Optical Heart Rate Detection (PBA Algorithm) using the MAX30105 Breakout
  By: Nathan Seidle @ SparkFun Electronics
  Date: October 2nd, 2016
  https://github.com/sparkfun/MAX30105_Breakout

  This is a demo to show the reading of heart rate or beats per minute (BPM) using
  a Penpheral Beat Amplitude (PBA) algorithm.

  It is best to attach the sensor to your finger using a rubber band or other tightening
  device. Humans are generally bad at applying constant pressure to a thing. When you
  press your finger against the sensor it varies enough to cause the blood in your
  finger to flow differently which causes the sensor readings to go wonky.

  Hardware Connections (Breakoutboard to Arduino):
  -5V = 5V (3.3V is allowed)
  -GND = GND
  -SDA = A4 (or SDA)
  -SCL = A5 (or SCL)
  -INT = Not connected

  The MAX30105 Breakout can handle 5V or 3.3V I2C logic. We recommend powering the board with 5V
  but it will also run at 3.3V.
*/
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ThingSpeak.h>

#define alert D5
#define ONE_WIRE_BUS D6



//just change the following
#define WIFI_SSID "narzo 50A"     //your wifi name
#define WIFI_PASSWORD "ambuja@321"      //wifi password
unsigned long myChannelNumber =2099225 ;    //channel id from thingspeak
const char * myWriteAPIKey = "FG63J0NGI9IO4G6F";    //read API from thingspeak

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
MAX30105 particleSensor;

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

const int LM35_PIN = A0;

WiFiClient client;
float Celsius = 0;
float Fahrenheit = 0;

void setup()
{
  sensors.begin();
  pinMode(alert,OUTPUT);
  pinMode(LM35_PIN,INPUT);
  Serial.begin(115200);
  Serial.println("Initializing...");


  // Connect to Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");


  
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED


  ThingSpeak.begin(client);
}

void loop()
{
  sensors.requestTemperatures();
  Celsius = sensors.getTempCByIndex(0);
  Fahrenheit = sensors.toFahrenheit(Celsius);
  int sensorValue = analogRead(LM35_PIN);
  long irValue = particleSensor.getIR();

  // Convert the analog reading to temperature in degrees Celsius
  float voltage = (sensorValue * 5.0) / 1023.0;
  float temperature = (voltage - 0.5) * 100.0;
  temperature=100-temperature;

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

//  temperature=100-temperature;

  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);
  Serial.print("Temperature: ");
  Serial.print(Fahrenheit);
  Serial.print(" °C");
  if(Fahrenheit>100){
    digitalWrite(alert,HIGH);
    }

   else{
    digitalWrite(alert,LOW);
    }
  
  ThingSpeak.writeField(myChannelNumber,1,irValue,myWriteAPIKey);
  ThingSpeak.writeField(myChannelNumber,2,beatsPerMinute,myWriteAPIKey);
  ThingSpeak.writeField(myChannelNumber,3,beatAvg,myWriteAPIKey);
  ThingSpeak.writeField(myChannelNumber,4,Fahrenheit,myWriteAPIKey);
  
  if (irValue < 50000)
    Serial.print(" No finger?");

  Serial.println();
}

  
