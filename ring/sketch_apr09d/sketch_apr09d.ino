#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ThingSpeak.h>

#define WIFI_SSID "narzo 50A"
#define WIFI_PASSWORD "ambuja@321"
unsigned long myChannelNumber = 2099225;
const char * myWriteAPIKey = "FG63J0NGI9IO4G6F";
//#define THINGSPEAK_APIKEY "FHNLGWXLEJA9L0AX"
//#define THINGSPEAK_CHANNELID 2099225

MAX30105 particleSensor;

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;
int lst;

WiFiClient client;

void setup()
{

  pinMode(D3,OUTPUT);
  Serial.begin(9600);
  Serial.println("Initializing...");

  // Connect to Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Initialize ThingSpeak
  

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

  
  long irValue = particleSensor.getIR();

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

      // Send data to ThingSpeak
//      ThingSpeak.setField(1, beatsPerMinute);
//      ThingSpeak.setField(2, beatAvg);
//      ThingSpeak.writeFields(THINGSPEAK_CHANNELID, THINGSPEAK_APIKEY);

      
    }
  }

  if(irValue<3000){
    digitalWrite(D3,HIGH);
    }

  else{
    digitalWrite(D3,LOW);
    }
  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);
  lst=(lst+irValue)/2;
  beatsPerMinute=lst;
  ThingSpeak.writeField(myChannelNumber,1,irValue,myWriteAPIKey);
  ThingSpeak.writeField(myChannelNumber,2,beatsPerMinute,myWriteAPIKey);
  if (irValue < 50000)
    Serial.print(" No finger?");

  Serial.println();

  // Wait for 15 seconds before taking the next reading
  delay(5000);
}
