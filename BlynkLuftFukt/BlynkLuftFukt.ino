
#include <DHT.h>
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include "EEPROM.h"

#define BLYNK_PRINT Serial
#define DHTPIN 4          // What digital pin we're connected to

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11     // DHT 11
#define DHTTYPE DHT22   // DHT 22, AM2302, AM2321
//#define DHTTYPE DHT21   // DHT 21, AM2301

DHT dht(DHTPIN, DHTTYPE);
BlynkTimer timer;
int relayPin = 5; // pin for the relay to switch on power to the dehumidifier
int addr = 0; // byteaddresse
unsigned long startMillis;  // value for counting on-time for the dehumidifier - its not good to switch it on/off to often
unsigned long stopMillis; // value for counting off-time for the dehumidifier
unsigned long currentMillis; // last on value for the dehumidifier
long waitTime = 60000; // 60000 = 1 minute, the amount of time to wait between dehumidifier startups..
long lastCheck = 0; // last check for relay on
long onTime = 0; // time running the dehumidifier

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "a8eb9fc739fc4b4f87e595d2507dd247";

// Your credentials.
// Set password to "" for open networks.
char ssid[] = "somenetwork";
char pass[] = "Secret with capital s";
int triggerHumidity = 45; // trigger for lowest humidity accepted
int maxTemp = 30; //sikkerhetsfunksjon for å unngå overoppheting..
float currentTemp =25.0;
float currentHumidity = 30.0;


void setup()
{
  // Debug console
  Serial.begin(9600);
  EEPROM.begin(512);
  Blynk.begin(auth, ssid, pass);
  dht.begin();
  int humValue = eeGetInt(addr); // henter lagret verdi om den finnes
  //int humValue = EEPROM.read(addr);
  if (humValue != 255)
  {
    triggerHumidity = humValue;
  }

  // Setup a function to be called every second
  timer.setInterval(1000L, sendSensor);
  pinMode(relayPin, OUTPUT);
}

void loop()
{
  Blynk.run();
  //Serial.print("Humidity trigger: ");
  //Serial.println(humidity);
  Dehumidify();
  timer.run();
}
void Dehumidify()
{
  // calculate time running the dehumidifier
  if(digitalRead(relayPin) == HIGH)
  {
    if (lastCheck > 0)
    {
      onTime = onTime + (millis() - lastCheck);
      lastCheck = millis();
    }
    else { lastCheck = millis(); }
  }
  Serial.print("Triggerhumidity: " );
  Serial.print(triggerHumidity);
  //float triggerHumidity = float(humidity);
  if (triggerHumidity < currentHumidity && currentTemp < float(maxTemp)) // checking that the room isnt on fire with the temp value
  {
      if ((millis() - stopMillis) > waitTime)
      {
        Serial.print("vått og kaldt og breiflabb over alt: ");
        Serial.print(millis() -stopMillis);
        Serial.print(" millis. ");
        digitalWrite(relayPin, HIGH);
//        Blynk.virtualWrite(V8, HIGH);
        Serial.println(currentHumidity);
      }
      else
      {
        //Serial.println("Come gather round people Wherever you roam and admit that the waters around you have grown.. we will sink like a stone..");
        Serial.println("venter på timer..");
      }
  }
  else
  {
    Serial.print("tørt og godt.. Triggerhumidity: ");   
    // checking if we need to switch off, then start the timer..
    if(digitalRead(relayPin) == HIGH)
    {
      stopMillis = millis();
      digitalWrite(relayPin, LOW);
//      Blynk.virtualWrite(V8, LOW);
    }
    Serial.print(currentHumidity);
    Serial.println(currentHumidity);
  }
}


void sendSensor()
{
  float h = dht.readHumidity();
  float t = dht.readTemperature(); // or dht.readTemperature(true) for Fahrenheit

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  currentHumidity = h;
  currentTemp = t;
  if(!Blynk.connected()){
    Serial.println("Not connected to Blynk server"); 
    Blynk.connect();  // try to connect to server with default timeout
  }
  else { // if we have a connection to the blynk cloud
    Blynk.virtualWrite(V5, h);
    Blynk.virtualWrite(V6, t);
    long rssi = WiFi.RSSI();
    Blynk.virtualWrite(V7, rssi);
    // incicator to the app for wether the demudifier is on or off
    if(digitalRead(relayPin) == HIGH)
    {
      Blynk.virtualWrite(V8, HIGH);
    }
    else {
      Blynk.virtualWrite(V8, LOW);
    }
    long onTimeTemp = 0;
    if (onTime > 60000)
    {
      onTimeTemp = onTime / 60000; // sender verdi i minutter
    }
    Blynk.virtualWrite(V9, onTimeTemp); 
  } // end blynk connection
}


// SLIDER
BLYNK_WRITE(V0) {  // reads value from slider, set range of 0-255, and passes it onto PWM pin

    //Blynk.syncVirtual(V1);
    int value = param.asInt();
    //EEPROM.write(addr, value); // triggerHumidity
    eeWriteInt(addr, value);
    triggerHumidity = value;
    Serial.println(value);

}

int eeGetInt(int pos) {
  int val;
  byte* p = (byte*) &val;
  *p        = EEPROM.read(pos);
  *(p + 1)  = EEPROM.read(pos + 1);
  *(p + 2)  = EEPROM.read(pos + 2);
  *(p + 3)  = EEPROM.read(pos + 3);
  return val;
}

void eeWriteInt(int pos, int val) {
    byte* p = (byte*) &val;
    EEPROM.write(pos, *p);
    EEPROM.write(pos + 1, *(p + 1));
    EEPROM.write(pos + 2, *(p + 2));
    EEPROM.write(pos + 3, *(p + 3));
    EEPROM.commit();
}
