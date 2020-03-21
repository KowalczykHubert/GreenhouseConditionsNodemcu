#define BLYNK_PRINT Serial

#include <Credentials.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>


#include <OneWire.h>
#include <DallasTemperature.h>
// Data wire is plugged into pin 2 on the Arduino
#define ONE_WIRE_BUS 12
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);


#define SEALEVELPRESSURE_HPA (1013.25) // define sea level pressure
Adafruit_BME280 bme;                   // I2C Create an object of Adafruit_BME280 class

BlynkTimer timer;           // Create an object of BlynkTimer class
char auth[] = APIKEY;       // You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).

char ssid[] = SSIDHOME;   // Your WiFi credentials.
char pass[] = PASSHOME;   // Set password to "" for open networks.

int seconds =30; // Timer interval in seconds


float t_bme; // Declare the temperature variables (bme sensor)
float h_bme;
float t_ds;
//#define numOfMeasurements 3
//float h_bme_avg;
//float h_bme_meas[numOfMeasurements];



void checkWeather()
{

  
  //BME280
 /* delay(2000);
  h_bme_avg = 0;
  for (int i = 0; i < numOfMeasurements; i++)
  {
    delay(3000);
    h_bme_meas[i] = bme.readHumidity();
    h_bme_avg += h_bme_meas[i];
    Serial.print(h_bme_meas[i]);
    Serial.print(" ");
  }
  */
  t_bme = bme.readTemperature();
  h_bme = bme.readHumidity();
  //h_bme_avg /= numOfMeasurements;
  float p = (bme.readPressure() / 100.0F);
  float Alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
  Serial.println();
  Serial.println(".......BME280......");
  Serial.print("Temperature = ");
  Serial.print(t_bme);
  Serial.println(" °C");
  Serial.print("Humidity = ");
  Serial.print(h_bme); //h_bme_avg
  Serial.println(" %");
  Serial.print("Pressure = ");
  Serial.print(p);
  Serial.println(" hPa");
  Serial.print("Approx. Altitude = ");
  Serial.print(Alt);
  Serial.println(" m");
  Serial.println();
  
  sensors.requestTemperatures();
  t_ds = sensors.getTempCByIndex(0);
  Serial.print("Temperature dallas is: ");
  Serial.println(t_ds); // Why "byIndex"? You can have more than one IC on the same bus. 0 refers to the first IC on the wire
  //Update value every 1 sec.
  
  
  Blynk.virtualWrite(V0, (t_bme-1));
  Blynk.virtualWrite(V1, h_bme); //h_bme_avg
  Blynk.virtualWrite(V2, p);
  Blynk.virtualWrite(V3, t_ds);
  delay(500);
 
}


void setup() {
  delay(500);
  Serial.begin(9600);
  Serial.setTimeout(2000);
  Blynk.begin(auth, ssid, pass); // connection to internet
  bme.begin(0x76);
  sensors.begin(); // dallas
  Blynk.run();
  checkWeather();

  
 

  // Wait for serial to initialize.
  while(!Serial) { }
  
  Serial.println("I'm awake.");

  Serial.println("Going into deep sleep for 20 seconds");
  ESP.deepSleep(30e6); // 20e6 is 20 microseconds
}

void loop() {
}

/*
void setup()
{
  Serial.begin(9600);        // Debug console

  Blynk.begin(auth, ssid, pass); // connection to internet

  bme.begin(0x76);
  sensors.begin(); // dallas

  timer.setInterval(seconds * 1000L, checkPlants);
  checkPlants();
}

void loop()
{
  Blynk.run();
  timer.run();
}
*/