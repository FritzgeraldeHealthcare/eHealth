


//-----style guard -----

#ifdef _cplusplus

    extern "C" {
  #endif
    uint8_t temprature_sens_read();//temprature sensor reading
#ifdef _cplusplus
}
#endif
uint8_t temprature_sens_read();

// -----header files----


#include <DHT.h>  // Including library for dht
#include "ThingSpeak.h" 
#include "MQ135.h"
#include <WiFi.h>


#include <OneWire.h>
#include <DallasTemperature.h>

#define SENSOR A0 // Set the A0 as SENSOR GPIO36 OR SP FOR ECG
#define CAPTEUR 32//MQ135 Air Quality Sensor GPIO32
/*
 * (GPIO 21 = SDA, GPIO 22 = SCL)
 * OUTPUT GPIO36 Sp
 * Lo- GPIO10 SD3
 * Lo+ GPIO09 SD2
 */

OneWire ourWire(15); //Creating a OneWire object on ESP32 GPIO 15, formally GPIO4
DallasTemperature DS18B20(&ourWire); //A variable or object is declared for the sensor
 
///String apiKey = "23MXN6KOCINKKGGZ"; //  Enter your Write API key from ThingSpeak

///const char *ssid = "Airbox-3D39";
const char *ssid =  "Infinix SMART 5"; // replace with your wifi ssid and wpa2 key
const char *pass =  "93627123";
const char* server = "api.thingspeak.com";

unsigned long channelID = 2179831;
const char* WriteAPIKey = "TIY1JLAZH6BBHLOM";
 
#define DHTPIN 4 // Digital pin connected to the dht sensor
#define DHTTYPE DHT22//for dht22 module
 
///DHT dht(DHT_SENSOR_PIN, DHT22);

//Now initializing the dht22 sensor
DHT dht(DHTPIN, DHTTYPE);

///WiFiServer server(80);
 
WiFiClient client;

//------Timer Variables

unsigned long lastTime = 0;
unsigned long timerDelay = 19000;

char str_ecg_sensor[10];

///DHT dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);

/****************************************
  Principal Functions
  ****************************************/
 
void setup(){
  Serial.begin(115200);
  pinMode(SENSOR, INPUT);
  pinMode(CAPTEUR, INPUT);
  Serial.print("Connecting to WiFi : "); //Print WiFi network connection message
  Serial.println(ssid); 

  WiFi.begin(ssid, pass); //WIFi connection starts
   // Connect or reconnect to WiFi
    if(WiFi.status() != WL_CONNECTED){
      Serial.print("Attempting to connect to : ");
      Serial.print(ssid);
      while(WiFi.status() != WL_CONNECTED){
        WiFi.begin(ssid, pass); 
        delay(500);/// normally 15s not 1s  
      } 
      Serial.println("\nConnection Established \n.");
    } 
  ThingSpeak.begin(client); //Initialize the ThingSpeak Server
  
  Serial.println(F("DHT | DS18B20 | Air Quality | ECG | Hall Value | test string!"));
  
  dht.begin();
  DS18B20.begin(); //Start DS18B20 sensor
  delay(1000);
}

void dht22_data(){
  //Wait a few seconds between measurements.
  delay(2500);
  //Sensor readings may also be up to 2s old, sensor is slow

  float h = dht.readHumidity();

  //Read temperature as Celsius (the default)
  float t = dht.readTemperature();

  //Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  Serial.print(F("Humidity(%): "));
  Serial.println(h);
  Serial.print(F("Room Temp.: "));
  Serial.print(t);
  Serial.println(F("°C"));
  Serial.print(F("Room Temp.: "));
  Serial.print(f);
  Serial.println(F("°F"));
  delay(2000);
  //Sign in ThingSpeak to field 1 & 2 the temperature reading
  ThingSpeak.setField(1, h);
  ThingSpeak.setField(2, t);
}

//Function to read the sensor and assign the fields to ThingSpeak
void leer_sensor_DS18B20() {
  DS18B20.requestTemperatures(); //The command to read the temperature is sent
  float body_temp = DS18B20.getTempCByIndex(0); //The temperature is obtained in ºC
  float tem = body_temp + 3;
  Serial.print("Body Temperature: ");
  Serial.print(tem);
  Serial.println(" °C");
  delay(200);
  //Sign in ThingSpeak to field 3 the temperature reading
  ThingSpeak.setField(3, tem);
 }

void Air_Quality(){
  float MQ135_data = analogRead(CAPTEUR);// Normally A0
    if (MQ135_data < 500){
      Serial.print("Fresh Air Quality: ");
    }
    else if(MQ135_data > 500 && MQ135_data < 1000){
      Serial.print("Moderate Air Quality: ");
    }
    else{
      Serial.print("Poor Air Quality: ");
    }
    Serial.print(MQ135_data);
    Serial.println(" PPM");
    delay(500);
    //Sign in ThingSpeak to field 1 the temperature reading
    ThingSpeak.setField(4, MQ135_data);

}

void ECG_Rhythm(){
   float ecg_sensor = analogRead(SENSOR); 
   Serial.print("ECG Value in bytes is : ");
   Serial.println(ecg_sensor);
   ThingSpeak.setField(5, ecg_sensor);
 }

void Hall_Value(){
  int hal = 0;//Hall value reading mV/T
  hal = hallRead(); 
  Serial.print("Hall Value : ");
  Serial.println(hal);
  ThingSpeak.setField(6, hal);
}
       
void loop(){
  if (millis() - lastTime > timerDelay) {
  dht22_data();//Room Temperature function
  leer_sensor_DS18B20(); //Body Temperature function
  Air_Quality();// Air Quality in PPM (CO, NO2, SMOKE, ALCOHOL)
  ECG_Rhythm();//ECG Rhythm function
  Hall_Value();
  //Stream the data to the ThingSpeak server
  int datum = ThingSpeak.writeFields(channelID, WriteAPIKey);
  if (datum == 200){
  Serial.println("Data sent to ThingSpeak\n");
  }
  else {
    Serial.println("Problem updating - error HTTP " + String(datum));
  }
  delay(19000);
  lastTime = millis();
  }
}
 
