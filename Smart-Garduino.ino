/*
 WiFiEsp example: ConnectWPA
 
 This example connects to an encrypted WiFi network using an ESP8266 module.
 Then it prints the  MAC address of the WiFi shield, the IP address obtained
 and other network details.

 For more details see: http://yaab-arduino.blogspot.com/p/wifiesp-example-connect.html
*/
#include <SoftwareSerial.h>
#include "WiFiEsp.h"
#include <Adafruit_Sensor.h>
#include "DHT.h"


// Emulate Serial1 on pins 6/7 if not present
#ifndef HAVE_HWSERIAL1
SoftwareSerial Serial1(6, 7); // RX, TX
#endif

char ssid[] = "";          // your network SSID (name) 
char pass[] = "";    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;  // the Wifi radio's status

int air_temp[20];
int air_humidity[20];
int soil_moisture[20];
int soil_moisture_wet[20];      // array of vwc (volumetric water content) data right after watering the garden
float increase_in_vwc;


#define soil_moisture_pin A0
#define watervalve_pin 2         // control signal for water valve
#define DHTPIN 3                 // Air Temperature and Humidity Sensor
#define DHTTYPE DHT22
#define VWC_THRESHOLD 20.0       // the moisture level that triggers the watering system (20% VWC)

#define AMOUNT_WATERED 200.0                   // Amount of water dispensed, assumed to be 200mL  

DHT dht(DHTPIN, DHTTYPE);   

void setup()
{
  pinMode(watervalve_pin, OUTPUT);
  digitalWrite(watervalve_pin, LOW);
  dht.begin();                    // initialize Temperature/Humidity Sensor
  
  // initialize serial for debugging
  Serial.begin(230400);
  // initialize serial for ESP module
  Serial1.begin(115200);
  // initialize ESP module
  WiFi.init(&Serial1);

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue
    while (true);
  }

  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
  }

  Serial.println("You're connected to the network");
}

float mapfloat(long x, long in_min, long in_max, long out_min, long out_max)
{
  return ((float)((x - in_min) * (out_max - out_min))) / ((float)(in_max - in_min)) + out_min;
}

int* getHumidityData(){
  return air_humidity;
}

int* getTemperatureData(){
  return air_temp;
}

int* getMoistureData(){
  return soil_moisture;
}

float getVwcIncrease(){
  return increase_in_vwc;
}

void loop()
{
  // Print the network connection information every 10 seconds
  //printCurrentNet();
  //printWifiData();
  //delay(10000);
  //AMOUNT_WATERED

  // Collect air temperature and air relative humidity data
  for(int i = 0; i < sizeof(air_humidity)/2; i++){
    air_temp[i] = dht.readTemperature();      // Gets the values of the temperature
    air_humidity[i] = dht.readHumidity();     // Gets the values of the humidity
    
    // Printing the results on the serial monitor
    Serial.print("Temperature = ");
    Serial.print(air_temp[i]);
    Serial.print(" *C ,");
    Serial.print("    Humidity = ");
    Serial.print(air_humidity[i]);
    Serial.println(" % ");
    
    delay(2000); // Delays 2 secods, as the DHT22 sampling rate is 0.5Hz
  }

  // Collect soil moisture data
  int sum = 0;
  float reading;
  float vwc;
  
  for(int i = 0; i < sizeof(soil_moisture)/2; i++){
    reading = analogRead(soil_moisture_pin);
    reading = mapfloat(reading, 0, 1023, 0, 5);
                         
    if (reading <= 1.1){        // get volumetric water content (soil moisture level)
      vwc = 10*reading - 1.0;
      if (vwc < 0)
        vwc = 0;
    }else if (reading <= 1.3){
      vwc = 25*reading - 17.5;
    }else if (reading <= 1.82){
      vwc = 48.08*reading - 47.5;
    }else if (reading <= 2.2){
      vwc = 26.32*reading - 7.89;
    }else{
      vwc = 62.5*reading - 87.5;
    }
    Serial.println("Reading: " + String(reading));
    Serial.println("VWC: " + String(vwc));

    soil_moisture[i] = vwc;
    sum += vwc;
    delay(700);
  }
  
  float average_vwc = (float)sum/(float)(sizeof(soil_moisture)/2);
  if (average_vwc <= VWC_THRESHOLD){          // Turn the Watering Valve ON for 19 seconds
      digitalWrite(watervalve_pin, HIGH);
      delay(19000);       

      // get vwc (volumetric water content) readings right after watering. Data could be useful in the future
      sum = 0;
      for (int i = 0; i < sizeof(soil_moisture_wet)/2; i++){
        analogRead(soil_moisture_pin);
        reading = mapfloat(reading, 0, 1023, 0, 3);
        
        if (reading <= 1.1){
          vwc = 10*reading - 1.0;
          if (vwc < 0)
            vwc = 0;
        }else if (reading <= 1.3){
          vwc = 25*reading - 17.5;
        }else if (reading <= 1.82){
          vwc = 48.08*reading - 47.5;
        }else if (reading <= 2.2){
          vwc = 26.32*reading - 7.89;
        }else{
          vwc = 62.5*reading - 87.5;
        }

        Serial.println("Reading: " + String(reading));
        Serial.println("VWC: " + String(vwc));
    
        soil_moisture_wet[i] = vwc;
        sum += vwc;
        delay(700);
      }
      Serial.println("Size of array: "+String((float)(sizeof(soil_moisture_wet)/2)));
      increase_in_vwc = ((float)sum / (float)(sizeof(soil_moisture_wet)/2)) - average_vwc;
      Serial.println("Increase in VWC: "+String(increase_in_vwc));
    }
  
  // Collect data samples once every 20 minutes (20 min * 60 seconds/min * 1000 ms/second = 1200000 ms)
  delay(1200000);
}


// For Troubleshooting Wi-Fi Connection
void printWifiData()
{
  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print your MAC address
  byte mac[6];
  WiFi.macAddress(mac);
  char buf[20];
  sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
  Serial.print("MAC address: ");
  Serial.println(buf);
}

void printCurrentNet()
{
  // print the SSID of the network you're attached to
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print the MAC address of the router you're attached to
  byte bssid[6];
  WiFi.BSSID(bssid);
  char buf[20];
  sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X", bssid[5], bssid[4], bssid[3], bssid[2], bssid[1], bssid[0]);
  Serial.print("BSSID: ");
  Serial.println(buf);

  // print the received signal strength
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI): ");
  Serial.println(rssi);
}
