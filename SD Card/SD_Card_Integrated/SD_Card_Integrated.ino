//#include "WiFiEsp.h"
#include <Adafruit_Sensor.h>
#include "DHT.h"

#include <SD.h>
#include <SPI.h>
#include <TimeLib.h>

#define soil_moisture_pin A0
#define watervalve_pin 2         // control signal for water valve
#define DHTPIN 3                 // Air Temperature and Humidity Sensor
#define DHTTYPE DHT22
int lightpin = A3;


#define VWC_THRESHOLD 20.0       // the moisture level that triggers the watering system (20% VWC)
#define ARRAY_SIZE 5.0



// Emulate Serial1 on pins 6/7 if not present
#ifndef HAVE_HWSERIAL1
#include <SoftwareSerial.h>
SoftwareSerial Serial1(6, 7); // RX, TX
#endif

char ssid[] = "MMS";             // your network SSID (name) 
char pass[] = "steven678";       // your network password (use for WPA, or use as key for WEP)
//int status = WL_IDLE_STATUS;     // the Wifi radio's status

int air_temp[(int)ARRAY_SIZE];
int air_humidity[(int)ARRAY_SIZE];
float soil_moisture[(int)ARRAY_SIZE];
float soil_moisture_wet[(int)ARRAY_SIZE];     // array of vwc (volumetric water content) data right after watering the garden
float light_intensity[(int)ARRAY_SIZE];

float average_vwc_wet;

float average_humidity;
float average_temp;
float average_vwc;
float average_light;

boolean daytime = true;


DHT dht(DHTPIN, DHTTYPE);
//WiFiEspClient client;
//char server[] = "http://ec2-18-191-185-66.us-east-2.compute.amazonaws.com";
File myFile;
String datetime;

void setup()
{
  pinMode(watervalve_pin, OUTPUT);
  digitalWrite(watervalve_pin, LOW);
  dht.begin();                    // initialize Temperature/Humidity Sensor
  
  // initialize serial for debugging
  Serial.begin(230400);
  // Wait for Serial port to open
  while (!Serial) {
  }
  
  // initialize serial for ESP Wifi module
  /*Serial1.begin(115200);
  // Wait for Serial port to open
  while (!Serial1){
  }
  // initialize ESP Wifi module
  WiFi.init(&Serial1);

  // check for the presence of the Wifi shield
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
  //client.connect(server, 80);*/

  // initialize SD Card
  int cs = 10;
  pinMode(cs, OUTPUT);
  pinMode(SS, OUTPUT);

  if(!SD.begin(cs)){
    Serial.println("SD did not initialize");
    while (1);
  }
  Serial.println("SD Initialized.");

  char myFileName[] = "weather1.csv";
  myFile = SD.open(myFileName, FILE_WRITE);
  //myFile.println("Humidity,Temperature,Moisture (VWC),Moisture Wet (VWC),Light Intensity,Datetime");
  //myFile.flush();
}

// Scale value "x" from an old range of values (in_min to in_max) to a new range of values (out_min to out_max)
float mapfloat(long x, long in_min, long in_max, long out_min, long out_max)
{
  return ((float)((x - in_min) * (out_max - out_min))) / ((float)(in_max - in_min)) + out_min;
}

void loop()
{
  // Print the network connection information every 10 seconds
  //if (client.connected()) {
    Serial.println("Start of loop");
    average_vwc_wet = -1.0;

    // Collect air temperature and relative humidity data (20 samples collected)
    for(int i = 0; i < ARRAY_SIZE; i++){
    //for(int i = 0; i < 5; i++){
      air_temp[i] = dht.readTemperature();      // Gets temperature values
      air_humidity[i] = dht.readHumidity();     // Gets humidity values
      
      // Print results on the serial monitor
      Serial.print("Temperature = ");
      Serial.print(air_temp[i]);
      Serial.print(" *C ,");
      Serial.print("    Humidity = ");
      Serial.print(air_humidity[i]);
      Serial.println(" % ");
      
      delay(2000); // Delay 2 seconds, as the DHT22 sampling rate is 0.5Hz
    }
  
    // Calculate average temperature and humidity
    int sum = 0;
    for (int i = 0; i < ARRAY_SIZE; i++){
      sum += air_humidity[i];
    }
    average_humidity = (float)sum / ARRAY_SIZE;
    Serial.println("Average Humidity: "+String(average_humidity));
    sum = 0;
    for(int i = 0; i < ARRAY_SIZE; i++){
      sum += air_temp[i];
    }
    average_temp = (float)sum / ARRAY_SIZE;
    Serial.println(String(average_temp));
    
    // Collect soil moisture data (20 samples collected)
    float reading;
    float vwc;        // Volumetric Water content
    for(int i = 0; i < ARRAY_SIZE; i++){
      reading = analogRead(soil_moisture_pin);
      reading = mapfloat(reading, 0, 1023, 0, 5);
                           
      if (reading <= 1.1){        // calculate volumetric water content from voltage using piecewise linear functions
        vwc = 10*reading - 1.0;
        if (vwc < 0.0)
          vwc = 0.0;
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
      delay(700);
    }
    
    // Calculate Average VWC
    double sum_dbl = 0.0;
    for (int i = 0; i < ARRAY_SIZE; i++){
      sum_dbl += soil_moisture[i];
    }
    average_vwc = sum_dbl/ARRAY_SIZE;
    Serial.println("Average VWC: "+String(average_vwc));
    
    if (average_vwc <= VWC_THRESHOLD){          // If soil is dry, turn the Watering Valve ON for 19 seconds
      digitalWrite(watervalve_pin, HIGH);
      delay(9500);   
      digitalWrite(watervalve_pin, LOW);    
  
      delay(75000); 
      //delay(2000);
      // get vwc (volumetric water content) readings right after watering. This data could be useful in the future
      // Wait 75 seconds to let the water drain and seep into soil
      for (int i = 0; i < ARRAY_SIZE; i++){
        reading = analogRead(soil_moisture_pin);
        reading = mapfloat(reading, 0, 1023, 0, 5);
        
        if (reading <= 1.1){        // Calculate vwc from voltage using piecewise linear functions
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
        delay(700);
      }
      // Calculate average VWC after watering
      sum_dbl = 0;
      for (int i = 0; i < ARRAY_SIZE; i++){
        sum_dbl += soil_moisture_wet[i];
      }
      
      average_vwc_wet = sum_dbl / ARRAY_SIZE;
      Serial.println("Average VWC after watering the garden: "+String(average_vwc_wet));
    }
    
    // Add timestamp to data
    /*if(String(month()).length() < 2){
      myFile.print("0"+String(month())+"-");
    }else{
      myFile.print(String(month())+"-");
    }

    if(String(day()).length() < 2){
      myFile.print("0"+String(day())+"-");
    }else{
      myFile.print(String(day())+"-");
    }
    myFile.print(String(year())+".");

    if(String(hour()).length() < 2){
      myFile.print("0"+String(hour())+":");
    }else{
      myFile.print(String(hour())+":");
    }
    if(String(minute()).length() < 2){
      myFile.print("0"+String(minute()));
    }else{
      myFile.print(String(minute()));
    }*/
    
    // Collect Light Readings. 
    // Currently, light is being represented as the photoresistor's resistance (lower resistance means brighter)
    // We Need to convert Ohms to Lux/Candela later
    /*for (int i = 0; i < sizeof(light_intensity)/2; ++i){
      float reading = analogRead(lightpin);
      reading = mapfloat(reading, 0, 1023, 0, 5);
      Serial.println("Light intensity: "+String(reading));
  
      light_intensity[i] = reading;
      delay(700);
    }
    int sum = 0;
    for (int i = 0; i < sizeof(light_intensity)/2; ++i){
      sum += light_intensity[i];
    }
    average_light = (float)sum / (float)(sizeof(light_intensity)/2);
    Serial.println("Average Light: "+average_light)*/
  
    // Collect Light Readings. 
    // Currently, light is being represented as the photoresistor's resistance (lower resistance means brighter)
    // We Need to convert Ohms to Lux/Candela later
    float resistance;
    if (daytime){
      Serial.println("Daytime activated ************ ");
      // Resistor being used for daytime mode is 200 Ohms
      for (int i = 0; i < ARRAY_SIZE; ++i){
        reading = analogRead(lightpin);
        reading = mapfloat(reading, 0, 1023, 0, 5);
  
        resistance = (200.0*4.7)/reading - 200.0;
        Serial.println("Daytime intensity: "+String(resistance));
        light_intensity[i] = resistance;
        delay(700);
      }
      sum_dbl = 0.0;
      for (int i = 0; i < ARRAY_SIZE; ++i){
        Serial.println(light_intensity[i]);
        sum_dbl += light_intensity[i];
      }
      average_light = sum_dbl / ARRAY_SIZE;
      Serial.println("Average Light: "+String(average_light));
      
      // For now, resistance of photoresistor represents brightness.
      // Later, change brightness measure to Lux/Candela
      if (average_light > 440.0){
        daytime = false;
        lightpin = A1;
      }
      
    }else{
      Serial.println("Nighttime activated ************ ");
      // Resistor being used for nighttime mode is 1000 Ohm
      for (int i = 0; i < ARRAY_SIZE; ++i){
        reading = analogRead(lightpin);
        reading = mapfloat(reading, 0, 1023, 0, 5);
  
        resistance = (1000.0*4.7)/reading - 1000.0;
        Serial.println("Nighttime intensity: "+String(resistance));
        light_intensity[i] = resistance;
        delay(700);
      }
      sum_dbl = 0.0;
      for (int i = 0; i < ARRAY_SIZE; ++i){
        Serial.println(light_intensity[i]);
        sum_dbl += light_intensity[i];
      }
      average_light = sum_dbl / ARRAY_SIZE;
      Serial.println("Average Light: "+String(average_light));
      
      if (average_light <= 400.0){
        daytime = true;
        lightpin = A3;
      }
    }
    
    // POST data to server
    /*
    client.println("POST /data/?air_temp="+String(average_temp)+"&air_humidity="+String(average_humidity)+"&soil_moisture="+String(average_vwc)+"&soil_moisture_wet="+String()+"&light_intensity="+String(average_light)+" HTTP/1.1");
    client.println("Host:  http://ec2-18-191-185-66.us-east-2.compute.amazonaws.com:80");
    */
    // Log data in a CSV file
    myFile.print(String(average_humidity)+","+String(average_temp)+","+String(average_vwc)+","+String(average_vwc_wet)+","+String(average_light)+",");
    myFile.println();
    myFile.flush();
  /*}
  else{
    // Enhancement: log to a log file
  }*/

  
  // Collect data samples once every 20 minutes (20 min * 60 seconds/min * 1000 ms/second = 1200000 ms)
  //delay(5000);
  delay(1200000);
}


// For Troubleshooting Wi-Fi Connection
/*void printWifiData()
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
}*/
