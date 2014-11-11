/**
 * ReadSHT1xValues
 *
 * Read temperature and humidity values from an SHTxx-series (SHT10,
 * SHT11, SHT15, SHT71) sensor.
 *
 * Copyright 2012 JAS
 * www.practicalarduino.com
 */
 
#include "SHTxx.h"
 
// Specify data and clock connections and instantiate SHTxx object
 // Sensor SHT71 power with vccPin & gndPin
 #define clockPin 8
 #define vccPin 9
 #define gndPin 10
 #define dataPin 11
 
SHTxx sht71(dataPin, clockPin);
 
void setup()
 {
// Serial.begin(57600); // Open serial connection to report values to host
 Serial.begin(9600); // Open serial connection to report values to host
 Serial.println("Starting up");
 // Power sensor
 pinMode(gndPin, OUTPUT);
 pinMode(vccPin, OUTPUT);
 digitalWrite(vccPin, HIGH);
 digitalWrite(gndPin, LOW);
 }
 
void loop()
{
 String SHT71_str = "";
 SHT71_str = SHT71();
 Serial.println(SHT71_str);
 
 delay(1000);
}
 
String SHT71()
{
 String str = "";
 char s[10];
 float temp_c;
 float humidity;
 
// Read values from the sensor
 temp_c = sht71.readTemperatureC();
 humidity = sht71.readHumidity();
// Print the values to the serial port
 str = "SHT71,";
 dtostrf(temp_c,7,3,&s[0]);
 str += s ;
 str += ",";
 dtostrf(humidity,7,3,&s[0]);
 str += s ;
 return str; 
} 
 


