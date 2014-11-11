#include <Wire.h>


void setup() {
#define HYT_ADDR 0x28 // I2C address of the HYT 221, 271, 371 and most likely the rest of the family
  Wire.begin(); // Join I2c Bus as master
  Serial1.begin(57600); // Start serial communication for serial console output
}

void loop() {
  String HYT271_str = "";
  HYT271_str = HYT271();
  Serial1.println(HYT271_str);
//  delay(1000);
}

String HYT271()
{
 String str = "";
 char s[10];
  double humidity;
  double temperature;
  
  Wire.beginTransmission(HYT_ADDR); // Begin transmission with given device on I2C bus
  Wire.requestFrom(HYT_ADDR, 4); // Request 4 bytes
  
  // Read the bytes if they are available
  // The first two bytes are humidity the last two are temperature
  
  if(Wire.available() == 4) {
    int b1 = Wire.read();
    int b2 = Wire.read();
    int b3 = Wire.read();
    int b4 = Wire.read();
    
    Wire.endTransmission(); // End transmission and release I2C bus
    
    // combine humidity bytes and calculate humidity
    
    int rawHumidity = b1 << 8 | b2;
    
    // compound bitwise to get 14 bit measurement first two bits
    // are status/stall bit (see intro text)
    
    rawHumidity = (rawHumidity &= 0x3FFF);
    humidity = 100.0 / pow(2,14) * rawHumidity;
    
    // combine temperature bytes and calculate temperature
    
    b4 = (b4 >> 2); // Mask away 2 least significant bits see HYT 221 doc
    int rawTemperature = b3 << 6 | b4;
    
    temperature = 165.0 / pow(2,14) * rawTemperature - 40;
    
    str = "HTY271,";
    dtostrf(temperature,5,2,&s[0]);
    str += s ;
    str += "," + String(rawTemperature) + ",";
    dtostrf(humidity,6,2,&s[0]);
    str += s;
    str += "," + String(rawHumidity);
    return str;
    
  }
  else {
    return "Not enough bytes available on wire.";
  }
 
}
