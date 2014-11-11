#include <Wire.h>

#define Addr 0x1E               // 7-bit address of HMC5883 compass
#define FUNK        // Ausgabe über Funkmodem
#define USB        // Ausgabe über USB 

void setup() {
#ifdef USB
  Serial.begin(9600);
#endif

#ifdef FUNK
  Serial1.begin(9600);
#endif

  delay(100);                   // Power up delay
  Wire.begin();
  
  // Set operating mode to continuous
  Wire.beginTransmission(Addr); 
  Wire.write(byte(0x02));
  Wire.write(byte(0x00));
  Wire.endTransmission();
}

void loop() {
 String Tele_str = "";
 Tele_str = HMC5883();

#ifdef USB
 Serial.println(Tele_str);
#endif

#ifdef FUNK
 Serial1.println(Tele_str);
#endif
   
  delay(500);
}

String HMC5883()
{
 String str = "";
 char s[20];
 int x, y, z;

  // Initiate communications with compass
 Wire.beginTransmission(Addr);
 Wire.write(byte(0x03));       // Send request to X MSB register
 Wire.endTransmission();

 Wire.requestFrom(Addr, 6);    // Request 6 bytes; 2 bytes per axis
 if(Wire.available() <=6) {    // If 6 bytes available
   x = Wire.read() << 8 | Wire.read();
   z = Wire.read() << 8 | Wire.read();
   y = Wire.read() << 8 | Wire.read();
 }

 str = "HMC5883,";
 str += String(x) + "," + String(y) + "," + String(z) ;
 return str; 
} 

