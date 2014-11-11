//=======================================================================================================
/*
Arduino Micro
Pin 01 	MOSI            SD-Card
Pin 02 	SS		SD-Card
Pin 03 	TX		Modem RX
Pin 04	RX		Modem TX
Pin 05	RESET
Pin 06 	GND
Pin 07	 2 / SDA	I2C
Pin 08	 3(PWM) / SCL	I2C
Pin 09 	 4 / A6
Pin 10	 5(PWM)		Mx2125	x
Pin 11	 6(PWM) / A7	Mx2125	y
Pin 12	 7
Pin 13 	 8 / A8
Pin 14	 9(PWM) / A9
Pin 15	10(PWM) / A10
Pin 16 	11(PWM)
Pin 17	12 / A11

Pin 18	13 (PWM)
Pin 19 	3.3V
Pin 20	AREF
Pin 21	A0		Wind ref volt
Pin 22	A1
Pin 23 	A2
Pin 24	A3		Vin
Pin 25	A4
Pin 26 	A5		Wind Sensor Signal
Pin 27	---
Pin 28	---
Pin 29 	+5V
Pin 30	RESET
Pin 31	GND
Pin 32	Vin		+9V
Pin 33 	MISO		SD-Card
Pin 34	SCK		SD-Card

*/
//=======================================================================================================

#define USE_HYT271     // Temp/Feuchte
#define USE_BMP085     // Druck
#define USE_L3G4200D     //  Triple-Axis Gyro GY-50
#define USE_HMC5883      // Compass
#define USE_VIN        // Spannung 
#define USE_2125       // Beschleunigung
//#define USE_SD_CARD       // SD memory card
#define USE_CHECK_MEM       // Fügt den noch freien Speicher in das Telegramm
//#define USE_WINDSPEED       // Fügt Windgeschwindigkeit ins Telegramm ein.
//#define USE_UNO       // USE_UNO muss für Arduino Uno definiert sein.
#define USE_USB       // USE_UNO muss für Arduino Uno definiert sein.
#define USE_FUNK       // USE_UNO muss für Arduino Uno definiert sein.

#define ADD_UNIT_ID 0       // Adresse wo im EEPROM die UINT-ID des arduion steht.
#define VALUE 3

#include <Wire.h>
#include <TimerOne.h>  
#ifdef USE_CHECK_MEM0
  #include <MemoryFree.h>
#endif
#include <EEPROM.h>
#include <wiring_analog.h>
#include <SPI.h>
#include <SD.h>
const int chipSelect = 10;

#ifdef USE_SD_CARD
  File myFile;
  char F_Name[13];
  int F_Num = 1;
#endif

unsigned long time;
unsigned long myTime=1;
unsigned long myTime_a;
unsigned long myT = 0;
int UnitID=-1;
int address = 0;
byte value;
// BMP085 settig

#ifdef USE_BMP085
 #define BMP085_ADDRESS  0x77  // I2C address of BMP085

 const unsigned char OSS = 0;  // Oversampling Setting
// Calibration values
 int ac1;
 int ac2;
 int ac3;
 unsigned int ac4;
 unsigned int ac5;
 unsigned int ac6;
 int b1;
 int b2;
 int mb;
 int mc;
 int md;

// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
  long b5; 

//d  static String tele_BMP085 = "NA";
//d  unsigned long t_BMP085=0;
#endif

#ifdef USE_HMC5883
 #define HMC5883_ADDRESS 0x1E  // 7-bit address of HMC5883 compass
#endif


// HYT271 setting
#ifdef USE_HYT271
  #define HYT_ADDR 0x28 // I2C address of the HYT 221, 271, 371 and most likely the rest of the family
#endif

// L3G4200D setting 

#ifdef USE_L3G4200D
 #define CTRL_REG1 0x20
 #define CTRL_REG2 0x21
 #define CTRL_REG3 0x22
 #define CTRL_REG4 0x23
 #define CTRL_REG5 0x24

 int L3G4200D_Address = 105; //I2C address of the L3G4200D
 
 int A3_x;
 int A3_y;
 int A3_z;

#endif

#ifdef USE_VIN
//  analogReference(INTERNAL);
  #ifdef USE_UNO
    int sensorPin = A0;    // select the input pin for the potentiometer
  #else
    int sensorPin = A3;    // select the input pin for the potentiometer
  #endif
  int sensorValue = 0;  // variable to store the value coming from the sensor
#endif

#ifdef USE_2125
  #ifdef USE_UNO
    const int xPin = 2;     // X output of the accelerometer
    const int yPin = 3;     // Y output of the accelerometer
  #else
    const int xPin = 5;     // X output of the accelerometer
    const int yPin = 6;     // Y output of the accelerometer
  #endif
#endif

//========================================================================
//========================================================================
//========================================================================
//
//  setup
// 
//========================================================================
//========================================================================
//========================================================================


void setup(){
  EEPROM.write(ADD_UNIT_ID,VALUE);
#ifdef USB
  Serial.begin(57600);
#endif
#ifdef FUNK
  Serial1.begin(57600);
#endif
  delay(100);                   // Power up delay
  Wire.begin();
  
  Timer1.initialize(10000);
  Timer1.attachInterrupt(Time_Step);
  UnitID = EEPROM.read(ADD_UNIT_ID);
  
#ifdef USE_BMP085
  bmp085Calibration();
#endif
  
#ifdef USE_HMC5883
  
  // Set operating mode to continuous
  Wire.beginTransmission(HMC5883_ADDRESS); 
  Wire.write(byte(0x02));
  Wire.write(byte(0x00));
  Wire.endTransmission();
#endif 

#ifdef USE_L3G4200D
  setupL3G4200D(2000); // Configure L3G4200  - 250, 500 or 2000 deg/sec
#endif
   
#ifdef USE_2125
  pinMode(xPin, INPUT);
  pinMode(yPin, INPUT);
#endif
#ifdef USE_SD_CARD

  if (!SD.begin(10)) {
#ifdef USB
    Serial.println("# initialization failed!");
#endif
#ifdef FUNK
    Serial1.println("# initialization failed!");
#endif
    return;
  }
#ifdef USB
  Serial1.println("# initialization of SD-Card done.");
#endif
#ifdef FUNK
  Serial1.println("# initialization of SD-Card done.");
#endif
  sprintf (F_Name, "%08i.txt", F_Num);
  while (SD.exists(F_Name)){
    sprintf (F_Name, "%08i.csv", ++F_Num);
  }
#ifdef USB
  Serial.print("Use File:");
  Serial.println(F_Name);
#endif
#ifdef FUNK
  Serial1.print("Use File:");
  Serial1.println(F_Name);
#endif
  delay (1000);
#endif
}

//========================================================================
//========================================================================
//========================================================================
//
//  LOOP
// 
//========================================================================
//========================================================================
//========================================================================

void loop()
{
  unsigned long l_time;
  String telegram = "";
  
  time = millis();
//  telegram += String(time) ;
  myTime_a = myTime;
  
  if ((myTime % 10) == 0){
    l_time = millis();
    telegram += String(l_time) ;


#ifdef USE_BMP085
    telegram += BMP085();
#else
    telegram += ",BMP,NA,NA";
#endif

#ifdef USE_HYT271
    telegram += HYT271();
#else
    telegram += ",HYT271,NA,NA";
#endif

#ifdef USE_WINDSPEED
    telegram += ",WS," +   String("Na");
#else
    telegram += ",WS,NA";
#endif

#ifdef USE_L3G4200D
    telegram += L3G4200();
#else
    telegram += ",GY50,NA,NA,NA";
#endif

#ifdef USE_HMC5883
    telegram += HMC5883();
#else
    telegram += ",HMC,NA,NA,NA";
#endif

#ifdef USE_2125
    telegram += Memsic2125();
#else
    telegram += ",M2125,NA,NA";
#endif

#ifdef USE_VIN
    telegram += VIN();
#endif

    telegram += ",MC," +  String(UnitID);
#ifdef USE_CHECK_MEM
//    telegram += "," + String(freeMemory());
#endif
  }  
    if (telegram.length() != 0)
    {
      myT++;
#ifdef USE_SD_CARD
      File dataFile = SD.open(F_Name, FILE_WRITE);
      if (dataFile) {
        dataFile.println(telegram);
        dataFile.close();
      }  
      else {
#ifdef USB
        Serial.print("# error opening File:");
        Serial.println(F_Name);
#endif
#ifdef FUNK
        Serial1.print("# error opening File:");
        Serial1.println(F_Name);
#endif
      }
#endif    

#ifdef USE_SD_CARD
    if (myT == 30){
#else
    if (myT == 5){
#endif    

#ifdef USB
      Serial.println(telegram);
#endif
#ifdef FUNK
      Serial1.println(telegram);
#endif
      myT = 0;
    }
  }
  
}

//========================================================================
//========================================================================
//========================================================================
//
//  Timer Interruop LOOP
// 
//========================================================================
//========================================================================
//========================================================================

void Time_Step() {
  myTime++;
}

#ifdef USE_VIN
long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

String VIN()
{
  float Vin;
  double Vcc; 
  String str = ",Vin,";
  char s[20];
  
  Vcc = readVcc()/1000.0;
  sensorValue = analogRead(sensorPin);
//  Vin=(sensorValue / 1023.0) * Vcc * 3.0;
  Vin=sensorValue/1.0;
  
  dtostrf(Vin,0,1,&s[0]);
  str += s;
  return str;
}
#endif

/*
     Memsic2125
*/
#ifdef USE_2125
String Memsic2125()
{
  String str = ",M2125,";
  
  // variables to read the pulse widths:
  int pulseX, pulseY;
  // variables to contain the resulting accelerations
  int accelerationX, accelerationY;
 
  // read pulse from x- and y-axes:
  pulseX = pulseIn(xPin,HIGH);  
  pulseY = pulseIn(yPin,HIGH);
 
  // convert the pulse width into acceleration
  // accelerationX and accelerationY are in milli-g's:
  // earth's gravity is 1000 milli-g's, or 1g.
  accelerationX = ((pulseX / 10) - 500) * 8;
  accelerationY = ((pulseY / 10) - 500) * 8;

  str += String(accelerationX) + "," + String(accelerationY);
  
  return str;
}
#endif
/*
  Auslesen des Drucksensor
*/
#ifdef USE_BMP085
String BMP085()
{
 String str = ",BMP,";
 char s[20];

 float temperature = bmp085GetTemperature(bmp085ReadUT()); //MUST be called first
 float pressure = bmp085GetPressure(bmp085ReadUP());

 dtostrf(temperature,0,1,&s[0]);
 str += s ;
 str += ",";
 dtostrf(pressure/100.0,0,2,&s[0]);
 str += s ;
 return str;
}
#endif

/*
  Auslesen des Temperatur und Feuchte Sensor HYT271
*/
#ifdef USE_HYT271

String HYT271()
{
 String str = ",HYT271,";
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
    
    dtostrf(temperature,0,2,&s[0]);
    str += s ;
    str += ",";
    dtostrf(humidity,0,2,&s[0]);
    str += s ;
    return str;
    
  }
  else {
    return "Not enough bytes available on wire.";
  }
 
}
#endif

/*
  Auslesen des Compass
*/
#ifdef USE_HMC5883
String HMC5883()
{
 String str = ",HMC,";
//del char s[20];
 int x, y, z;

  // Initiate communications with compass
 Wire.beginTransmission(HMC5883_ADDRESS);
 Wire.write(byte(0x03));       // Send request to X MSB register
 Wire.endTransmission();

 Wire.requestFrom(HMC5883_ADDRESS, 6);    // Request 6 bytes; 2 bytes per axis
 if(Wire.available() <=6) {    // If 6 bytes available
   x = Wire.read() << 8 | Wire.read();
   z = Wire.read() << 8 | Wire.read();
   y = Wire.read() << 8 | Wire.read();
 }

 str += String(x) + "," + String(y) + "," + String(z) ;
 return str; 
} 
#endif
/*
  Auslesen des Triple-Axis Gyro
*/

#ifdef USE_L3G4200D
String L3G4200()
{
 String str = ",GY50,";
//del  char s[20];

  getGyroValues();  // This will update x, y, and z with new values
  str += String(A3_x) + "," + String(A3_y) + "," + String(A3_z) ;
  return str; 
}
#endif

// Stores all of the bmp085's calibration values into global variables
// Calibration values are required to calculate temp and pressure
// This function should be called at the beginning of the program
#ifdef USE_BMP085
void bmp085Calibration()
{
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
}

// Calculate temperature in deg C
float bmp085GetTemperature(unsigned int ut){
  long x1, x2;

  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  float temp = ((b5 + 8)>>4);
  temp = temp /10;

  return temp;
}

// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
long bmp085GetPressure(unsigned long up){
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;

  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;

  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;

  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;

  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;

  long temp = p;
  return temp;
}

// Read 1 byte from the BMP085 at 'address'
char bmp085Read(unsigned char address)
{
  unsigned char data;

  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(BMP085_ADDRESS, 1);
  while(!Wire.available())
    ;

  return Wire.read();
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char address)
{
  unsigned char msb, lsb;

  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2)
    ;
  msb = Wire.read();
  lsb = Wire.read();

  return (int) msb<<8 | lsb;
}

// Read the uncompensated temperature value
unsigned int bmp085ReadUT(){
  unsigned int ut;

  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();

  // Wait at least 4.5ms
  delay(5);

  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}

// Read the uncompensated pressure value
unsigned long bmp085ReadUP(){

  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;

  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();

  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));

  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  msb = bmp085Read(0xF6);
  lsb = bmp085Read(0xF7);
  xlsb = bmp085Read(0xF8);

  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);

  return up;
}
#endif

void writeRegister(int deviceAddress, byte address, byte val) {
  Wire.beginTransmission(deviceAddress); // start transmission to device 
  Wire.write(address);       // send register address
  Wire.write(val);         // send value to write
  Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address){

  int v;
  Wire.beginTransmission(deviceAddress);
  Wire.write(address); // register to read
  Wire.endTransmission();

  Wire.requestFrom(deviceAddress, 1); // read a byte

  while(!Wire.available()) {
    // waiting
  }

  v = Wire.read();
  return v;
}

/*
L3G4200D Routines
*/
#ifdef USE_L3G4200D
void getGyroValues(){

  byte xMSB = readRegister(L3G4200D_Address, 0x29);
  byte xLSB = readRegister(L3G4200D_Address, 0x28);
  A3_x = ((xMSB << 8) | xLSB);

  byte yMSB = readRegister(L3G4200D_Address, 0x2B);
  byte yLSB = readRegister(L3G4200D_Address, 0x2A);
  A3_y = ((yMSB << 8) | yLSB);

  byte zMSB = readRegister(L3G4200D_Address, 0x2D);
  byte zLSB = readRegister(L3G4200D_Address, 0x2C);
  A3_z = ((zMSB << 8) | zLSB);
}

int setupL3G4200D(int scale){
  //From  Jim Lindblom of Sparkfun's code

  // Enable x, y, z and turn off power down:
  writeRegister(L3G4200D_Address, CTRL_REG1, 0b00001111);

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000000);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);

  // CTRL_REG4 controls the full-scale range, among other things:

  if(scale == 250){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00000000);
  }else if(scale == 500){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00010000);
  }else{
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00110000);
  }

  // CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
  writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);
}
#endif


