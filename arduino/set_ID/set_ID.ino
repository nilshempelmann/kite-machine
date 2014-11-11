/*
 * EEPROM Read
 *
 * Reads the value of each byte of the EEPROM and prints it 
 * to the computer.
 * This example code is in the public domain.
 */

#include <EEPROM.h>
#define VALUE 5
// start reading from the first byte (address 0) of the EEPROM
int address = 0;
byte value;

void setup()
{
  // initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  value = EEPROM.read(address);
  
  Serial.print("Aktuelle ID auf Adresse ");
  Serial.print(address);
  Serial.print(":");
  Serial.print(value, DEC);
  Serial.println();
  
  EEPROM.write(address,VALUE);
  value = EEPROM.read(address);

  Serial.print("Change ID to: \"");
  Serial.print(address);
  Serial.print("\":");
  Serial.print(value, DEC);
  Serial.print(" ");
  Serial.print(VALUE);
  Serial.println();
}

void loop()
{
  value = EEPROM.read(address);
  
  Serial.print("Aktuelle ID auf Adresse ");
  Serial.print(address);
  Serial.print(":");
  Serial.print(value, DEC);
  Serial.println();

  delay(500);
}
