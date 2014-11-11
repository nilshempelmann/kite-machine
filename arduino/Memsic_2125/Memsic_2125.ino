/*
   Memsic2125
   
   Read the Memsic 2125 two-axis accelerometer.  Converts the
   pulses output by the 2125 into milli-g's (1/1000 of earth's
   gravity) and prints them over the serial connection to the
   computer.
   
   The circuit:
    * X output of accelerometer to digital pin 2
    * Y output of accelerometer to digital pin 3
    * +V of accelerometer to +5V
    * GND of accelerometer to ground
 
   http://www.arduino.cc/en/Tutorial/Memsic2125
   
   created 6 Nov 2008
   by David A. Mellis
   modified 30 Aug 2011
   by Tom Igoe
   
   This example code is in the public domain.

 */

// these constants won't change:
#define USB
#define FUNK


const int xPin = 5;     // X output of the accelerometer
const int yPin = 6;     // Y output of the accelerometer

void setup() {
  // initialize serial communications:
#ifdef USB
  Serial.begin(57600);
#endif
#ifdef FUNK
  Serial1.begin(57600);
#endif
  // initialize the pins connected to the accelerometer
  // as inputs:
  pinMode(xPin, INPUT);
  pinMode(yPin, INPUT);
}

void loop() {
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

  // print the acceleration
#ifdef USB
  Serial.print("X: ");
  Serial.print(accelerationX);
  Serial.print(" Y: ");
  Serial.print(accelerationY);
  Serial.println();
#endif
#ifdef FUNK
  Serial1.print("X: ");
  Serial1.print(accelerationX);
  Serial1.print(" Y: ");
  Serial1.print(accelerationY);
  Serial1.println();
#endif

  delay(1000);
}

