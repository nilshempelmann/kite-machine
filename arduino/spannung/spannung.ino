int sensorPin = A3;    // select the input pin for the potentiometer
int ledPin = 13;      // select the pin for the LED
int sensorValue = 0;  // variable to store the value coming from the sensor
float Vcc;
void setup() {
  Serial1.begin(9600);
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);  
}

void loop() {
   char s[20];
  // read the value from the sensor:
  sensorValue = analogRead(sensorPin);
  Vcc=0.01474 * sensorValue;
  delay(1000);  
  Serial1.print("Vcc=");
  Serial1.print( dtostrf(Vcc,8,2,&s[0]));
  Serial1.print(",");
  Serial1.println(sensorValue);
  
}

