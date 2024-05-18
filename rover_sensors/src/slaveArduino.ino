//Code for arduino nano
#include <Wire.h>

#define SLAVE_ADDRESS 0x12
#define SENSOR_PIN A0
#define BUZZER_PIN 8

const int Trigger = 6;
const int Echo = 5;

void setup() {
  Wire.begin(SLAVE_ADDRESS);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(Trigger, OUTPUT); 
  pinMode(Echo, INPUT); 
  digitalWrite(Trigger, LOW); 
}

void loop() {
}

//Function to write buzzer
void receiveEvent(int howMany) {
  if (howMany > 0) {
    int buzzerState = Wire.read();
    if (buzzerState == 1) {
      digitalWrite(BUZZER_PIN, HIGH); 
    } else {
      digitalWrite(BUZZER_PIN, LOW);
    }
  }
}

void requestEvent() {
  int sensorValue = analogRead(SENSOR_PIN);
  long t;
  long d; 

  digitalWrite(Trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trigger, LOW);
  t = pulseIn(Echo, HIGH);
  d = t / 59;

  // Array to store 4 bytes 
  byte data[4];

  data[0] = highByte(sensorValue);
  data[1] = lowByte(sensorValue);
  data[2] = highByte(d);
  data[3] = lowByte(d);

  // Send array to master(Jetson Nano)
  Wire.write(data, 4);
}
