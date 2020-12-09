#include <Arduino.h>
#include <SPI.h>
#include "RF24.h"
#include <Wire.h>

RF24 radio(5, 4); // CE, CSN
const byte address[6] = "10101"; //David: 11001 Henning: 10101

struct Data_Package {
  uint16_t voltage;
  uint16_t x; // Left - Right
  uint16_t y; // Forward - Backward
  uint16_t z; // Twist: Left - Right
  uint8_t button;
};

Data_Package data;

void setup() {

  pinMode(2, INPUT_PULLUP); // Joystick Button

  pinMode(7, OUTPUT); // Status LED's
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);

  //Serial.begin(115200);

  radio.begin();
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_1MBPS);
  radio.setCRCLength(RF24_CRC_8);
  radio.setAutoAck(1);
  radio.enableAckPayload();
  radio.openWritingPipe(address);
  //Serial.println("Transmitter");
  radio.stopListening();
}

void loop() {
  for(int i = 7; i < 11; i++){
    digitalWrite(i, LOW);
  }
  int voltage = analogRead(A0);
  if(voltage > 700){
    digitalWrite(7, HIGH);
  }
  if(voltage > 640){
    digitalWrite(8, HIGH);
  }
  if(voltage > 590){
    digitalWrite(9, HIGH);
  }
  if(voltage > 470){
    digitalWrite(10, HIGH);
  }

  data.x = analogRead(A1);
  data.y = analogRead(A3);
  data.z = analogRead(A2);
  data.voltage = analogRead(A0);
  data.button = digitalRead(2);
  //Serial.println(data.voltage);
  delay(5);
  radio.write(&data, sizeof(Data_Package));
  delay(20);
}
