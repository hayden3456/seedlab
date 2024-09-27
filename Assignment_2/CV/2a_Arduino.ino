#include <Wire.h>
#define MY_ADDR 8

volatile uint8_t offset = 0;
volatile uint8_t number = 0;
volatile uint8_t reply = 0;

void setup() {
  Serial.begin(9600); // Set baud rate
  Wire.begin(MY_ADDR);
  Wire.onReceive(receive);  // read data from Pi
  Wire.onRequest(request);  // send data to Pi
}

void loop() {
  if (number > 0) {
    printReceived();
    number = 0;
  }
}

void printReceived() {  // confirm to the user that the number is received from the Pi
  Serial.print("Number Received: ");
  Serial.println(number, DEC);
  number += 100;
  Serial.print("Number Sent: ");
  Serial.println(number, DEC);
  Serial.write(number);
}

void receive() {
  offset = Wire.read();  // first byte from the bus
  while (Wire.available()) {  // if there is data on the bus after the first byte, read it
    reply = Wire.read();  // read the data on the bus
  }
}

void request() {
  reply += 100;  // add 100 to the number sent from Pi
  Wire.write(reply);  // send the number to the Pi
  reply = 0;  // reset
}
