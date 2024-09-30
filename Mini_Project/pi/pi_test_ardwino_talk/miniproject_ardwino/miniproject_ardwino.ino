#include <Wire.h>

void receiveEvent(int bytesReceived);  // Function to handle received I2C data
volatile uint8_t offset = 0;
int arr[2];
int change_flag = 0;
int count = 0;

void setup() {
  Serial.begin(9600);            // Start serial communication at 9600 baud rate
  Wire.begin(0x08);              // Join I2C bus as a slave with address 0x08 (change to your desired slave address)
  Wire.onReceive(receiveEvent);   // Register the receive event handler
}

void loop() {
  // Main loop does nothing, all the work is done in receiveEvent
  if(change_flag == 1)
  {
    Serial.println("Recived: " + String(arr[0]) + " , " + String(arr[1]));
    change_flag = 0;  
  }
  delay(100);
}

// This function is called when data is received on the I2C bus
void receiveEvent(int bytesReceived) {
  offset = Wire.read();
  while (Wire.available()) {    // Loop through all received bytes
    int x = Wire.read();          // Read each byte
    arr[count++] = x;
    if (count >= 2)
        {
          count = 0;
        }               // Print it to the serial monitor
  }
  change_flag = 1;
  Serial.println();                // New line after each event
}
