void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.println("Enter data:");
  
  // Wait for input from the serial monitor
  while (Serial.available() == 0) {}

  // Read the incoming string from serial input
  String incomingString = Serial.readString();
  incomingString.trim(); // Remove any whitespace
  
  Serial.println("Received string:");

  // Iterate through each character of the string
  for (int i = 0; i < incomingString.length(); i++) {
    char c = incomingString.charAt(i);
    Serial.print(c);
    Serial.print(" - ASCII: ");
    Serial.println((int)c);
  }
}
