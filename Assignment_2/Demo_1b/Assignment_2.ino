const int enablePin = 4;
const int signPin1 = 7;
const int signPin2 = 8;
const int motorPin1 = 9;
const int motorPin2 = 10;
int speed = 0;

void setup() {
  pinMode(enablePin, OUTPUT);
  pinMode(signPin1, OUTPUT);
  pinMode(signPin2, OUTPUT);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  digitalWrite(enablePin, HIGH);
}

void loop() {
  //Set the direction of the motor
  digitalWrite(signPin1, HIGH);
  speed = 127;
  //Turn on the motor
  analogWrite(motorPin1, speed);
  delay(3000);
  //Turn off the motor
  speed = 0;
  analogWrite(motorPin1, speed);
  delay(3000);
}
