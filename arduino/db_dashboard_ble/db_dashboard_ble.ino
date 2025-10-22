// Define the digital pin to read from
const int inputPin1 = 12; // Choose any digital pin
const int inputPin2 = 11; // Choose any digital pin
const int inputPin3 = 10; // Choose any digital pin
const int inputPin4 = 9; // Choose any digital pin

void setup() {
  // Initialize the digital pin as an input
  pinMode(inputPin1, INPUT);
  pinMode(inputPin2, INPUT_PULLUP);
  pinMode(inputPin3, INPUT_PULLUP);
  pinMode(inputPin4, INPUT);

  // Start the serial communication to display readings
  Serial.begin(9600);
}

void loop() {
  // Read the state of the digital input pin
  int pinState1 = digitalRead(inputPin1);
  int pinState2 = digitalRead(inputPin2);
  int pinState3 = digitalRead(inputPin3);
  int pinState4 = digitalRead(inputPin4);

  // Print the state 1 to the Serial Monitor
  if (pinState1 == HIGH) {
    Serial.println("Pin 1 is HIGH (1)");
  } else {
    Serial.println("Pin 1 is LOW (0)");
  }

  // Print the state 2 to the Serial Monitor
  if (pinState2 == HIGH) {
    Serial.println("Pin 2 is HIGH (1)");
  } else {
    Serial.println("Pin 2 is LOW (0)");
  }
    // Print the state 3 to the Serial Monitor
  if (pinState3 == HIGH) {
    Serial.println("Pin 3 is HIGH (1)");
  } else {
    Serial.println("Pin 3 is LOW (0)");
  }
    // Print the state 4 to the Serial Monitor
  if (pinState4 == HIGH) {
    Serial.println("Pin 4 is HIGH (1)");
  } else {
    Serial.println("Pin 4 is LOW (0)");
  }

  Serial.println("");

  // Add a short delay for readability
  delay(500); // Adjust delay as needed
}
