#include <Servo.h>

Servo myServo;

void setup() {
  myServo.attach(3); // Attach the servo to pin 9
  Serial.begin(115200); // Start serial communication at 9600 baud
  Serial.println("Arduino is ready."); // Confirm that Arduino is ready
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Read the input until newline
    int angle = input.toInt(); // Convert the input to an integer
    if (angle >= 0 && angle <= 180) {
      myServo.write(angle); // Set the servo angle
      Serial.print("Servo angle set to: ");
      Serial.println(angle); // Print confirmation
    } else {
      Serial.println("Invalid angle received."); // Handle invalid input
    }
  }
}
