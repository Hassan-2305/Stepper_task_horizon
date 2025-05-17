#include <Arduino.h>

// Define pins for DM556 driver
#define STEP_PIN 6  // PUL+ connected to this pin
#define DIR_PIN 8   // DIR+ connected to this pin

// Motor specifications
#define MICROSTEPS 15
#define STEPS_PER_REV 6400   // Full steps per revolution (e.g., 1.8° per step motor)
#define TOTAL_STEPS (STEPS_PER_REV * MICROSTEPS)


float current_angle = 0;  // Initialize the current angle (zero position)

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  Serial.begin(9600); // Start Serial communication
  Serial.println("Enter a target angle (0 to 90) or 'q' to reset to zero:");
}

void stepMotor(int steps, bool direction) {
  // Set direction
  digitalWrite(DIR_PIN, !direction);

  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(500); // Adjust based on desired speed
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(500); // Adjust based on desired speed
  }
}

void rotateToAngle(float target_angle) {
  // Validate input range
  if (target_angle < 0 || target_angle > 90) {
    Serial.println("Invalid angle. Please enter a value between 0 and 90.");
    return;
  }

  // Calculate the angular difference relative to the zero position
  float angle_difference = target_angle - current_angle;

  // Determine the direction: clockwise for positive difference, counterclockwise for negative
  bool direction = angle_difference > 0;

  // Calculate the number of steps required
  int steps = abs(angle_difference) * ( TOTAL_STEPS/ 360.0);

  // Rotate the motor
  stepMotor(steps, direction);

  // Update the current angle
  current_angle = target_angle;

  Serial.print("Moved to absolute angle: ");
  Serial.println(current_angle);
}

void resetToZero() {
  // Rotate the motor back to zero degrees
  Serial.println("Resetting to zero position...");
  rotateToAngle(0); // Call rotateToAngle with the target angle as 0
  Serial.println("Arm reset to zero position.");
}

void loop() {
  if (Serial.available() > 0) {
    // Read the input from the Serial Monitor
    String input = Serial.readStringUntil('\n'); // Read until newline
    input.trim(); // Remove any extra spaces or newlines

    // Check if the input is 'q' or 'Q' to reset to zero
    if (input.equalsIgnoreCase("q")) {
      resetToZero();
      return;
    }

    // Convert the input to a float
    float target_angle = input.toFloat();

    // Rotate the motor to the target angle
    Serial.print("Rotating to angle: ");
    Serial.println(target_angle);
    rotateToAngle(target_angle);

    delay(1000); // Short delay before accepting next input
    Serial.println("Enter a target angle (0 to 90) or 'q' to reset to zero:");
  }
}
