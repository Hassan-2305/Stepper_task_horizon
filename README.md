# Stepper_task_horizon
Lower level code for the  Base Stepper motors to take  angles as input
Stepper Motor Angle Controller using DM556 and Arduino
📌 Introduction
This project demonstrates how to control the angular position of a stepper motor using an Arduino and DM556 stepper driver. The user inputs a target angle (0° to 90°) via the Serial Monitor, and the motor rotates precisely to that angle. Additionally, entering 'q' resets the motor back to the 0° position.

📑 Table of Contents
Features
About the Stepper Motor & DM556 Driver
Hardware Requirements
Software & Libraries
Installation & Setup
Code Explanation
Includes & Definitions
Function: stepMotor()
Function: rotateToAngle()
Function: resetToZero()
loop() Logic
Main Logic & Equation
Usage
Output Example
Customization

✨ Features
Input angle between 0° to 90° via Serial Monitor
Precise control of stepper motor rotation using microstepping
Reset command to bring the motor back to 0°
Real-time feedback on motor movement

⚙️ About the Stepper Motor & DM556 Driver
Stepper Motor
Typically moves in discrete steps (e.g., 1.8° per step)
High accuracy, suitable for robotics and CNC machines
Controlled using step and direction signals
DM556 Stepper Driver
Digital stepper driver that supports microstepping
Accepts STEP (PUL+) and DIRECTION (DIR+) signals
Works with 2-phase and 4-phase stepper motors

🧰 Hardware Requirements
Component
Quantity
Arduino Due
1
DM556 Stepper Driver
1
Stepper Motor
1
Power Supply (12–36V)
1
Jumper Wires
As needed






💻 Software & Libraries
Arduino IDE
Arduino.h (Built-in with Arduino IDE)
No external libraries are required.

🔧 Installation & Setup
Connect STEP_PIN (PUL+) to Arduino digital pin 6.
Connect DIR_PIN (DIR+) to Arduino digital pin 8.
Ensure DM556 is powered via external supply.
Ground DM556 and Arduino together.
Upload the code via Arduino IDE.
Open Serial Monitor (baud rate: 9600).

📜 Code Explanation
Includes & Definitions
#include <Arduino.h>
#define STEP_PIN 6
#define DIR_PIN 8
#define MICROSTEPS 15
#define STEPS_PER_REV 6400
#define TOTAL_STEPS (STEPS_PER_REV * MICROSTEPS)
STEP_PIN, DIR_PIN: Pins connected to DM556 for controlling step and direction.
MICROSTEPS: User-defined microstepping multiplier.
TOTAL_STEPS: Total microsteps for one full revolution (360°).

Function: stepMotor()
void stepMotor(int steps, bool direction)
Sets motor direction using digitalWrite(DIR_PIN, !direction)
Sends high-low pulses to STEP_PIN to move the motor by the specified number of steps
delayMicroseconds(500) controls speed (smaller = faster)

Function: rotateToAngle()
void rotateToAngle(float target_angle)
Validates input (only accepts angles between 0° and 90°)
Computes angle difference:
float angle_difference = target_angle - current_angle;
Converts angle to number of steps:
int steps = abs(angle_difference) * (TOTAL_STEPS / 360.0);
Rotates motor and updates current_angle

Function: resetToZero()
void resetToZero()
Calls rotateToAngle(0) to bring motor back to home position (0°)

loop() Logic
void loop()
Listens for serial input
If input is q, calls resetToZero()
Else, converts input to angle and calls rotateToAngle()

🔣 Main Logic & Equation
Key Equation
steps = (angle_difference) × (TOTAL_STEPS / 360)
Where:
angle_difference is the change in angle
TOTAL_STEPS is the total microsteps per full 360° rotation

🧪 Usage
Upload code to Arduino.
Open Serial Monitor (9600 baud).
Input a target angle (e.g., 45) and press Enter.
The motor rotates to that angle.
Enter q to reset the motor to 0°.

📤 Output Example
Enter a target angle (0 to 90) or 'q' to reset to zero:
Rotating to angle: 45.00
Moved to absolute angle: 45.00
Enter a target angle (0 to 90) or 'q' to reset to zero:

🔧 Customization
This section guides you through adapting the project to different stepper motors, drivers, rotation ranges, and control strategies.

1. 🧭 Changing the Rotation Range
Default Range: The code allows rotation between 0° and 90°.
To customize:
In rotateToAngle() function:
if (target_angle < 0 || target_angle > 90)
➤ Change 90 to your desired upper limit, e.g., 180, 360, etc.
Be sure the motor and mechanical setup support the extended motion range safely.

2. ⚙️ Adjusting Microstepping and Steps Per Revolution
DM556 driver allows various microstepping settings via DIP switches.
Update the following defines to match your motor and driver setup:
#define MICROSTEPS 15         // Change based on DM556 DIP switch (e.g., 1, 2, 4, 8, 16, etc.)
#define STEPS_PER_REV 6400    // Full-step count per revolution. For 1.8° motors, it's 200.
#define TOTAL_STEPS (STEPS_PER_REV * MICROSTEPS)
🔍 Example:
If your motor is 1.8°/step (200 full steps/rev) and DM556 is set to 16 microsteps:
#define MICROSTEPS 16
#define STEPS_PER_REV 200

3. 🚦 Modifying Step Speed
Modify delayMicroseconds(500) in stepMotor() to control the motor speed:
delayMicroseconds(500); // Lower = faster, Higher = slower
Be cautious — too fast may cause skipped steps; too slow may waste time.

4. ↪️ Changing Step and Direction Pins
#define STEP_PIN 6
#define DIR_PIN 8
Just ensure the new pins support digital output and aren’t reserved (especially on Arduino Due).

5. 🔄 Using Degrees or Radians
Current angle input/output is in degrees.
float steps = abs(angle_difference) * (TOTAL_STEPS / (2 * PI));

6. 🖥️ Changing Serial Communication Speed
You can modify the serial speed depending on your system's needs:
Serial.begin(9600);
➤ For faster communication: use 115200 or higher.

7. 🧠 Adding Position Feedback (Optional)
To track position more accurately, consider integrating:
Rotary encoders
Limit switches for homing
Hall effect sensors
This code currently assumes open-loop control — the motor moves based on command, with no feedback.


