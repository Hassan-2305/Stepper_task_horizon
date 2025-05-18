Stepper Motor Angle Controller using DM556 and Arduino

📌 **Introduction**
This project demonstrates how to control the angular position of a stepper motor using an Arduino and DM556 stepper driver. The user inputs a target angle (0° to 90°) via the Serial Monitor, and the motor rotates precisely to that angle. Additionally, entering 'q' resets the motor back to the 0° position.





**✨ Features**

Input angle between 0° to 90° via Serial Monitor

Precise control of stepper motor rotation using microstepping

Reset command to bring the motor back to 0°

Real-time feedback on motor movement

**⚙️ About the Stepper Motor & DM556 Driver**

Stepper Motor

Moves in discrete steps (e.g., 1.8° per step)

High accuracy, ideal for robotics and CNC machines

Controlled via step and direction signals

DM556 Stepper Driver

Digital driver supporting microstepping

Accepts STEP (PUL+) and DIRECTION (DIR+) signals

Compatible with 2-phase and 4-phase stepper motors


**🧰 Hardware Requirements**

| Component             | Quantity  |
| --------------------- | --------- |
| Arduino Due           | 1         |
| DM556 Stepper Driver  | 1         |
| Stepper Motor         | 1         |
| Power Supply (12–36V) | 1         |
| Jumper Wires          | As needed |


**💻 Software & Libraries**

Arduino IDE

Arduino.h (built-in with Arduino IDE)

No external libraries required


**🔧 Installation & Setup**

Connect STEP_PIN (PUL+) to Arduino digital pin 6

Connect DIR_PIN (DIR+) to Arduino digital pin 8

Power the DM556 via external power supply

Connect common ground between DM556 and Arduino

Upload the code through Arduino IDE

Open Serial Monitor (baud rate: 9600)


**📜 Code Explanation**

Includes & Definitions
#include <Arduino.h>
#define STEP_PIN 6
#define DIR_PIN 8
#define MICROSTEPS 15
#define STEPS_PER_REV 6400
#define TOTAL_STEPS (STEPS_PER_REV * MICROSTEPS)

STEP_PIN, DIR_PIN: Pins connected to DM556

MICROSTEPS: Microstepping multiplier

TOTAL_STEPS: Total microsteps per 360° rotation

**Function: stepMotor()**

void stepMotor(int steps, bool direction)

Sets motor direction using digitalWrite(DIR_PIN, !direction)

Sends high-low pulses to STEP_PIN to move the motor by the specified number of steps

delayMicroseconds(500) controls speed (smaller = faster)




**Function: rotateToAngle()**


void rotateToAngle(float target_angle)
Validates angle (0° to 90°)

Calculates difference:


float angle_difference = target_angle - current_angle;
int steps = abs(angle_difference) * (TOTAL_STEPS / 360.0);
Rotates motor and updates current_angle

**Function: resetToZero()**

void resetToZero()
Calls rotateToAngle(0)

Brings motor back to home position (0°)

loop() Logic

void loop()
Listens for serial input

If input is 'q', calls resetToZero()

Else, parses angle and calls rotateToAngle()


**🔣 Main Logic & Equation**



steps = (angle_difference) × (TOTAL_STEPS / 360)
Where:

angle_difference is the desired rotation delta

TOTAL_STEPS is the total microsteps per 360°


**🧪 Usage**

Upload code to Arduino

Open Serial Monitor at 9600 baud

Input target angle (e.g., 45) → press Enter

Motor rotates to specified angle

Enter 'q' to reset to 0°


**📤 Output Example**


Enter a target angle (0 to 90) or 'q' to reset to zero:
Rotating to angle: 45.00
Moved to absolute angle: 45.00
Enter a target angle (0 to 90) or 'q' to reset to zero:

**🔧 Customization**

**1. 🧭 Changing the Rotation Range**

Default: 0° to 90°

To extend, modify this line in rotateToAngle():

if (target_angle < 0 || target_angle > 90)
Change 90 to desired limit (e.g., 180 or 360)


**2. ⚙️ Adjusting Microstepping & Steps Per Revolution**

Update the following:


#define MICROSTEPS 16         // Based on DM556 DIP switches
#define STEPS_PER_REV 200     // 1.8°/step = 200 full steps/rev
Then:


#define TOTAL_STEPS (STEPS_PER_REV * MICROSTEPS)

**3. 🚦 Modifying Step Speed**

In stepMotor():


delayMicroseconds(500); // Lower = faster, Higher = slower
Balance speed and reliability to avoid missed steps.


**4. ↪️ Changing Step & Direction Pins**

Change:


#define STEP_PIN 6
#define DIR_PIN 8
Use digital output pins (avoid reserved ones on Arduino Due)

**5. 🔄 Using Degrees or Radians**

Convert to radians if needed:


float steps = abs(angle_difference) * (TOTAL_STEPS / (2 * PI));

**6. 🖥️ Changing Serial Communication Speed**

From:

Serial.begin(9600);
To:


Serial.begin(115200);
For faster communication.


**7. 🧠 Adding Position Feedback (Optional)**

Consider integrating:

Rotary encoders

Limit switches (for homing)

Hall effect sensors

Current version uses open-loop control — no feedback on actual position.












Limit switches

Hall effect sensors

Currently, it's an open-loop control system—no feedback.

