/*
 * 3-DOF Robotic Arm - Arduino Controller
 * 
 * Upload this sketch using the Arduino IDE.
 * 
 * Hardware Connections:
 *   - Servo 1 (Base / Waist)     -> Pin 9  (PWM)
 *   - Servo 2 (Shoulder)         -> Pin 10 (PWM)
 *   - Servo 3 (Elbow)            -> Pin 11 (PWM)
 *   - External Power Supply (5V) -> Servo VCC (Red)
 *   - Common Ground (GND)        -> Arduino GND & Power Supply GND
 * 
 * Serial Protocol:
 *   Format: <angle1,angle2,angle3>  OR  angle1,angle2,angle3
 *   Values: 0 to 180 (degrees)
 *   Examples:
 *     90,90,90       -> All servos at center
 *     <45,120,60>    -> Joint 1: 45°, Joint 2: 120°, Joint 3: 60°
 */

#include <Servo.h>

// Servo pin definitions
#define PIN_SERVO1 9
#define PIN_SERVO2 10
#define PIN_SERVO3 11

Servo servo1;
Servo servo2;
Servo servo3;

// Default home angles (degrees: 0 - 180)
int current_angle1 = 90;
int current_angle2 = 90;
int current_angle3 = 90;

String inputBuffer = "";

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000); // Wait for serial port on USB boards

  // Attach servos
  servo1.attach(PIN_SERVO1);
  servo2.attach(PIN_SERVO2);
  servo3.attach(PIN_SERVO3);

  // Move to initial home position (90°, 90°, 90°)
  servo1.write(current_angle1);
  servo2.write(current_angle2);
  servo3.write(current_angle3);

  Serial.println(F("========================================"));
  Serial.println(F(" 3-DOF Robotic Arm Arduino Controller "));
  Serial.println(F(" Ready on pins: 9 (Base), 10 (Shoulder), 11 (Elbow)"));
  Serial.println(F(" Send format: <a1,a2,a3> or a1,a2,a3 (0-180)"));
  Serial.println(F("========================================"));
}

void parseAndMove(String data) {
  data.trim();
  if (data.length() == 0) return;

  // Remove optional enclosing brackets '<' and '>'
  if (data.startsWith("<") && data.endsWith(">")) {
    data = data.substring(1, data.length() - 1);
  }

  // Parse comma separated values: a1, a2, a3
  int firstComma = data.indexOf(',');
  int secondComma = data.indexOf(',', firstComma + 1);

  if (firstComma > 0 && secondComma > firstComma) {
    int a1 = data.substring(0, firstComma).toInt();
    int a2 = data.substring(firstComma + 1, secondComma).toInt();
    int a3 = data.substring(secondComma + 1).toInt();

    // Constrain angles to safe servo limit (0 - 180 degrees)
    a1 = constrain(a1, 0, 180);
    a2 = constrain(a2, 0, 180);
    a3 = constrain(a3, 0, 180);

    // Apply angles to physical servos
    servo1.write(a1);
    servo2.write(a2);
    servo3.write(a3);

    current_angle1 = a1;
    current_angle2 = a2;
    current_angle3 = a3;

    // Send acknowledgement back to Serial Monitor
    Serial.print(F("ACK: Joint1="));
    Serial.print(a1);
    Serial.print(F(" deg | Joint2="));
    Serial.print(a2);
    Serial.print(F(" deg | Joint3="));
    Serial.print(a3);
    Serial.println(F(" deg"));
  } else {
    // Single angle test (optional fallback: just joint1)
    Serial.print(F("Invalid format! Use: a1,a2,a3 (Received: "));
    Serial.print(data);
    Serial.println(F(")"));
  }
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        parseAndMove(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
      // Prevent buffer overflow
      if (inputBuffer.length() > 64) {
        inputBuffer = "";
      }
    }
  }
}
