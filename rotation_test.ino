#include <Arduino_LSM6DSOX.h>
#include <Arduino.h>

// ——— Motor pins ———  
const int pwmPin1 = 11;
const int in1Pin1  = 8;
const int in2Pin1  = 7;

const int pwmPin2 = 9;
const int in1Pin2 = 6;
const int in2Pin2 = 5;

// ——— Proportional gain and tolerances ———  
const float Kp        = 5.0f;   // tuning: how “hard” we push to turn.
const float ANGLE_TOL = 2.0f;   // ±2° tolerance
const int   MIN_POWER = 50;     // below this, motors might not overcome friction

// ——— Yaw integration globals ———  
float yaw = 0.0f;               // always holds “current integrated yaw” in degrees
unsigned long prevTime = 0;

// ——— Target angle state ———  
float targetAbsYaw = 0.0f;      // absolute yaw we want to reach (degrees)
bool  haveTarget   = false;

void setup() {
  // Configure motor pins
  pinMode(in1Pin1, OUTPUT);
  pinMode(in2Pin1, OUTPUT);
  pinMode(pwmPin1, OUTPUT);

  pinMode(in1Pin2, OUTPUT);
  pinMode(in2Pin2, OUTPUT);
  pinMode(pwmPin2, OUTPUT);

  // Serial for debugging & user input
  Serial.begin(115200);
  while (!Serial) { ; }  // wait for Serial to be ready

  // Initialize the IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  // Start the yaw timer
  prevTime = millis();

  Serial.println();
  Serial.println("→ Enter desired **relative** yaw (e.g. +20 or -45):");
  Serial.println("   Robot will rotate that many degrees from wherever it is now.");
}

void loop() {
  // —— 1) Check for new Serial input (a relative angle) ——
  if (Serial.available() > 0) {
    float angleIn = Serial.parseFloat(); 
    // consume rest of line
    while (Serial.available() && (Serial.read() != '\n')) { ; }

    // Record “where I am right now”:
    float startYaw = yaw;              // yaw was integrated up to this moment
    targetAbsYaw = startYaw + angleIn; // e.g. if start=90, angleIn=+20 → targetAbs=110

    // Normalize targetAbsYaw into [–180…+180]
    while (targetAbsYaw > 180.0f)  targetAbsYaw -= 360.0f;
    while (targetAbsYaw < -180.0f) targetAbsYaw += 360.0f;

    haveTarget = true;

    Serial.print("→ Received command: ");
    Serial.print((angleIn >= 0) ? "+" : "");
    Serial.print(angleIn, 2);
    Serial.print("°.  ");
    Serial.print("Start yaw= ");
    Serial.print(startYaw, 2);
    Serial.print("°,  Target→ ");
    Serial.print(targetAbsYaw, 2);
    Serial.println("°");
  }

  // —— 2) Integrate IMU gyro Z → yaw at a high rate ——
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0f;  // seconds since last loop
  if (dt > 0.0f) {
    prevTime = currentTime;
    float gx, gy, gz;
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(gx, gy, gz);
      // gz is already in °/s for Arduino_LSM6DSOX.h
      yaw += gz * dt;   // integrate into degrees: deg/s * s = deg

      // Wrap yaw into [–180…+180]
      if (yaw > 180.0f)       yaw -= 360.0f;
      else if (yaw < -180.0f) yaw += 360.0f;
    }
  }

  // —— 3) If we have a target, run our P‐controller on the two motors ——
  if (haveTarget) {
    // Compute the smallest‐angle error from current yaw → targetAbsYaw
    float error = targetAbsYaw - yaw;
    while (error > 180.0f)  error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    // Debug print
    Serial.print("Current yaw: ");
    Serial.print(yaw, 2);
    Serial.print("°,  Error: ");
    Serial.print(error, 2);
    Serial.println("°");

    // If we’re within tolerance, stop everything
    if (fabs(error) <= ANGLE_TOL) {
      // Brake both motors
      digitalWrite(in1Pin1, LOW);
      digitalWrite(in2Pin1, LOW);
      analogWrite(pwmPin1, 0);

      digitalWrite(in1Pin2, LOW);
      digitalWrite(in2Pin2, LOW);
      analogWrite(pwmPin2, 0);

      haveTarget = false;
      Serial.print("→ **Reached** target yaw: ");
      Serial.print(yaw, 2);
      Serial.println("°   (within tolerance)");
    }
    else {
      // Proportional control: v = Kp * error
      float controlF = Kp * error;
      controlF = constrain(controlF, -127.0f, 127.0f);
      int control = (int) controlF;

      // Guarantee a minimum command so we overcome static friction
      if (abs(control) < MIN_POWER) {
        control = (control > 0) ? MIN_POWER : -MIN_POWER;
      }

      // For a differential tank‐drive turning in place:
      //   left wheel speed = +control
      //   right wheel speed = –control
      int leftCmd  = control;
      int rightCmd = -control;

      // Motor 1 wiring: positive → forward, negative → reverse
      if (leftCmd > 0) {
        digitalWrite(in1Pin1, HIGH);
        digitalWrite(in2Pin1, LOW);
      } else if (leftCmd < 0) {
        digitalWrite(in1Pin1, LOW);
        digitalWrite(in2Pin1, HIGH);
      } else {
        digitalWrite(in1Pin1, LOW);
        digitalWrite(in2Pin1, LOW);
      }
      // Scale 0–127 → 0–254 for analogWrite (8‐bit PWM)
      analogWrite(pwmPin1, abs(leftCmd) * 2);

      // Motor 2 wiring is reversed: positive control → reverse, negative → forward
      if (rightCmd > 0) {
        digitalWrite(in1Pin2, LOW);
        digitalWrite(in2Pin2, HIGH);
      } else if (rightCmd < 0) {
        digitalWrite(in1Pin2, HIGH);
        digitalWrite(in2Pin2, LOW);
      } else {
        digitalWrite(in1Pin2, LOW);
        digitalWrite(in2Pin2, LOW);
      }
      analogWrite(pwmPin2, abs(rightCmd) * 2);
    }
  }

  // Small delay to keep serial output readable
  delay(10);
}
