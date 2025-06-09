#include <Arduino.h>
#include <Arduino_LSM6DSOX.h>
#include <math.h>

// ─────────────────────────────────────────────────────────────────────────────
// HEADING‐HOLD WITH DRIFT COMPENSATION (GYRO BIAS + ACCEL GATING)
// ─────────────────────────────────────────────────────────────────────────────

// Motor pins (tank drive):
const int pwmPin1 = 11;
const int in1Pin1  = 8;
const int in2Pin1  = 7;

const int pwmPin2 = 9;
const int in1Pin2 = 6;
const int in2Pin2 = 5;

// P‐controller gains & thresholds:
const float Kp            = 5.5f;    // Proportional gain
const float ANGLE_TOL     = 2.0f;    // ±2° tolerance around 0° → motors OFF
const int   MIN_POWER     = 40;      // minimum PWM to overcome friction
const unsigned long HOLD_DELAY = 1500; // 1.5 seconds delay

// Drift compensation parameters:
const float GYRO_RATE_TOL = 0.3f;    // °/s minimum to consider real rotation
const float ACC_TOL       = 0.05f;   // g tolerance off 1.0g to detect movement

// Yaw integration (unwrapped + bias):
float yawAccum    = 0.0f;        // continuously accumulates gyro‐integrated yaw (°)
unsigned long prevTime = 0;      // for dt calculation
static float  gyroBias   = 0.0f; // estimated zero‐rate bias (°/s)

// State machine:
enum State {
  IDLE,       // At zero, waiting for disturbance
  WAIT,       // Detected movement, waiting 1.5 s before correcting
  CORRECT     // Actively applying P‐control to return to zero
};
State currentState = IDLE;
unsigned long movementTime = 0;  // timestamp when we first left tolerance

// ─────────────────────────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  // Motor pins (outputs):
  pinMode(in1Pin1, OUTPUT);
  pinMode(in2Pin1, OUTPUT);
  pinMode(pwmPin1, OUTPUT);
  pinMode(in1Pin2, OUTPUT);
  pinMode(in2Pin2, OUTPUT);
  pinMode(pwmPin2, OUTPUT);

  // Serial (for debug):
  Serial.begin(115200);

  // Initialize IMU:
  if (!IMU.begin()) {
    Serial.println("ERROR: IMU failed to initialize!");
    while (1);
  }
  Serial.println();
  Serial.println("IMU initialized. Heading‐hold starting at yaw=0°.");

  // Initialize time base:
  prevTime = millis();
}

// ─────────────────────────────────────────────────────────────────────────────
// MAIN LOOP
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  // A) Read sensors + compute dt
  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0f;
  if (dt <= 0.0f) return;
  prevTime = now;

  float gx, gy, gz;        // gyro (°/s)
  float ax, ay, az;        // accel (g)
  IMU.readGyroscope(gx, gy, gz);
  IMU.readAcceleration(ax, ay, az);

  // B) Detect "true" movement to gate drift
  float accMag = sqrt(ax*ax + ay*ay + az*az);
  bool linMoved = fabs(accMag - 1.0f) > ACC_TOL;
  bool rotMoved = fabs(gz - gyroBias) > GYRO_RATE_TOL;

  // C) Update gyro bias when truly still
  if (!linMoved && !rotMoved) {
    gyroBias = 0.995f * gyroBias + 0.005f * gz;
  }
  float gzCal = gz - gyroBias;

  // D) Integrate yaw only on significant rotation
  if (fabs(gzCal) > GYRO_RATE_TOL) {
    yawAccum += gzCal * dt;
  }

  // E) Compute error relative to 0°
  float error = 0.0f - yawAccum;

  // F) State machine: IDLE, WAIT, CORRECT
  switch (currentState) {
    case IDLE:
      if (fabs(error) <= ANGLE_TOL) {
        // motors OFF
        digitalWrite(in1Pin1, LOW);
        digitalWrite(in2Pin1, LOW);
        analogWrite(pwmPin1, 0);
        digitalWrite(in1Pin2, LOW);
        digitalWrite(in2Pin2, LOW);
        analogWrite(pwmPin2, 0);
        Serial.print("IDLE: yaw=");
        Serial.print(yawAccum, 2);
        Serial.println("°");
      } else {
        movementTime = now;
        currentState  = WAIT;
        Serial.print("MOVE→WAIT: yaw=");
        Serial.print(yawAccum, 2);
        Serial.println("°");
      }
      break;

    case WAIT:
      if (fabs(error) <= ANGLE_TOL) {
        currentState = IDLE;
        Serial.println("WAIT→IDLE: back in tol");
      } else if (now - movementTime >= HOLD_DELAY) {
        currentState = CORRECT;
        Serial.println("WAIT→CORRECT: delay over");
      } else {
        // motors OFF
        digitalWrite(in1Pin1, LOW);
        digitalWrite(in2Pin1, LOW);
        analogWrite(pwmPin1, 0);
        digitalWrite(in1Pin2, LOW);
        digitalWrite(in2Pin2, LOW);
        analogWrite(pwmPin2, 0);
      }
      break;

    case CORRECT:
      if (fabs(error) <= ANGLE_TOL) {
        currentState = IDLE;
        // motors OFF
        digitalWrite(in1Pin1, LOW);
        digitalWrite(in2Pin1, LOW);
        analogWrite(pwmPin1, 0);
        digitalWrite(in1Pin2, LOW);
        digitalWrite(in2Pin2, LOW);
        analogWrite(pwmPin2, 0);
        Serial.print("CORRECT→IDLE: yaw=");
        Serial.print(yawAccum, 2);
        Serial.println("°");
      } else {
        float rawP = constrain(Kp * error, -127.0f, 127.0f);
        int pwmCmd = (int)rawP;
        if (abs(pwmCmd) < MIN_POWER) pwmCmd = pwmCmd>0?MIN_POWER:-MIN_POWER;

        int leftCmd  =  pwmCmd;
        int rightCmd = -pwmCmd;

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
        analogWrite(pwmPin1, abs(leftCmd) * 2);

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

        Serial.print("CORRECT: yaw=");
        Serial.print(yawAccum, 2);
        Serial.print("°, err=");
        Serial.print(error, 2);
        Serial.print("°, pwm=");
        Serial.println(pwmCmd);
      }
      break;
  }

  delay(50);
}
