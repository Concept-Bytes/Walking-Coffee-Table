#include <ArduinoBLE.h>

// —— BLE service & characteristic ——  
BLEService ledService("19B10000-E8F2-537e-4f6c-d104768a1214");
BLECharacteristic switchCharacteristic(
  "19B10001-E8F2-537e-4f6c-d104768a1214",
  BLERead | BLEWrite,
  2
);

// —— Motor pins ——  
const int pwmPin1 = 11, in1Pin1 = 8, in2Pin1 = 7;
const int pwmPin2 =  9, in1Pin2 = 6, in2Pin2 = 5;

// —— Dead-zone (~5% of 127 ≃ 6) ——  
const int DEAD_ZONE = 6;

// —— Smoothing factor: fraction of the gap covered each update ——  
//    4/10 = 40%
const int SMOOTH_NUM = 4;
const int SMOOTH_DEN = 10;

// —— State for ramping ——  
int rampedLeft  = 0;
int rampedRight = 0;

void setup() {
  // motor pin-modes
  pinMode(in1Pin1, OUTPUT);
  pinMode(in2Pin1, OUTPUT);
  pinMode(pwmPin1, OUTPUT);
  pinMode(in1Pin2, OUTPUT);
  pinMode(in2Pin2, OUTPUT);
  pinMode(pwmPin2, OUTPUT);

  // Initialize BLE peripheral
  if (!BLE.begin()) {
    // If BLE fails, we'll just hang here
    while (1);
  }
  BLE.setLocalName("LED");
  BLE.setAdvertisedService(ledService);
  ledService.addCharacteristic(switchCharacteristic);
  BLE.addService(ledService);

  // Center value = 127,127
  byte initialValue[2] = {127, 127};
  switchCharacteristic.writeValue(initialValue, 2);

  BLE.advertise();
}

void loop() {
  // Wait for a central to connect
  BLEDevice central = BLE.central();
  if (!central) return;

  while (central.connected()) {
    if (!switchCharacteristic.written()) continue;

    byte buf[2];
    if (switchCharacteristic.readValue(buf, 2) != 2) continue;

    // 1) Re-center: convert 0–255 → –127…+128
    int xRaw = (int)buf[0] - 127;
    int yRaw = (int)buf[1] - 127;

    // 2) Apply dead-zone
    if (abs(xRaw) < DEAD_ZONE) xRaw = 0;
    if (abs(yRaw) < DEAD_ZONE) yRaw = 0;

    // 3) Differential-drive mix (–127…+127)
    int leftTarget  = constrain(yRaw + xRaw, -127, 127);
    int rightTarget = constrain(yRaw - xRaw, -127, 127);

    // 4) Compute delta and step for Left
    int deltaL = leftTarget - rampedLeft;
    int stepL  = deltaL * SMOOTH_NUM / SMOOTH_DEN;
    if (stepL == 0 && deltaL != 0) {
      rampedLeft = leftTarget;  // snap the final few units
    } else {
      rampedLeft += stepL;
    }

    // 5) Compute delta and step for Right
    int deltaR = rightTarget - rampedRight;
    int stepR  = deltaR * SMOOTH_NUM / SMOOTH_DEN;
    if (stepR == 0 && deltaR != 0) {
      rampedRight = rightTarget;
    } else {
      rampedRight += stepR;
    }

    // 6) Drive motors: direction + PWM (scale –127…+127 → 0…254)
    digitalWrite(in1Pin1, rampedLeft  > 0);
    digitalWrite(in2Pin1, rampedLeft  < 0);
    analogWrite(pwmPin1,  abs(rampedLeft) * 2);

    digitalWrite(in1Pin2, rampedRight > 0);
    digitalWrite(in2Pin2, rampedRight < 0);
    analogWrite(pwmPin2,  abs(rampedRight) * 2);

    // loop back immediately for the next update
  }

  // once the central disconnects, go back to advertising
  BLE.stopAdvertise();
  BLE.advertise();
}
