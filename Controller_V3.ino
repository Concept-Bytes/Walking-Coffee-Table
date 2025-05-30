#include <ArduinoBLE.h>

// —— pin-outs ——
const int buttonPin = 2;   // your push-button (unused in this snippet, but there)
const int yAxisPin  = A0;  // Y output on analog channel 0
const int xAxisPin  = A2;   // analog channel 2 (was A2)

// —— BLE UUIDs ——
const char* serviceUuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* charUuid    = "19b10001-e8f2-537e-4f6c-d104768a1214";

// —— calibration storage ——
int rawCenterX = 0;
int rawCenterY = 0;

// 5% dead-zone of 255 ≃ 13 units
const int DEAD_ZONE = 255 * 0.05;

void calibrateJoystick() {
  long sumX = 0, sumY = 0;
  const int samples = 100;
  for (int i = 0; i < samples; i++) {
    sumX += analogRead(xAxisPin);
    sumY += analogRead(yAxisPin);
    delay(5);
  }
  rawCenterX = sumX / samples;
  rawCenterY = sumY / samples;
  Serial.print("Calibrated centre X = ");
  Serial.println(rawCenterX);
  Serial.print("Calibrated centre Y = ");
  Serial.println(rawCenterY);
}

void setup() {
  Serial.begin(9600);
  Serial.println("hELLO");
  // If your board supports it, force 12-bit ADC:
  #ifdef ARDUINO_ARCH_SAMD
    analogReadResolution(12);
  #endif

  pinMode(buttonPin, INPUT);

  Serial.println("Calibrating joystick—keep it still!");
  calibrateJoystick();

  if (!BLE.begin()) {
    Serial.println("BLE init failed");
    while (1);
  }
  BLE.setLocalName("LED");
  BLE.scanForUuid(serviceUuid);
  Serial.println("BLE scan started");
}

void controlLed(BLEDevice peripheral) {
  Serial.println("Connecting …");
  if (!peripheral.connect()) {
    Serial.println("Connect failed");
    return;
  }
  Serial.println("Connected");

  if (!peripheral.discoverAttributes()) {
    Serial.println("Discover failed");
    peripheral.disconnect();
    return;
  }

  BLECharacteristic ledChar = peripheral.characteristic(charUuid);
  if (!ledChar || !ledChar.canWrite()) {
    Serial.println("No writable characteristic");
    peripheral.disconnect();
    return;
  }

  while (peripheral.connected()) {
    // 1) read raw
    int rawX = analogRead(xAxisPin);
    int rawY = analogRead(yAxisPin);

    // 2) offset from centre
    int offX = rawX - rawCenterX;
    int offY = rawY - rawCenterY;

    // 3) compute half-ranges
    int posX = 4095 - rawCenterX;
    int negX = rawCenterX;
    int posY = 4095 - rawCenterY;
    int negY = rawCenterY;

    // 4) map into signed –255…+255
    int mappedX = (offX > 0)
      ? map(offX,  0,      posX,  0,   255)
      : map(offX, -negX,    0,   -255,  0);

    int mappedY = (offY > 0)
      ? map(offY,  0,      posY,  0,   255)
      : map(offY, -negY,    0,   -255,  0);

    // 5) flip Y so push-forward → positive
    mappedY = -mappedY;

    // 6) apply dead-zone
    if (abs(mappedX) < DEAD_ZONE) mappedX = 0;
    if (abs(mappedY) < DEAD_ZONE) mappedY = 0;

    // 7) convert signed to unsigned bytes
    byte sendX = map(mappedX, -255, 255, 0, 255);
    byte sendY = map(mappedY, -255, 255, 0, 255);

    // debug
    Serial.print("mX="); Serial.print(mappedX);
    Serial.print(" mY="); Serial.print(mappedY);
    Serial.print(" bX="); Serial.print(sendX);
    Serial.print(" bY="); Serial.println(sendY);

    // 8) send over BLE
    byte buf[2] = { sendX, sendY };
    ledChar.writeValue(buf, 2);

    delay(100);
  }

  Serial.println("Disconnected");
}

void loop() {
  BLEDevice periph = BLE.available();
  if (!periph) return;

  if (periph.localName() != String("LED")) {
    return;
  }

  BLE.stopScan();
  controlLed(periph);

  // once it drops, start scanning again
  BLE.scanForUuid(serviceUuid);
}
