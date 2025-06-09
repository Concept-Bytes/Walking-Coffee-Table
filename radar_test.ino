/*
  RD03D_MultiTarget_Fixed.ino

  Corrected parsing for RD-03D’s “signed 16-bit” fields so that
  Y ≈ –1000 mm (when you’re ~1 m away) appears properly instead of ~–32000 mm.

  Wiring (for an ESP32-based Nano ESP):
    RD-03D        →  Nano ESP
    ──────────      ─────────────────
      5 V   (pin 1)   →   5 V
      GND   (pin 2)   →   GND
      TX    (pin 3)   →   D0 (RX1)
      RX    (pin 4)   →   D1 (TX1)

  Notes:
   • Serial (USB) = 115200 baud for debug output.
   • Serial1       = 256000 baud talking to the radar.
   • Exactly 30 bytes/frame: header 0xAA 0xFF … footer 0x55 0xCC.
   • We implement parseSigned16(...) to mirror Python’s “raw & 0x8000 ? + : –” logic.
*/

#include <Arduino.h>
#include <math.h>

// ────────────────────────────────────────────────────
// 1) Multi‐target command (12 bytes), exactly as in Python
// ────────────────────────────────────────────────────
static const uint8_t RD03D_MULTI_CMD[12] = {
  0xFD, 0xFC, 0xFB, 0xFA,
  0x02, 0x00, 0x90, 0x00,
  0x04, 0x03, 0x02, 0x01
};

// ────────────────────────────────────────────────────
// 2) Buffer settings
// ────────────────────────────────────────────────────
#define FRAME_LEN    30
#define BUF_CAPACITY 300

uint8_t  bufferData[BUF_CAPACITY];
size_t   bufferLen = 0;


// ────────────────────────────────────────────────────
// 3) RD03D “target” struct
// ────────────────────────────────────────────────────
struct RD03D_Target {
  int16_t x_mm;     // mm (custom‐signed)
  int16_t y_mm;     // mm (custom‐signed)
  int16_t speed;    // cm/s (custom‐signed)
  uint16_t pixel;   // mm
  float   distance; // sqrt(x² + y²) in mm
  float   angle;    // atan2(x, y) * 180/π
};

// ────────────────────────────────────────────────────
// 4) parseSigned16(high, low) → replicate Python logic exactly
//
//    raw = (high<<8) | low
//    if (raw & 0x8000) != 0:
//      return  +(raw & 0x7FFF)
//    else
//      return  –(raw & 0x7FFF)
// ────────────────────────────────────────────────────
static int16_t parseSigned16(uint8_t highByte, uint8_t lowByte) {
  uint16_t raw = ((uint16_t)highByte << 8) | (uint16_t)(lowByte);
  if (raw & 0x8000) {
    // high bit set → positive
    return (int16_t)(raw & 0x7FFF);
  } else {
    // high bit clear → negative
    return (int16_t)(- (raw & 0x7FFF));
  }
}


// ────────────────────────────────────────────────────
// 5) Take one 30-byte frame → extract up to 3 targets
// ────────────────────────────────────────────────────
static void decodeFrame(const uint8_t *frame, RD03D_Target outTargets[3], uint8_t &outCount) {
  outCount = 0;

  // Quick sanity check on header/footer
  if (frame[0] != 0xAA || frame[1] != 0xFF || frame[28] != 0x55 || frame[29] != 0xCC) {
    return;
  }

  // Each of the three targets occupies an 8-byte block starting at byte 4:
  //   block i: base = 4 + i*8
  for (uint8_t i = 0; i < 3; i++) {
    const uint8_t base = 4 + i * 8;

    int16_t rawX   = parseSigned16(frame[base + 1], frame[base + 0]);
    int16_t rawY   = parseSigned16(frame[base + 3], frame[base + 2]);
    int16_t rawSpd = parseSigned16(frame[base + 5], frame[base + 4]);
    uint16_t pix   = (uint16_t)frame[base + 6] | ((uint16_t)frame[base + 7] << 8);

    // If all zero, that “slot” has no valid target
    if (rawX == 0 && rawY == 0 && rawSpd == 0 && pix == 0) {
      continue;
    }

    // Fill struct
    RD03D_Target tgt;
    tgt.x_mm = rawX;
    tgt.y_mm = rawY;
    tgt.speed = rawSpd;
    tgt.pixel = pix;

    // distance = sqrt(x² + y²)
    float fx = (float)rawX;
    float fy = (float)rawY;
    tgt.distance = sqrt(fx * fx + fy * fy);

    // angle = atan2(x, y) in degrees
    // (so x>0 → “to your right,” x<0 → “to your left”)
    tgt.angle = atan2(fx, fy) * 180.0f / PI;

    outTargets[outCount++] = tgt;
  }
}


// ────────────────────────────────────────────────────
// 6) Read from Serial1 into bufferData[]; extract the _latest_ complete frame.
//    If we find one, decode it and return true.
// ────────────────────────────────────────────────────
bool updateRadar(RD03D_Target outTargets[3], uint8_t &outCount) {
  // 1) Pull all available bytes from Serial1 into our ring buffer
  while (Serial1.available()) {
    if (bufferLen < BUF_CAPACITY) {
      bufferData[bufferLen++] = (uint8_t)Serial1.read();
    } else {
      // In the unlikely event of overflow, shift old data off
      memmove(bufferData, bufferData + FRAME_LEN, bufferLen - FRAME_LEN);
      bufferLen -= FRAME_LEN;
      bufferData[bufferLen++] = (uint8_t)Serial1.read();
    }
  }

  // 2) Search for ANY complete 30-byte frames. Save the last one's start index.
  int lastFrameStart = -1;
  for (int i = 0; i + FRAME_LEN <= (int)bufferLen; i++) {
    if (bufferData[i] == 0xAA &&
        bufferData[i + 1] == 0xFF &&
        bufferData[i + 28] == 0x55 &&
        bufferData[i + 29] == 0xCC) {
      lastFrameStart = i;
    }
  }

  // 3) If we found at least one, copy the _last_ complete frame, decode it,
  //    then discard everything up through that frame's end.
  if (lastFrameStart >= 0) {
    uint8_t frame[FRAME_LEN];
    memcpy(frame, bufferData + lastFrameStart, FRAME_LEN);

    // Remove everything up to (lastFrameStart + 30) from bufferData
    int bytesAfterFrame = bufferLen - (lastFrameStart + FRAME_LEN);
    memmove(bufferData,
            bufferData + (lastFrameStart + FRAME_LEN),
            bytesAfterFrame);
    bufferLen = bytesAfterFrame;

    // Decode into outTargets[]
    decodeFrame(frame, outTargets, outCount);
    return (outCount > 0);
  }

  // No complete frame found
  return false;
}


void setup() {
  // 1) USB Serial for debugging
  Serial.begin(115200);
  while (!Serial) { ; }
  delay(200);
  Serial.println(F("--- RD-03D Multi-Target (Fixed) ---"));

  // 2) Serial1 = 256000 baud talking to the RD-03D
  Serial1.begin(256000, SERIAL_8N1);
  delay(200);

  // 3) Send the “enter multi-target mode” command once:
  Serial.println(F("→ Sending MULTI-TARGET command to RD-03D …"));
  Serial1.write(RD03D_MULTI_CMD, sizeof(RD03D_MULTI_CMD));
  Serial1.flush();
  delay(200);
  Serial1.flush();
  Serial.println(F("✓ RD-03D should now be in MULTI-TARGET mode."));
}


void loop() {
  static RD03D_Target targets[3];
  static uint8_t       tgtCount = 0;

  // Check if a new, valid multi-target frame just arrived
  if (updateRadar(targets, tgtCount)) {
    Serial.print(F("Detected "));
    Serial.print(tgtCount);
    Serial.println(F(" target(s):"));

    for (uint8_t i = 0; i < tgtCount; i++) {
      Serial.print(F("  Target "));
      Serial.print(i + 1);
      Serial.print(F(": Dist = "));
      Serial.print(targets[i].distance, 1);
      Serial.print(F(" mm, Angle = "));
      Serial.print(targets[i].angle, 1);
      Serial.print(F("°, Speed = "));
      Serial.print(targets[i].speed);
      Serial.print(F(" cm/s, X="));
      Serial.print(targets[i].x_mm);
      Serial.print(F(" mm, Y="));
      Serial.print(targets[i].y_mm);
      Serial.println(F(" mm"));
    }
    Serial.println();
  }

  // Poll at ~10 Hz
  delay(100);
}
