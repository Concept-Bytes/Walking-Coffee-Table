#include <Arduino.h>
#include <PDM.h>
#include <math.h>

////////////////////////////////////////////////////////////////////////////////
// CONFIGURATION: Clap Detection
////////////////////////////////////////////////////////////////////////////////
static const int SAMPLE_RATE           = 16000;    // PDM sample rate (Hz)
static const int CHANNELS              = 1;        // mono
static const int BUFFER_SIZE           = 64;       // PCM samples per callback
static const int CLAP_THRESHOLD        = 3000;     // amplitude threshold
static const unsigned long DOUBLE_CLAP_WINDOW = 500; // ms for double-clap

////////////////////////////////////////////////////////////////////////////////
// CONFIGURATION: RD-03D Radar Commands
////////////////////////////////////////////////////////////////////////////////
static const uint8_t RD03D_SINGLE_CMD[12] = {
  0xFD, 0xFC, 0xFB, 0xFA,
  0x02, 0x00, 0x80, 0x00,
  0x04, 0x03, 0x02, 0x01
};

static const uint8_t RD03D_MULTI_CMD[12] = {
  0xFD, 0xFC, 0xFB, 0xFA,
  0x02, 0x00, 0x90, 0x00,
  0x04, 0x03, 0x02, 0x01
};

#define FRAME_LEN    30

struct RD03D_Target {
  int16_t x_mm;     // mm
  int16_t y_mm;     // mm
  int16_t speed;    // cm/s
  uint16_t pixel;   // mm
  float   distance; // sqrt(x² + y²)
  float   angle;    // atan2(x, y)*180/π
};

// Parse “signed 16-bit” exactly like Python logic
static int16_t parseSigned16(uint8_t highByte, uint8_t lowByte) {
  uint16_t raw = ((uint16_t)highByte << 8) | (uint16_t)(lowByte);
  if (raw & 0x8000) {
    return (int16_t)(raw & 0x7FFF);   // positive
  } else {
    return (int16_t)(- (raw & 0x7FFF)); // negative
  }
}

// Decode a single 30-byte frame into up to 3 targets
static void decodeFrame(const uint8_t *frame, RD03D_Target outTargets[3], uint8_t &outCount) {
  outCount = 0;
  if (frame[0] != 0xAA || frame[1] != 0xFF || frame[28] != 0x55 || frame[29] != 0xCC) {
    return;
  }
  for (uint8_t i = 0; i < 3; i++) {
    const uint8_t base = 4 + i*8;
    int16_t rawX   = parseSigned16(frame[base+1], frame[base+0]);
    int16_t rawY   = parseSigned16(frame[base+3], frame[base+2]);
    int16_t rawSpd = parseSigned16(frame[base+5], frame[base+4]);
    uint16_t pix   = (uint16_t)frame[base+6] | ((uint16_t)frame[base+7] << 8);

    // If slot is all zero → no valid target
    if (rawX==0 && rawY==0 && rawSpd==0 && pix==0) continue;

    RD03D_Target tgt;
    tgt.x_mm = rawX;
    tgt.y_mm = rawY;
    tgt.speed = rawSpd;
    tgt.pixel = pix;
    float fx = (float)rawX, fy = (float)rawY;
    tgt.distance = sqrt(fx*fx + fy*fy);
    tgt.angle = atan2(fx, fy) * 180.0f / PI;
    outTargets[outCount++] = tgt;
  }
}

////////////////////////////////////////////////////////////////////////////////
// PDM (Clap) Globals
////////////////////////////////////////////////////////////////////////////////
int16_t pcmBuffer[BUFFER_SIZE];
volatile bool pcmReady = false;
volatile int  pcmCount = 0;

bool wasBelowThreshold = true;
bool waitingForSecond  = false;
unsigned long firstClapTime = 0;

enum State {
  WAITING_FOR_CLAP,
  WAIT_FOR_SINGLE_FRAME
};
State currentState = WAITING_FOR_CLAP;

////////////////////////////////////////////////////////////////////////////////
// PDM (Microphone) Interrupt Handler
////////////////////////////////////////////////////////////////////////////////
void receiveHandler() {
  int bytesRead = PDM.read(pcmBuffer, sizeof(pcmBuffer));
  if (bytesRead > 0) {
    pcmCount = bytesRead / sizeof(int16_t);
    pcmReady = true;
  }
}

////////////////////////////////////////////////////////////////////////////////
// SETUP
////////////////////////////////////////////////////////////////////////////////
void setup() {
  // USB Serial for debugging
  Serial.begin(115200);
  while (!Serial) { ; }
  delay(100);
  Serial.println(F("=== Clap → Single-mode Capture → Multi ==="));

  // 1) Init PDM (microphone)
  if (!PDM.begin(CHANNELS, SAMPLE_RATE)) {
    Serial.println(F("Failed to start PDM microphone!"));
    while (1);
  }
  PDM.onReceive(receiveHandler);

  // 2) Init Radar UART (Serial1) and start in MULTI mode
  Serial1.begin(256000, SERIAL_8N1);
  delay(50);
  Serial1.write(RD03D_MULTI_CMD, sizeof(RD03D_MULTI_CMD));
  Serial1.flush();
}

////////////////////////////////////////////////////////////////////////////////
// LOOP (State Machine)
////////////////////////////////////////////////////////////////////////////////
void loop() {
  // ---------------------------------------------
  // 1) Step: WAITING_FOR_CLAP
  //    – Detect double–clap
  //    – On second clap: flush UART, send single‐mode, switch state
  // ---------------------------------------------
  if (pcmReady && currentState == WAITING_FOR_CLAP) {
    noInterrupts();
    int count = pcmCount;
    interrupts();

    // Compute average absolute amplitude
    uint32_t sumAbs = 0;
    for (int i = 0; i < count; i++) {
      sumAbs += abs(pcmBuffer[i]);
    }
    int avgAmp = sumAbs / count;

    unsigned long now = millis();
    if (waitingForSecond && (now - firstClapTime > DOUBLE_CLAP_WINDOW)) {
      waitingForSecond = false;
      firstClapTime = 0;
    }

    // Rising‐edge detection
    if (avgAmp > CLAP_THRESHOLD && wasBelowThreshold) {
      if (!waitingForSecond) {
        // First clap
        firstClapTime = now;
        waitingForSecond = true;
      } else {
        // Second clap ⇒ switch to SINGLE mode
        Serial.println(F("Double‐clap detected → switch to SINGLE mode"));
        // 1) Flush ANY bytes currently in Serial1's input buffer
        while (Serial1.available()) {
          Serial1.read();
        }
        delay(10); // small settling delay

        // 2) Send exactly one single‐mode command
        Serial1.write(RD03D_SINGLE_CMD, sizeof(RD03D_SINGLE_CMD));
        Serial1.flush();

        currentState = WAIT_FOR_SINGLE_FRAME;
        wasBelowThreshold = false;
        waitingForSecond = false;
        firstClapTime = 0;
      }
      wasBelowThreshold = false;
    }
    else if (avgAmp <= CLAP_THRESHOLD) {
      wasBelowThreshold = true;
    }

    pcmReady = false;
    return;
  }

  // ---------------------------------------------
  // 2) Step: WAIT_FOR_SINGLE_FRAME
  //    – Read UART byte-by-byte, looking for 0xAA 0xFF … 0x55 0xCC
  //    – As soon as we decode one valid 30-byte single‐target frame, print
  //      then switch back to multi
  // ---------------------------------------------
  if (currentState == WAIT_FOR_SINGLE_FRAME) {
    static uint8_t rollingBuf[FRAME_LEN];
    static int idx = 0;

    // Read any available bytes, one at a time
    while (Serial1.available()) {
      uint8_t b = Serial1.read();
      rollingBuf[idx++] = b;
      if (idx >= FRAME_LEN) {
        // Check if rollingBuf now holds a valid 30-byte frame
        if (rollingBuf[0] == 0xAA &&
            rollingBuf[1] == 0xFF &&
            rollingBuf[28] == 0x55 &&
            rollingBuf[29] == 0xCC) {
          // Decode & print:
          RD03D_Target targets[3];
          uint8_t tgtCount = 0;
          decodeFrame(rollingBuf, targets, tgtCount);

          Serial.print(F("Single‐mode capture → "));
          Serial.print(tgtCount);
          Serial.println(F(" target(s):"));
          for (uint8_t i = 0; i < tgtCount; i++) {
            Serial.print(F("  Target "));
            Serial.print(i + 1);
            Serial.print(F(": Dist="));
            Serial.print(targets[i].distance, 1);
            Serial.print(F("mm, Angle="));
            Serial.print(targets[i].angle, 1);
            Serial.print(F("°, Speed="));
            Serial.print(targets[i].speed);
            Serial.print(F("cm/s, X="));
            Serial.print(targets[i].x_mm);
            Serial.print(F("mm, Y="));
            Serial.print(targets[i].y_mm);
            Serial.println(F("mm"));
          }
          Serial.println();

          // Switch BACK to multi:
          Serial.println(F("Returning to MULTI mode"));
          Serial1.write(RD03D_MULTI_CMD, sizeof(RD03D_MULTI_CMD));
          Serial1.flush();
          // Reset roll buffer & index
          memset(rollingBuf, 0, FRAME_LEN);
          idx = 0;

          currentState = WAITING_FOR_CLAP;
          return;
        }
        // Otherwise, shift buffer left by one byte and decrement idx
        memmove(rollingBuf, rollingBuf + 1, FRAME_LEN - 1);
        idx = FRAME_LEN - 1;
      }
    }
  }
}
