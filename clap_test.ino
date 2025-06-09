#include <PDM.h>

////////////////////////////////////////////////////////////////////////////////
// CONFIGURATION
////////////////////////////////////////////////////////////////////////////////
static const int SAMPLE_RATE       = 16000;   // PDM sample rate (Hz)
static const int CHANNELS          = 1;       // mono
static const int BUFFER_SIZE       = 64;      // how many PCM samples per callback
static const int CLAP_THRESHOLD    = 4000;    // amplitude threshold for "clap"
static const unsigned long DOUBLE_CLAP_WINDOW = 500; 
// time window (ms) in which two threshold‐crossings count as a double‐clap

////////////////////////////////////////////////////////////////////////////////
// GLOBALS
////////////////////////////////////////////////////////////////////////////////
// Holds decoded PCM samples (signed 16‐bit)
int16_t pcmBuffer[BUFFER_SIZE];

// Flag set by receiveHandler() when BUFFER_SIZE samples have arrived
volatile bool pcmReady = false;
volatile int  pcmCount = 0;

////////////////////////////////////////////////////////////////////////////////
// For clap detection
////////////////////////////////////////////////////////////////////////////////
bool    wasBelowThreshold = true;    // tracks if the last block was below threshold
bool    waitingForSecond  = false;   // true after first clap until window expires
unsigned long firstClapTime = 0;     // millis() when first clap was detected

////////////////////////////////////////////////////////////////////////////////
// SETUP
////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait for Serial port to open */ }

  // Initialize PDM (mono @ 16 kHz)
  if (!PDM.begin(CHANNELS, SAMPLE_RATE)) {
    Serial.println("Failed to start PDM microphone!");
    while (1);
  }

  // Tell PDM to call receiveHandler() each time BUFFER_SIZE samples are ready
  PDM.onReceive(receiveHandler);
}

////////////////////////////////////////////////////////////////////////////////
// LOOP
////////////////////////////////////////////////////////////////////////////////
void loop() {
  if (!pcmReady) {
    // no new PCM block yet
    return;
  }

  // Copy out how many samples we got
  noInterrupts();
  int count = pcmCount;   // count == BUFFER_SIZE (in ints), typically
  interrupts();

  // Compute average absolute amplitude over this block
  uint32_t sumAbs = 0;
  for (int i = 0; i < count; i++) {
    sumAbs += abs(pcmBuffer[i]);
  }
  int avgAmp = sumAbs / count;  // range roughly 0..32767

  // ——————————————————————————————————————————————
  // Clap‐detection state machine
  // ——————————————————————————————————————————————
  unsigned long now = millis();

  // 1) If we have an active first‐clap and window expired, reset
  if (waitingForSecond && (now - firstClapTime > DOUBLE_CLAP_WINDOW)) {
    waitingForSecond = false;
    firstClapTime = 0;
  }

  // 2) Detect a rising‐edge crossing of the threshold
  if (avgAmp > CLAP_THRESHOLD && wasBelowThreshold) {
    // We just saw a “clap event” (peak crossing)
    if (!waitingForSecond) {
      // This is the first clap
      firstClapTime = now;
      waitingForSecond = true;
    }
    else {
      // We were waiting for a second clap
      if (now - firstClapTime <= DOUBLE_CLAP_WINDOW) {
        // Got the second clap in time
        Serial.println("Clap detected");
        waitingForSecond = false;
        firstClapTime = 0;
      }
      // (If it somehow crossed again after window expired,
      // firstClapTime would have been reset above.)
    }
    wasBelowThreshold = false;  // stay “in clap” until amplitude falls below
  }
  else if (avgAmp <= CLAP_THRESHOLD) {
    // Below threshold → ready to detect next rising‐edge
    wasBelowThreshold = true;
  }

  // Clear flag so we wait for next PDM block
  pcmReady = false;
}

////////////////////////////////////////////////////////////////////////////////
// PDM RECEIVE HANDLER
// Called automatically whenever BUFFER_SIZE new PCM samples arrive.
// We read them into pcmBuffer[] and set pcmReady = true.
////////////////////////////////////////////////////////////////////////////////
void receiveHandler() {
  int bytesRead = PDM.read(pcmBuffer, sizeof(pcmBuffer));
  if (bytesRead > 0) {
    pcmCount = bytesRead / sizeof(int16_t);
    pcmReady = true;
  }
}
