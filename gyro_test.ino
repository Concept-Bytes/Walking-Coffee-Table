#include <Arduino_LSM6DSOX.h>

float Ax, Ay, Az;    // Accelerometer readings (g)
float Gx, Gy, Gz;    // Gyroscope readings (°/s)

float roll  = 0.0;   // Rotation around X-axis (°)
float pitch = 0.0;   // Rotation around Y-axis (°)
float yaw   = 0.0;   // Integrated rotation around Z-axis (°)

unsigned long prevTime = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait for Serial to open */ }

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");

  Serial.print("Gyroscope sample rate     = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");

  prevTime = millis();
  Serial.println("\nPitch, Roll, Yaw (°):\n");
}

void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;  // seconds since last update

  // Only update/print every 100 ms
  if (dt >= 0.1) {
    prevTime = currentTime;

    // 1) Read gyro and integrate Z axis for yaw
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(Gx, Gy, Gz);
      // Gz is in degrees/second around Z-axis
      yaw += Gz * dt;
    }

    // 2) Read accelerometer and compute roll & pitch
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(Ax, Ay, Az);

      // Roll  = rotation about X-axis: atan2(Ay, Az)
      roll = atan2(Ay, Az) * 180.0 / PI;

      // Pitch = rotation about Y-axis: atan2(–Ax, sqrt(Ay² + Az²))
      pitch = atan2(-Ax, sqrt(Ay * Ay + Az * Az)) * 180.0 / PI;
    }

    // 3) Print all three
    // Serial.print("Pitch = ");
    // Serial.print(pitch, 2);
    // Serial.print("°\tRoll = ");
    // Serial.print(roll, 2);
    Serial.print("°\tYaw = ");
    Serial.print(yaw, 2);
    Serial.println("°");
  }
}
