#include "Arduino.h"
#include "bno08x_drone_gyro.h"

Bno08xDroneGyro gyro(10);

void setup() {
    Serial.begin(115200);

    gyro.setup();

    Serial.println("START");

    gyro.setModeEulerAndAcceleration();
}

void loop() {
    gyro.reload();

    gyro.printYawPitchRollAndAcceleration();

    delay(4);
}
