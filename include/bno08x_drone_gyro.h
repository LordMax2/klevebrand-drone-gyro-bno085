#pragma once

#include "Arduino.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO08x.h>
#include "yaw_pitch_roll.h"
#include "concept_drone_gyro.h"

class Bno08xDroneGyro
{
public:
    explicit Bno08xDroneGyro(const int reset_pin) : _gyro(reset_pin) {}

    void setup();
    bool reload();
    void reset();
    float yaw();
    float pitch();
    float roll();
    float accelerationX();
    float accelerationY();
    float accelerationZ();
    void printYawPitchRoll();
    void printYawPitchRollAndAcceleration();
    bool setModeAcro();
    bool setModeEuler();
    bool setModeEulerAndAcceleration();
    unsigned long timestampMilliseconds();

private:
    Adafruit_BNO08x _gyro;
    sh2_SensorValue_t _sensor_value;
    YawPitchRoll_t _yaw_pitch_roll;
    sh2_Accelerometer_t _acceleration = {0.0f, 0.0f, 0.0f};
    sh2_RotationVectorWAcc_t _rotational_vector = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    sh2_Accelerometer_t _rotated_acceleration = {0.0f, 0.0f, 0.0f};
    enum Mode
    {
        MODE_NONE,
        MODE_ACRO,
        MODE_EULER,
        MODE_EULER_AND_ACCELERATION
    };
    Mode _mode = MODE_NONE;

    static YawPitchRoll_t quaternionsToYawPitchRoll(const sh2_RotationVectorWAcc_t *rotational_vector, bool degrees = false);
    static YawPitchRoll_t quaternionsToYawPitchRoll(float qr, float qi, float qj, float qk, bool degrees = false);
    static void rotateVectorByQuaternion(float& x, float& y, float& z, float r, float i, float j, float k);
};

static_assert(DroneGyroConcept<Bno08xDroneGyro>, "Bno08xDroneGyro does not meet DroneGyroConcept");
