#ifndef BALL_CONTROLLER_H
#define BALL_CONTROLLER_H

#include <math.h>
#include "../core/global_state.h"

struct Quaternion
{
    float w, x, y, z;
    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}
};

struct Matrix3
{
    float m[3][3];
    Vector3 multiply(const Vector3 &v) const
    {
        return Vector3(
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z);
    }
    // Multiply by Transpose (Inverse for rotation matrices)
    Vector3 multiplyTranspose(const Vector3 &v) const
    {
        return Vector3(
            m[0][0] * v.x + m[1][0] * v.y + m[2][0] * v.z,
            m[0][1] * v.x + m[1][1] * v.y + m[2][1] * v.z,
            m[0][2] * v.x + m[1][2] * v.y + m[2][2] * v.z);
    }
};

// Return type for the solver
struct MagnetCommand
{
    int id;
    float current;
};

class BallController
{
private:
    Vector3 magnets[20];
    float max_current = 8.0f;
    float current_penalty = 2.0f;

    // Calibration State
    float yaw_offset = 0.0f;
    bool is_calibrated = false;

    static const int LUT_SIZE = 128;
    float torque_lut[LUT_SIZE];
    void initLUT();

    void initMagnets();
    Matrix3 quatToMatrix(const Quaternion &q);
    float getTorqueFactor(float angle_rad);

public:
    BallController();

    // 1. Regular Operation
    // Takes user joystick vector (x, y) and IMU quaternion. Returns number of active magnets.
    int solve(float joy_x, float joy_y, const Quaternion &q, MagnetCommand output[2]);

    // 2. Calibration Phase Methods
    // Step A: Find best magnet to fire, returns its ID.
    int getCalibrationMagnet(const Quaternion &q);

    // Step B: Call this after user pushes joystick in response to the fired magnet
    void finishCalibration(int fired_magnet_id, const Quaternion &q, float joy_x, float joy_y);

    bool isCalibrated() const { return is_calibrated; }
};

#endif