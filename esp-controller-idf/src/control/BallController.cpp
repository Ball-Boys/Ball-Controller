#include "BallController.h"

BallController::BallController()
{
    initMagnets();
    initLUT();
}

void BallController::initLUT()
{
    float max_angle = 1.570796f; // pi/2

    for (int i = 0; i < LUT_SIZE; i++)
    {
        float angle = ((float)i / (LUT_SIZE - 1)) * max_angle;

        if (angle < 0.1f)
        {
            torque_lut[i] = 0.0f;
        }
        else
        {
            float num = 60.0f * sinf(angle);
            float den = 0.01f + 1.0f - cosf(angle);
            float val = (num / den) * expf(-2.5f * angle);
            float val_per_amp = val / 8.0f;
            torque_lut[i] = (val_per_amp > 0.0f) ? val_per_amp : 0.0f;
        }
    }
}

void BallController::initMagnets()
{
    float phi = (1.0f + sqrtf(5.0f)) / 2.0f;
    float inv_phi = 1.0f / phi;
    int idx = 0;

    float x_vals[] = {-1, 1};
    float y_vals[] = {-1, 1};
    float z_vals[] = {-1, 1};

    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            for (int k = 0; k < 2; k++)
            {
                magnets[idx++] = Vector3(x_vals[i], y_vals[j], z_vals[k]).normalized();
            }
        }
    }

    float i_vals[] = {-1, 1};
    float j_vals[] = {-1, 1};
    for (int a = 0; a < 2; a++)
    {
        for (int b = 0; b < 2; b++)
        {
            magnets[idx++] = Vector3(0, i_vals[a] * phi, j_vals[b] * inv_phi).normalized();
            magnets[idx++] = Vector3(i_vals[a] * inv_phi, 0, j_vals[b] * phi).normalized();
            magnets[idx++] = Vector3(i_vals[a] * phi, j_vals[b] * inv_phi, 0).normalized();
        }
    }
}

Matrix3 BallController::quatToMatrix(const Quaternion &q)
{
    Matrix3 mat;
    float xx = q.x * q.x;
    float yy = q.y * q.y;
    float zz = q.z * q.z;
    float xy = q.x * q.y;
    float xz = q.x * q.z;
    float yz = q.y * q.z;
    float wx = q.w * q.x;
    float wy = q.w * q.y;
    float wz = q.w * q.z;

    mat.m[0][0] = 1.0f - 2.0f * (yy + zz);
    mat.m[0][1] = 2.0f * (xy - wz);
    mat.m[0][2] = 2.0f * (xz + wy);

    mat.m[1][0] = 2.0f * (xy + wz);
    mat.m[1][1] = 1.0f - 2.0f * (xx + zz);
    mat.m[1][2] = 2.0f * (yz - wx);

    mat.m[2][0] = 2.0f * (xz - wy);
    mat.m[2][1] = 2.0f * (yz + wx);
    mat.m[2][2] = 1.0f - 2.0f * (xx + yy);
    return mat;
}

float BallController::getTorqueFactor(float angle_rad)
{
    float max_angle = 1.570796f; // pi/2

    if (angle_rad >= max_angle)
        return 0.0f;
    if (angle_rad < 0.1f)
        return 0.0f;

    // 1. Map the angle to a floating-point index (0 to 127)
    float index_float = (angle_rad / max_angle) * (LUT_SIZE - 1);

    // 2. Get the integer index below the target
    int idx = (int)index_float;

    // Safety bound
    if (idx >= LUT_SIZE - 1)
        return torque_lut[LUT_SIZE - 1];

    // 3. Linear Interpolation (mix the two closest array values)
    float fraction = index_float - idx;
    float val1 = torque_lut[idx];
    float val2 = torque_lut[idx + 1];

    return val1 * (1.0f - fraction) + val2 * fraction;
}

// ---------------------------------------------------------
// SOLVER CORE
// ---------------------------------------------------------

struct Candidate
{
    int id;
    Vector3 vec;
};

int BallController::solve(float joy_x, float joy_y, const Quaternion &q, MagnetCommand output[2])
{
    // 1. Apply Yaw Offset to Joystick Input
    float target_mag = sqrtf(joy_x * joy_x + joy_y * joy_y);
    if (target_mag < 0.001f)
        return 0; // Deadzone

    float theta_joy = atan2f(joy_y, joy_x);
    float theta_imu = theta_joy + yaw_offset; // Apply calibration

    Vector3 desired_force_world(cosf(theta_imu) * target_mag, sinf(theta_imu) * target_mag, 0.0f);

    // 2. Coordinate Transforms
    Matrix3 R = quatToMatrix(q);
    Vector3 target_force_body = R.multiplyTranspose(desired_force_world);
    Vector3 gravity_ball = R.multiplyTranspose(Vector3(0, 0, -1.0f));

    // 3. Find Candidates
    Candidate candidates[20];
    int num_candidates = 0;

    for (int i = 0; i < 20; i++)
    {
        float dot_g = magnets[i].dot(gravity_ball);
        float dot_clamp = fmaxf(-1.0f, fminf(1.0f, dot_g));
        float angle = acosf(dot_clamp);

        float strength = getTorqueFactor(angle);
        if (strength <= 0.00001f)
            continue;

        Vector3 proj_component = magnets[i] - (gravity_ball * dot_g);
        if (proj_component.norm() < 0.01f)
            continue;

        Vector3 force_dir = proj_component.normalized();
        Vector3 force_vec = force_dir * strength;

        candidates[num_candidates].id = i;
        candidates[num_candidates].vec = force_vec;
        num_candidates++;
    }

    // 4. Optimization Search
    float min_score = 1e9f; // Infinity
    int best_count = 0;

    // A. Single Magnets
    for (int i = 0; i < num_candidates; i++)
    {
        float mag_len_sq = candidates[i].vec.dot(candidates[i].vec);
        float projection = candidates[i].vec.dot(target_force_body);
        if (projection <= 0)
            continue;

        float current = projection / mag_len_sq;
        if (current > max_current)
            current = max_current;

        Vector3 produced = candidates[i].vec * current;
        float error_dist = (target_force_body - produced).norm();
        float score = error_dist + (current_penalty * current);

        if (score < min_score)
        {
            min_score = score;
            output[0].id = candidates[i].id;
            output[0].current = current;
            best_count = 1;
        }
    }

    // B. Pairs
    float t_len = target_force_body.norm();
    if (t_len > 0.001f)
    {
        Vector3 t_hat = target_force_body / t_len;
        Vector3 ref = (fabsf(t_hat.z) < 0.9f) ? Vector3(0, 0, 1) : Vector3(0, 1, 0);
        Vector3 y_hat = ref.cross(t_hat).normalized();

        for (int i = 0; i < num_candidates; i++)
        {
            for (int j = i + 1; j < num_candidates; j++)
            {
                float Ax = candidates[i].vec.dot(t_hat);
                float Ay = candidates[i].vec.dot(y_hat);
                float Bx = candidates[j].vec.dot(t_hat);
                float By = candidates[j].vec.dot(y_hat);

                float det = (Ax * By) - (Ay * Bx);
                if (fabsf(det) < 0.0001f)
                    continue;

                float Ia = (t_len * By) / det;
                float Ib = -(t_len * Ay) / det;

                if (Ia >= 0 && Ib >= 0)
                {
                    if (Ia > max_current)
                        Ia = max_current;
                    if (Ib > max_current)
                        Ib = max_current;

                    Vector3 produced = (candidates[i].vec * Ia) + (candidates[j].vec * Ib);
                    float error_dist = (target_force_body - produced).norm();
                    float score = error_dist + (current_penalty * (Ia + Ib));

                    if (score < min_score - 0.0001f)
                    {
                        min_score = score;
                        output[0].id = candidates[i].id;
                        output[0].current = Ia;
                        output[1].id = candidates[j].id;
                        output[1].current = Ib;
                        best_count = 2;
                    }
                }
            }
        }
    }

    return best_count;
}

// ---------------------------------------------------------
// CALIBRATION LOGIC
// ---------------------------------------------------------

int BallController::getCalibrationMagnet(const Quaternion &q)
{
    Matrix3 R = quatToMatrix(q);
    Vector3 gravity_ball = R.multiplyTranspose(Vector3(0, 0, -1.0f));

    int best_id = -1;
    float max_strength = -1.0f;

    for (int i = 0; i < 20; i++)
    {
        float dot_g = magnets[i].dot(gravity_ball);
        float dot_clamp = fmaxf(-1.0f, fminf(1.0f, dot_g));
        float angle = acosf(dot_clamp);

        // Find the magnet that has the absolute highest pull strength
        float strength = getTorqueFactor(angle);
        if (strength > max_strength)
        {
            max_strength = strength;
            best_id = i;
        }
    }
    return best_id;
}

void BallController::finishCalibration(int fired_magnet_id, const Quaternion &q, float joy_x, float joy_y)
{
    if (fired_magnet_id < 0 || fired_magnet_id >= 20)
        return;

    Matrix3 R = quatToMatrix(q);
    Vector3 gravity_ball = R.multiplyTranspose(Vector3(0, 0, -1.0f));

    // 1. Calculate the theoretical force vector of the fired magnet
    float dot_g = magnets[fired_magnet_id].dot(gravity_ball);
    Vector3 proj_component = magnets[fired_magnet_id] - (gravity_ball * dot_g);
    Vector3 force_dir_body = proj_component.normalized();

    // Convert to IMU World Frame
    Vector3 force_dir_world = R.multiply(force_dir_body);

    // 2. Determine IMU angle
    float theta_imu = atan2f(force_dir_world.y, force_dir_world.x);

    // 3. Determine User Joystick angle
    float theta_joy = atan2f(joy_y, joy_x);

    // 4. Store Offset
    yaw_offset = theta_imu - theta_joy;
    is_calibrated = true;
}