#include "pid_lib.h"

// Helper function to normalize angle to [-180, 180]
static float constrainAngle(float x) {
    x = fmodf(x + 180.0f, 360.0f);
    if (x < 0) x += 360.0f;
    return x - 180.0f;
}

void PIDController::PID_Init(ServoPID *pid, float kp, float ki, float kd, float i_lim, float out_lim) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integralLimit = i_lim;
    pid->outputLimit = out_lim;
    pid->integralAccumulator = 0.0f;
    pid->prevMeasurement = 0.0f;
    pid->deadband = 20.0f;
    pid->lastUpdateTime = millis();
    pid->minDt = 0.001f;   // 1ms minimum
    pid->maxDt = 0.1f;     // 100ms maximum
}

void PIDController::PID_Reset(ServoPID *pid, float current_pos) {
    pid->integralAccumulator = 0.0f;
    pid->prevMeasurement = current_pos;
    pid->lastUpdateTime = millis();
}

float PIDController::PID_Compute(ServoPID *pid, float setpoint, float measurement) {
    // 0. Calculate adaptive dt from elapsed time
    uint32_t now = millis();
    float dt = (float)(now - pid->lastUpdateTime) / 1000.0f;

    if (dt < pid->minDt) dt = pid->minDt;
    if (dt > pid->maxDt) dt = pid->maxDt;

    pid->lastUpdateTime = now;

    // 1. Error with wrap-around
    float error = setpoint - measurement;
    error = constrainAngle(error);

    // 2. Proportional
    float P = pid->Kp * error;

    // 3. Integral with anti-windup
    pid->integralAccumulator += error * dt;

    if (pid->integralAccumulator > pid->integralLimit)
        pid->integralAccumulator = pid->integralLimit;
    if (pid->integralAccumulator < -pid->integralLimit)
        pid->integralAccumulator = -pid->integralLimit;

    float I = pid->Ki * pid->integralAccumulator;

    // 4. Derivative on measurement (kick avoidance)
    float deltaMeasurement = measurement - pid->prevMeasurement;
    deltaMeasurement = constrainAngle(deltaMeasurement);

    float D = -pid->Kd * (deltaMeasurement / dt);

    // 5. Total output
    float output = P + I + D;

    // 6. Deadband compensation
    if (output > 0.1f) output += pid->deadband;
    if (output < -0.1f) output -= pid->deadband;

    // 7. Output saturation
    if (output > pid->outputLimit) output = pid->outputLimit;
    if (output < -pid->outputLimit) output = -pid->outputLimit;

    // 8. Store state
    pid->prevMeasurement = measurement;

    return output;
}
