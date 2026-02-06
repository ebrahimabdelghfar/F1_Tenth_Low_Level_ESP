#include "filter_lib.h"

/* ==========================================================================
 * Helper Functions (Private / Static)
 * ========================================================================== */

/**
 * @brief Constrain angle to [0, 360) degrees.
 */
float AngleFilter::constrainAngle360(float x) {
    x = fmodf(x, 360.0f);
    if (x < 0.0f) x += 360.0f;
    return x;
}

/**
 * @brief Calculate the shortest signed difference between two angles (degrees).
 * Result is in range [-180, 180].
 */
float AngleFilter::angleDiff(float a, float b) {
    float diff = a - b;
    while (diff < -180.0f) diff += 360.0f;
    while (diff >  180.0f) diff -= 360.0f;
    return diff;
}

/* ==========================================================================
 * Median Filter Implementation
 * ========================================================================== */

void AngleFilter::initMedian() {
    for (int i = 0; i < 3; i++) {
        _median.buffer[i] = 0.0f;
    }
    _median.head = 0;
}

float AngleFilter::applyMedian(float input) {
    // 1. Insert into circular buffer
    _median.buffer[_median.head] = input;
    _median.head = (_median.head + 1) % 3;

    float a = _median.buffer[0];
    float b = _median.buffer[1];
    float c = _median.buffer[2];

    // 2. Circular Unwrapping Logic
    // Assume the true spread is < 180 degrees.
    // Map b and c relative to a to handle wrap-around.
    float b_unwrapped = a + angleDiff(b, a);
    float c_unwrapped = a + angleDiff(c, a);

    // 3. Sorting Network (3-input) to find median
    float md; // median value

    if (a > b_unwrapped) {
        if (b_unwrapped > c_unwrapped) {        // a > b > c
            md = b_unwrapped;
        } else if (a > c_unwrapped) {            // a > c > b
            md = c_unwrapped;
        } else {                                 // c > a > b
            md = a;
        }
    } else { // b >= a
        if (a > c_unwrapped) {                   // b > a > c
            md = a;
        } else if (b_unwrapped > c_unwrapped) {  // b > c > a
            md = c_unwrapped;
        } else {                                 // c > b > a
            md = b_unwrapped;
        }
    }

    // 4. Wrap result back to [0, 360)
    return constrainAngle360(md);
}

/* ==========================================================================
 * Kalman Filter Implementation
 * ========================================================================== */

void AngleFilter::initKalman(float default_dt, float Q_ang, float Q_vel, float R) {
    _kalman.angle      = 0.0f;
    _kalman.velocity   = 0.0f;
    _kalman.default_dt = default_dt;
    _kalman.dt         = default_dt;
    _kalman.last_tick  = 0;  // Will be set on first update
    _kalman.Q_angle    = Q_ang;
    _kalman.Q_vel      = Q_vel;
    _kalman.R_meas     = R;

    // Initialize covariance matrix
    _kalman.P[0][0] = 100.0f; _kalman.P[0][1] = 0.0f;
    _kalman.P[1][0] = 0.0f;   _kalman.P[1][1] = 100.0f;
}

float AngleFilter::updateKalman(float measAngle) {
    // --- 0. CALCULATE DT ---
    uint32_t current_tick = millis();
    float dt;
    if (_kalman.last_tick == 0) {
        // First update, use default dt
        dt = _kalman.default_dt;
    } else {
        // Calculate elapsed time in seconds
        uint32_t elapsed_ms = current_tick - _kalman.last_tick;
        dt = (float)elapsed_ms / 1000.0f;
        // Clamp dt to reasonable bounds (0.1ms to 1s)
        if (dt < 0.0001f) dt = _kalman.default_dt;
        if (dt > 1.0f)    dt = _kalman.default_dt;
    }
    _kalman.last_tick = current_tick;
    _kalman.dt = dt;  // Store for reference

    // --- 1. PREDICT STEP ---
    // State Prediction: x = F * x
    // angle = angle + velocity * dt
    float newAngle = _kalman.angle + _kalman.velocity * dt;
    newAngle = constrainAngle360(newAngle); // Keep state bounded
    float newVel = _kalman.velocity;

    // Covariance Prediction: P = F*P*F' + Q
    // F = [1 dt; 0 1]
    float P00 = _kalman.P[0][0] + dt * (_kalman.P[0][1] + _kalman.P[1][0]) + dt * dt * _kalman.P[1][1] + _kalman.Q_angle;
    float P01 = _kalman.P[0][1] + dt * _kalman.P[1][1];
    float P10 = _kalman.P[1][0] + dt * _kalman.P[1][1];
    float P11 = _kalman.P[1][1] + _kalman.Q_vel;

    // --- 2. UPDATE STEP ---
    // Innovation (Measurement Residual) with Circular Handling
    float y = angleDiff(measAngle, newAngle);

    // Innovation Covariance: S = H*P*H' + R
    // H = [1 0], so S = P00 + R
    float S = P00 + _kalman.R_meas;

    // Kalman Gain: K = P*H' * inv(S)
    float K0 = P00 / S;
    float K1 = P10 / S;

    // State Update: x = x + K*y
    _kalman.angle    = constrainAngle360(newAngle + K0 * y);
    _kalman.velocity = newVel + K1 * y;

    // Covariance Update: P = (I - K*H) * P
    float P00_new = (1.0f - K0) * P00;
    float P01_new = (1.0f - K0) * P01;
    float P10_new = P10 - K1 * P00;
    float P11_new = P11 - K1 * P01;

    _kalman.P[0][0] = P00_new;
    _kalman.P[0][1] = P01_new;
    _kalman.P[1][0] = P10_new;
    _kalman.P[1][1] = P11_new;

    return _kalman.angle;
}

/* ==========================================================================
 * Hybrid Filter Implementation
 * ========================================================================== */

void AngleFilter::begin(float default_dt, float Q_ang, float Q_vel, float R) {
    initMedian();
    initKalman(default_dt, Q_ang, Q_vel, R);
}

float AngleFilter::update(float measurement) {
    // Stage 1: Median Filter to remove spikes
    float median_out = applyMedian(measurement);

    // Stage 2: Kalman Filter for smoothing and tracking
    float kalman_out = updateKalman(median_out);

    return kalman_out;
}