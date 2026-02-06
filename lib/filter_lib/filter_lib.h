#ifndef ANGLE_FILTER_H
#define ANGLE_FILTER_H

#include <Arduino.h>
#include <math.h>

/* ==========================================================================
 * Struct Definitions
 * ========================================================================== */

// 3-Tap Median Filter Structure
struct Median3Filter {
    float buffer[3];
    uint8_t head;
};

// 1D Kalman Filter Structure (State: Position, Velocity)
struct KalmanState {
    // State Vector
    float angle;      // x (degrees)
    float velocity;   // x[2] (degrees/sec)

    // Covariance Matrix P (2x2) - Symmetric, store upper triangle
    float P[2][2];

    // Tuning Parameters
    float Q_angle;    // Process noise: angle variance
    float Q_vel;      // Process noise: velocity variance
    float R_meas;     // Measurement noise variance

    // Timing
    float dt;           // Loop time in seconds
    float default_dt;   // Default dt if first update
    uint32_t last_tick; // Last update tick (ms)
};

/* ==========================================================================
 * Class Definition
 * ========================================================================== */

class AngleFilter {
public:
    /**
     * @brief Initialize the Hybrid Filter (Median + Kalman)
     * @param default_dt Default loop time in seconds (used for first update)
     * @param Q_ang Process noise variance for angle
     * @param Q_vel Process noise variance for velocity
     * @param R Measurement noise variance
     */
    void begin(float default_dt, float Q_ang, float Q_vel, float R);

    /**
     * @brief Update the filter with a new raw angle measurement
     * @param measurement Raw angle input (degrees)
     * @return Filtered angle (degrees)
     */
    float update(float measurement);

    /**
     * @brief Initialize the Median filter state
     */
    void initMedian();

    /**
     * @brief Apply Median filter with a new input
     * @param input New angle measurement (degrees)
     * @return Median filtered angle (degrees)
     */
    float applyMedian(float input);

    /**
     * @brief Initialize the Kalman filter parameters
     * @param default_dt Default loop time in seconds (used for first update)
     * @param Q_ang Process noise variance for angle
     * @param Q_vel Process noise variance for velocity
     * @param R Measurement noise variance
     */
    void initKalman(float default_dt, float Q_ang, float Q_vel, float R);

    /**
     * @brief Update the Kalman filter with a new measurement
     * @param measAngle New measured angle (degrees)
     * @return Updated angle estimate (degrees)
     */
    float updateKalman(float measAngle);

private:
    Median3Filter _median;
    KalmanState   _kalman;

    // Helper functions for circular math
    /**
     * @brief Constrain angle to [0, 360) degrees
     */
    static float constrainAngle360(float x);

    /**
     * @brief Calculate the shortest signed difference between two angles (degrees)
     * @return Difference a - b in range [-180, 180]
     */
    static float angleDiff(float a, float b);
};

#endif // ANGLE_FILTER_H