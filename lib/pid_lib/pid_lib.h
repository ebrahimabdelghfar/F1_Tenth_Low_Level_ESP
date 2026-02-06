#ifndef INC_PID_CONTROL_H_
#define INC_PID_CONTROL_H_

#include <Arduino.h>
#include <stdint.h>
#include <math.h>

typedef struct
{
    /* Gains */
    float Kp;
    float Ki;
    float Kd;

    /* Memory Terms */
    float prevMeasurement;
    float integralAccumulator;

    /* Limits and Clamps */
    float integralLimit; // Anti-windup limit
    float outputLimit;   // Max PWM offset (e.g., 500us)
    float deadband;      // Mechanical deadband compensation

    /* Timing */
    uint32_t lastUpdateTime; // Last update timestamp in ms
    float minDt;             // Minimum dt to prevent division issues (seconds)
    float maxDt;             // Maximum dt to prevent large jumps (seconds)

} ServoPID;

class PIDController
{
public:
    /* Function Prototypes */
    /**
     * @brief Initialize the PID controller with specified parameters.
     * @param pid Pointer to ServoPID structure
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param i_lim Integral windup limit
     * @param out_lim Output limit
     */
    void PID_Init(ServoPID *pid, float kp, float ki, float kd, float i_lim, float out_lim);

    /**
     * @brief Reset the PID controller state.
     * @param pid Pointer to ServoPID structure
     * @param current_pos Current position to initialize the controller
     */
    void PID_Reset(ServoPID *pid, float current_pos);

    /**
     * @brief Compute the PID output.
     * @param pid Pointer to ServoPID structure
     * @param setpoint Desired target position
     * @param measurement Current position measurement
     * @return Computed PID output (PWM offset)
     */
    float PID_Compute(ServoPID *pid, float setpoint, float measurement);
};

#endif /* INC_PID_CONTROL_H_ */