#ifndef __SERVO_H__
#define __SERVO_H__

#include <Arduino.h>
#include "ESP32Servo.h"
#include "servo_config.h"
class ServoControl
{
public:
    /**
     * @brief Initialize the servo motor
     * @param pin: GPIO pin number to which the servo is connected
     * @return None
     */
    void Servo_Init(int pin);
    /**
     * @brief Set the servo angle
     * @param angle: Desired servo angle in degrees
     * @return None
     */
    void setServoAngle(float angle);
    /**
     * @brief Set the servo pulse width
     * @param effort: Control effort (PWM offset from center)
     * @return None
     */
    void setServoPulseWidth(float effort);
    /**
     * @brief Map a voltage value to a servo angle
     * @param voltage: Input voltage
     * @param min_angle: Minimum angle corresponding to minimum voltage
     * @param max_angle: Maximum angle corresponding to maximum voltage
     * @return Mapped servo angle in degrees
     */
    float map_voltage_to_angle(float voltage, float min_angle, float max_angle);
    /**
     * @brief Read and wrap angle to [-180, 180]
     * @param angle: Input angle in degrees
     * @return Wrapped angle in degrees
     */
    float readWrappedAngle(float angle);

private:
    Servo servo; // Create a Servo object for controlling the steering servo
};

#endif /* __SERVO_H__ */