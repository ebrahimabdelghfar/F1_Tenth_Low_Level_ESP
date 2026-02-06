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
     * @param feedbackPin: GPIO pin number for feedback (analog input)
     * @return None
     */
    void Servo_Init(int pin, int feedbackPin);
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
     * @brief Get the current angle of the servo
     * @return Current servo angle in degrees
     */
    float getCurrentAngle(void); // Get the current angle of the servo in degrees

    /**
     * @brief Constrain an angle to the range [-180, 180]
     * @param x: Input angle in degrees
     * @return Constrained angle in degrees
     */
    float constrainAngle(float x);

private:
    Servo servo; // Create a Servo object for controlling the steering servo
    int analogPin; // GPIO pin for reading the current angle (if using feedback)
    float lastAngle; // Store the last angle for wrap-around handling
};

#endif /* __SERVO_H__ */