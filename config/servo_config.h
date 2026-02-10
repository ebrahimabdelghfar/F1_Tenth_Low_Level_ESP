#ifndef __SERVO_CONFIG_H__
#define __SERVO_CONFIG_H__

#define SERVO_MIN_ANGLE -180.0f        // Minimum angle in degrees
#define SERVO_MAX_VOLTAGE 1.65f        // Maximum voltage in volts
#define ESP32_VOLTAGE 3.3f             // Minimum voltage in volts
#define ANGLE_OFFSET 11.0f              // Angle offset in degrees
#define SERVO_MAX_ANGLE 180.0f         // Maximum angle in degrees
#define SERVO_PWM_FREQUENCY 50         // PWM frequency in Hz
#define SERVO_KP 10.0f                 // Proportional gain for control
#define SERVO_KI 0.01f                 // Integral gain for control
#define SERVO_KD 0.0f                  // Derivative gain for control
#define PWM_CENTER 1500                // Center PWM pulse width in microseconds
#define SERVO_CONTROL_FREQUENCY 100.0f // Control loop frequency in Hz
#define INTEGRAL_WINDUP_GUARD 1000.0f  // Integral windup guard value
#define OUTPUT_LIMIT 350.0f            // Maximum output limit
#define ADC_RESOLUTION 4095.0f         // 12-bit ADC
#define MAX_PWM_PULSE_WIDTH 2000       // Maximum PWM pulse width in microseconds
#define MIN_PWM_PULSE_WIDTH 500       // Minimum PWM pulse width in microseconds
#define NUM_ANGLE_SAMPLES 20           // Number of samples for moving average filter

#endif /* __SERVO_CONFIG_H__ */