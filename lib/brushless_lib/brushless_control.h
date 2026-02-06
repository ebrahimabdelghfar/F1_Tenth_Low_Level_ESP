#ifndef __BRUSHLESS_CONTROL_H__
#define __BRUSHLESS_CONTROL_H__
#include <Arduino.h>
class BrushlessControl {
  public:
    /**
     * @brief Initialize the brushless motor controller (e.g., set up PWM pin, calibrate ESC)
     * @param pwm_pin The GPIO pin used for PWM signal to the ESC
     * @return None
     */
    void init(int pwm_pin);
    /**
     * @brief Set throttle value between -1.0 (full reverse) and 1.0 (full forward)
     * @param throttle Throttle value in range [-1.0, 1.0]
     * @return None
     * @note This function should handle ESC arming and signal generation based on the throttle input
     */
    void set_throttle(float throttle);

    /**
     * @brief Stop the motor immediately (e.g., set throttle to 0 or send a stop signal)
     * @return None
     * @note This function can be used for safety to quickly stop the motor in case of disconnection or emergency conditions
     */
    void stop();
  private:
    // Add private members for motor control (e.g., PWM pin, ESC calibration)
};
#endif // __BRUSHLESS_CONTROL_H__