#include "brushless_control.h"
#include <ESP32Servo.h>
Servo esc; // Create a ServoEasing object for ESC control
class BrushlessControl {
  public:
    void init(int pwm_pin) {
        esc.attach(pwm_pin, 500, 2000); // Attach ESC to specified pin with typical min/max pulse widths
    }
    void set_throttle(float throttle) {
        // Map throttle from [-1.0, 1.0] to [500, 2000] microseconds for ESC control
        int pulse_width = map(throttle * 100, -100, 100, 500, 2000);
        esc.writeMicroseconds(pulse_width); // Send the pulse width to the ESC
    }
    void stop() {
        esc.writeMicroseconds(1500); // Send neutral signal to stop the motor (assuming 1500us is neutral)
    }
};
