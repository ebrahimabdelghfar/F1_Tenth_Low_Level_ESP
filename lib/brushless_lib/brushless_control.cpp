#include "brushless_control.h"
#include <ESP32Servo.h>
#include "motor_config.h"
Servo esc; // Create a ServoEasing object for ESC control
void BrushlessControl::init(int pwm_pin) {
    esc.attach(pwm_pin, MIN_THROTTLE, MAX_THROTTLE); // Attach ESC to specified pin with typical min/max pulse widths
    esc.writeMicroseconds(NEUTRAL); // Start at neutral position (1500us)
    delay(2000); // Delay to allow ESC to initialize (some ESCs require a delay after power-up before receiving signals)
}

void BrushlessControl::set_throttle(float throttle) {
    // Map throttle from [-1.0, 1.0] to [1000, 2000] microseconds for ESC control
    int pulse_width = map(throttle * 100, -100, 100, MIN_THROTTLE, MAX_THROTTLE);
    esc.writeMicroseconds(pulse_width); // Send the pulse width to the ESC
}

void BrushlessControl::stop() {
    esc.writeMicroseconds(NEUTRAL); // Send neutral signal to stop the motor (assuming 1500us is neutral)
}
