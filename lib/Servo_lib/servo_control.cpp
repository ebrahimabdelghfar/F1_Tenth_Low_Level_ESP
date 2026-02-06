#include "servo_control.h"
void ServoControl::Servo_Init(int pin) {
    servo.attach(pin, MIN_PWM_PULSE_WIDTH, MAX_PWM_PULSE_WIDTH); // Attach servo to specified pin with min/max pulse widths
    servo.writeMicroseconds(PWM_CENTER); // Start at center position (1500us)
    delay(1000); // Delay to allow servo to initialize
}


