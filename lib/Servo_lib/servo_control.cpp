#include "servo_control.h"
void ServoControl::Servo_Init(int pin, int feedbackPin)
{
    servo.attach(pin, MIN_PWM_PULSE_WIDTH, MAX_PWM_PULSE_WIDTH); // Attach servo to specified pin with min/max pulse widths
    servo.writeMicroseconds(PWM_CENTER);                         // Start at center position (1500us)
    analogPin = feedbackPin;                                     // Store the feedback pin for reading current angle
    delay(1000);                                                 // Delay to allow servo to initialize
}

void ServoControl::setServoPulseWidth(float effort)
{
    // Map angle (-90 to 90) to pulse width (1000 to 2000 microseconds)
    uint16_t pulseWidth = PWM_CENTER + (int16_t)effort;                           // effort is already in microseconds offset
    pulseWidth = constrain(pulseWidth, MIN_PWM_PULSE_WIDTH, MAX_PWM_PULSE_WIDTH); // Constrain to valid range
    servo.writeMicroseconds(pulseWidth);
}

float ServoControl::getCurrentAngle(void)
{
    int averageReading = 0;
    for (int i = 0; i < NUM_ANGLE_SAMPLES; i++)
    {
        int reading = analogRead(analogPin); // Read the current pulse width in microseconds
        averageReading += reading;
    }
    averageReading /= NUM_ANGLE_SAMPLES;
    float voltage = ((float)averageReading / ADC_RESOLUTION) * ESP32_VOLTAGE;
    float angle = (voltage / SERVO_MAX_VOLTAGE) * SERVO_MAX_ANGLE;
    return angle;
}

float ServoControl::constrainAngle(float x)
{
    x = fmod(x + 180.0f, 360.0f);
    if (x < 0)
        x += 360.0f;
    return x - 180.0f;
}
