#define F_CPU 16000000

#include "math.h"
#include <util/delay.h>

float i = 1.0;

uint16_t leesSensorWaarde();
void initSensor();
void setServo(uint16_t value);
void initServo();

int main() {
    PORTB |= (1 << PB0);
    initUsart();
    initSensor();
    initServo();

    // PID constants
    double Kp = 20.5;  // Proportional gain
    double Ki = 0.8;   // Integral gain
    double Kd = 0.3;   // Derivative gain

    // Error values
    double distanceToTarget, prevDistanceToTarget;
    double integralSum = 0; // Sum of past errors (Integral)
    double proportionalTerm, integralTerm, derivativeTerm;

    // Maximum allowed error sum (to prevent windup)
    double maxIntegralSum = 100;
    double minIntegralSum = -100;

    while(1) {
        values[i] = leesSensorWaarde();

        if (i >= 6) {
            i = 0;

            // Calculate average of the last 6 sensor values
            uint16_t total = 0;
            for (uint8_t j = 0; j < 6; j++){
                total += values[j];
            }
            total = total / 6;

            // Convert sensor value to distance (mm)
            double sensorDistance = 3600 * pow(total, -1.046);
            distanceToTarget = 22 - sensorDistance;

            // Proportional Term (P)
            proportionalTerm = Kp * distanceToTarget;

            // Integral Term (I)
            integralSum += distanceToTarget;
            // Prevent integral windup by limiting the integral sum
            if (integralSum > maxIntegralSum) integralSum = maxIntegralSum;
            if (integralSum < minIntegralSum) integralSum = minIntegralSum;
            integralTerm = Ki * integralSum;

            // Derivative Term (D)
            derivativeTerm = Kd * (distanceToTarget - prevDistanceToTarget);
            prevDistanceToTarget = distanceToTarget;

            // Sum all terms and apply to servo
            setServo(proportionalTerm + integralTerm + derivativeTerm + 2800); // 2800 is the horizontal base value
        }
        i++;
        _delay_ms(1);
    }
}

uint16_t leesSensorWaarde() {
    ADCSRA |= (1 << ADSC); //Start conversion
    while (~ADCSRA & (1<<ADSC)); // Wait for flag
    uint16_t value = ADC;
    return value;
}


void initSensor() {
    ADMUX |= (1 << REFS0); // AVcc with external capacitor on AREF pin, ADC0.
    ADCSRA |= (1 << ADEN);
    ADCSRA |= ((1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2) | ADIE); // Set prescaler at 128 (16Mhz / 200Khz = 80, closed rounded up it 128), and enable interrupts for ADC
}

void setServo(uint16_t value) {
    if (value > 23000) {
        value=23000-1;
    }
        OCR1A = value;
}

void initServo() {
    DDRB |= (1 << PORTB0) | (1 << PORTB1); // set servo pins to output
    PORTB |= (1 << PINB0);
    TCCR1A |= (1 << COM1A1) | (1 << WGM11); // non-invert , fast-pwm , OCR1A top
    TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS11); // pre-scaler 8
    ICR1 = 23000; // 46hz
}
