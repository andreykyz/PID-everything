#ifndef CONFIGURATION_H
#define CONFIGURATION_H

/* Enable servo actuator */
// #define SERVO_PIN 12

/* Enable Dallas thermo sensor */
// #define ONE_WIRE_PIN 8

/* Enable heart beat*/
#define BLINK_PIN 13

/*
 * Thermistor works only with the following circuit topology
 *
 * Vcc---NTC---ADC---SERIES_RESISTOR---GND
 * 
 * Analog pin used to read the NTC
 */
#define NTC_PIN A1

/* PID regulator range */
#define PID_MIN 0
#define PID_MAX 255

#define PWM_PIN 11

#endif