#ifndef CONFIGURATION_H
#define CONFIGURATION_H

/* Enable servo actuator */
#define SERVO_PIN 12
#ifdef SERVO_PIN
#define PID_MIN 0
#define PID_MAX 90
#endif

/* Enable Dallas thermo sensor */
#define ONE_WIRE_PIN 8

/* Enable heart beat*/
#define BLINK_PIN 13

/*
 * Thermistor works only with the following circuit topology
 *
 * Vcc---NTC---ADC---SERIES_RESISTOR---GND
 * 
 * Analog pin used to read the NTC
 */
// #define NTC_PIN A1

/* PID regulator range */
#ifndef PID_MIN
#define PID_MIN 0
#endif
#ifndef PID_MAX
#define PID_MAX 255
#endif

// #define PWM_PIN 11

// #define AC_PHASE_CONTROL
#ifdef AC_PHASE_CONTROL
#define DETECT_ZERO_CROSS_PIN 2
#define TRIAC_PIN 11
#define TRIAC_PULSE 4
#endif

#endif