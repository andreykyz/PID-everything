#include "Configuration.h"

#include <DIO2.h>
#ifdef ONE_WIRE_PIN
#include <DallasTemperature.h>
#include <OneWire.h>
#endif
#ifdef NTC_PIN
#include <thermistor.h>
#endif
#include <EEPROM.h>
#include <PID_v1.h>
#ifdef SERVO_PIN
#include <Servo.h>
#endif
#include <SimpleTimer.h>
#include <U8glib.h>

#define PLUS_PIN 10
#define MINUS_PIN 9

#ifdef SERVO_PIN
Servo myservo;
int angel, angelPrev;
#define PID_MIN 0
#define PID_MAX 90
#endif

#ifdef PWM_PIN
int pwmOut = 0;
#endif

#ifdef ONE_WIRE_PIN
OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature sensors(&oneWire);
DeviceAddress thermometerAddress;
#endif

#ifdef NTC_PIN
//                             R      B   pull-down
THERMISTOR thermistor(NTC_PIN,100000,3950,10000);
#endif

int temperature = 20;
int temperatureEEPROMAddress = 0;
float temperatureSensor;

SimpleTimer timer;
int timerID;
int screenTimeout = 20000;
bool screenOn = true;

// PID
// Define Variables we'll be connecting to
double Setpoint, Input, Output;
// Define the aggressive and conservative Tuning Parameters
double aggKp = 4, aggKi = 0.2, aggKd = 1;
double consKp = 1, consKi = 0.05, consKd = 0.25;

PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

// Initialize display.
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE | U8G_I2C_OPT_DEV_0);
void draw(void) {
    char tmp[20];
    char tmp1[20];
    char tmp2[20];

    if (temperature < 0) {
        sprintf(tmp, "set       %d%cC", temperature, 0xb0);
    } else {
        sprintf(tmp, "set        %d%cC", temperature, 0xb0);
    }
    char str_temperatureSensor[6];
    dtostrf(temperatureSensor, 4, 1, str_temperatureSensor);
    if (temperatureSensor < 0) {
        sprintf(tmp1, "sensor %s%cC", str_temperatureSensor, 0xb0);
    } else {
        sprintf(tmp1, "sensor  %s%cC", str_temperatureSensor, 0xb0);
    }
#ifdef SERVO_PIN
    sprintf(tmp2, "out       %d%c", angel, 0xb0);
#endif
#ifdef PWM_PIN
    sprintf(tmp2, "pwm       %d%%", (pwmOut * 100)/255);
#endif
    if (screenOn) {
        u8g.setFont(u8g_font_gdb14);
        u8g.drawStr(15, 15, "Termostat");
        u8g.setFont(u8g_font_gdb12);
        int logoStep = 32;
        int lineStep = 16;
        u8g.drawStr(0, logoStep, tmp);
        u8g.drawStr(0, logoStep + lineStep, tmp1);
        u8g.drawStr(0, logoStep + lineStep * 2, tmp2);
    }
}

void readTemperature() {
#ifdef ONE_WIRE_PIN
    sensors.requestTemperatures();
    temperatureSensor = sensors.getTempC(thermometerAddress);
#endif
#ifdef NTC_PIN
    temperatureSensor = thermistor.read();
#endif
}

void disableDisplay() { screenOn = false; }

bool enableDisplay() {
    timer.restartTimer(timerID);
    if (screenOn) {
        return true;
    } else {
        screenOn = true;
        return false;
    }
}

void setup(void) {
#ifdef BLINK_PIN
    pinMode2(BLINK_PIN, OUTPUT);
#endif
#ifdef PWM_PIN
    pinMode(PWM_PIN, OUTPUT);
#endif
    pinMode2(PLUS_PIN, INPUT);
    pinMode2(MINUS_PIN, INPUT);
    // turn on pullup resistors
    digitalWrite2(PLUS_PIN, HIGH);
    digitalWrite2(MINUS_PIN, HIGH);

#ifdef ONE_WIRE_PIN
    sensors.begin();
    sensors.getAddress(thermometerAddress, 0);
    temperature = EEPROM.read(temperatureEEPROMAddress);
#endif

    enableDisplay();

    myPID.SetOutputLimits(PID_MIN, PID_MAX);
    myPID.SetMode(AUTOMATIC);

#ifdef SERVO_PIN
    myservo.attach(SERVO_PIN);
#endif

    timerID = timer.setInterval(screenTimeout, disableDisplay);

}

void loop(void) {

    readTemperature();

    u8g.firstPage();
    do {
        draw();
    } while (u8g.nextPage());

    if (digitalRead2(MINUS_PIN) == LOW) {
        if (enableDisplay()) {
            temperature--;
            if (temperature < -99) {
                temperature = -99;
            }
            EEPROM.update(temperatureEEPROMAddress, temperature);
        }
        delay(100);
    }
    if (digitalRead2(PLUS_PIN) == LOW) {
        if (enableDisplay()) {
            temperature++;
            if (temperature > 99) {
                temperature = 99;
            }
            EEPROM.update(temperatureEEPROMAddress, temperature);
        }
        delay(100);
    }

    double gap = abs((float)temperature - temperatureSensor);
    if (gap < 10) {// Agressiv if more then 10
        myPID.SetTunings(consKp, consKi, consKd);
    } else {
        myPID.SetTunings(aggKp, aggKi, aggKd);
    }

    Input = (double)temperatureSensor;
    Setpoint = (double)temperature;
    myPID.Compute();

#ifdef SERVO_PIN
    angel = (PID_MAX-(int)Output);
    if (angel != angelPrev) {
        myservo.write(angel);
    }
    angelPrev = angel;
#endif
#ifdef PWM_PIN
    pwmOut = (int)Output;
    analogWrite(PWM_PIN, pwmOut);
#endif
    timer.run();

    // Heart_beat
#ifdef BLINK_PIN
    digitalWrite2(BLINK_PIN, HIGH);
    delay(10);
    digitalWrite2(BLINK_PIN, LOW);
#endif
}
