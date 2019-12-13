// Include the graphics library.
#include <Arduino.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <PID_v1.h>
#include <SimpleTimer.h>
#include <U8glib.h>
#include <Servo.h>

#define ONE_WIRE_PIN 8
#define PLUS_PIN 10
#define MINUS_PIN 9
#define BLINK_PIN 13
#define SERVO_PIN 12

Servo myservo;

// Data wire is plugged into port 8 on the Arduino
OneWire oneWire(ONE_WIRE_PIN);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// arrays to hold device addresses
DeviceAddress thermometerAddress;

int temperature = 20;
int temperatureEEPROMAddress = 0;
float temperatureSensor;
int angel;
int plusPin = PLUS_PIN;
int minusPin = MINUS_PIN;

SimpleTimer timer;
int screenTimeout = 10000;
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
        sprintf(tmp, "set     %d%cC", temperature, 0xb0);
    } else {
        sprintf(tmp, "set      %d%cC", temperature, 0xb0);
    }
    char str_temperatureSensor[6];
    dtostrf(temperatureSensor, 4, 1, str_temperatureSensor);
    if (temperatureSensor < 0) {
        sprintf(tmp1, "sensor %s%cC", str_temperatureSensor, 0xb0);
    } else {
        sprintf(tmp1, "sensor  %s%cC", str_temperatureSensor, 0xb0);
    }
    sprintf(tmp2, "out     %d%c", angel, 0xb0);

    if (screenOn) {
        u8g.setFont(u8g_font_gdb14);
        u8g.drawStr(15, 15, "Termostat");
        u8g.setFont(u8g_font_gdb12);
        u8g.drawStr(0, 25, tmp);
        u8g.drawStr(0, 50, tmp1);
        u8g.drawStr(0, 70, tmp2);
    }
}

void readTemperature(DeviceAddress deviceAddress) {
    sensors.requestTemperatures();
    temperatureSensor = sensors.getTempC(deviceAddress);
}

void disableDisplay() { screenOn = false; }

bool enableDisplay() {
    timer.setTimeout(screenTimeout, disableDisplay);
    if (screenOn) {
        return true;
    } else {
        screenOn = true;
        return false;
    }
}

void setup(void) {
    pinMode(BLINK_PIN, OUTPUT);
    pinMode(plusPin, INPUT);
    pinMode(minusPin, INPUT);
    // turn on pullup resistors
    digitalWrite(plusPin, HIGH);
    digitalWrite(minusPin, HIGH);

    // Set font.
    // u8g.setFont(u8g_font_gdb12);

    // Start up sensor library
    sensors.begin();
    sensors.getAddress(thermometerAddress, 0);
    temperature = EEPROM.read(temperatureEEPROMAddress);

    // turn the PID on
    myPID.SetMode(AUTOMATIC);
    enableDisplay();

    myPID.SetOutputLimits(0, 90);
    myPID.SetMode(AUTOMATIC);

    myservo.attach(SERVO_PIN);
}

void loop(void) {
    u8g.firstPage();
    do {
        draw();
    } while (u8g.nextPage());

    if (digitalRead(minusPin) == LOW) {
        if (enableDisplay()) {
            temperature--;
            if (temperature < -99) {
                temperature = -99;
            }
            EEPROM.update(temperatureEEPROMAddress, temperature);
        } else {
            delay(500);
        }
        delay(100);
    }
    if (digitalRead(plusPin) == LOW) {
        if (enableDisplay()) {
            temperature++;
            if (temperature > 99) {
                temperature = 99;
            }
            EEPROM.update(temperatureEEPROMAddress, temperature);
        } else {
            delay(500);
        }

        delay(100);
    }
    readTemperature(thermometerAddress);

    double gap = abs((float)temperature - temperatureSensor);  // distance away from setpoint
    if (gap < 10) {  // we're close to setpoint, use conservative tuning parameters
        myPID.SetTunings(consKp, consKi, consKd);
    } else {
        // we're far from setpoint, use aggressive tuning parameters
        myPID.SetTunings(aggKp, aggKi, aggKd);
    }

    Input = (double)temperatureSensor;
    Setpoint = (double)temperature;
    myPID.Compute();
    
    angel = (int)Output;
    myservo.write(angel);

    timer.run();

    // Heart_beat
    digitalWrite(BLINK_PIN, HIGH);
    delay(10);
    digitalWrite(BLINK_PIN, LOW);
}
