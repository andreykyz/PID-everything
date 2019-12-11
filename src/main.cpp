// Include the graphics library.
#include <Arduino.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <SimpleTimer.h>
#include <U8glib.h>

#define ONE_WIRE_PIN 8
#define PLUS_PIN 10
#define MINUS_PIN 9

// Data wire is plugged into port 8 on the Arduino
OneWire oneWire(ONE_WIRE_PIN);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// arrays to hold device addresses
DeviceAddress thermometerAddress;

int temperature = 20;
int temperatureEEPROMAddress = 0;
float temperatureSensor;
int plusPin = PLUS_PIN;
int minusPin = MINUS_PIN;

SimpleTimer timer;
int screenTimeout = 10000;
bool screenOn = true;

// Initialize display.
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE | U8G_I2C_OPT_DEV_0);
void draw(void) {
    char tmp[20];
    char tmp1[20];

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
    if (screenOn) {
        u8g.setFont(u8g_font_gdb14);
        u8g.drawStr(15, 15, "Termostat");
        u8g.setFont(u8g_font_gdb12);
        u8g.drawStr(0, 35, tmp);
        u8g.drawStr(0, 60, tmp1);
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
    pinMode(plusPin, INPUT);
    pinMode(minusPin, INPUT);
    // turn on pullup resistors
    digitalWrite(plusPin, HIGH);
    digitalWrite(minusPin, HIGH);

    // Set font.
    u8g.setFont(u8g_font_gdb14);

    // Start up sensor library
    sensors.begin();
    sensors.getAddress(thermometerAddress, 0);
    temperature = EEPROM.read(temperatureEEPROMAddress);

    enableDisplay();
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
        }
        delay(100);
    }
    readTemperature(thermometerAddress);
    timer.run();
}
