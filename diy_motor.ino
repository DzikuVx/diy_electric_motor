#include "Arduino.h"
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#define IR_LED_PIN 9
#define IR_FREQUENCY 5000
#define IR_RECEVER_PIN 8
#define POWER_OUTPUT_PIN 10

#define BUTTON_1 12
#define BUTTON_2 14

#define VOLTAGE_MONITORING_PIN A1

#define SIGNAL_PIN 13

#define LOW_LOCK_TIME 5

void setup()
{
	pinMode(IR_LED_PIN, OUTPUT);
    // tone(IR_LED_PIN, IR_FREQUENCY);
    analogWrite(IR_LED_PIN, 64);

    pinMode(IR_RECEVER_PIN, INPUT);
    pinMode(POWER_OUTPUT_PIN, OUTPUT);
    digitalWrite(POWER_OUTPUT_PIN, LOW);


    pinMode(VOLTAGE_MONITORING_PIN, INPUT);

    Serial.begin(115200);

    display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // initialize with the I2C addr 0x3C (for the 128x32)
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.clearDisplay();
    display.display();
}

void loop()
{
    
    static uint32_t lastOledUpdate = 0;
    static uint8_t prevStatus;

    int vBatAdc = analogRead(VOLTAGE_MONITORING_PIN);
    int vBat = map(vBatAdc, 0, 430, 0, 121);

    bool oledOpen = false;

    static uint8_t status = LOW;
    static uint32_t lastLowInput = millis();
    
    if (digitalRead(IR_RECEVER_PIN) == LOW) {
        lastLowInput = millis();
    }

    if (millis() - lastLowInput > LOW_LOCK_TIME) {
        status = LOW;
    } else {
        status = HIGH;
    }

    // digitalWrite(SIGNAL_PIN, status);

    if (status == LOW) {
        analogWrite(POWER_OUTPUT_PIN, 100);
    } else {
        analogWrite(POWER_OUTPUT_PIN, 0);
    }

    if (prevStatus == LOW && status == HIGH) {
        oledOpen = true;
    } else {
        oledOpen = false;
    }

    prevStatus = status;

    if (millis() - lastOledUpdate > 300 && oledOpen) {
        display.clearDisplay();

        display.setTextSize(1);
        display.setCursor(0, 0);
        display.print("Vbat: ");
        display.print(vBat / 10.f);
        display.print("V");

        display.display();

        lastOledUpdate = millis();
    }

}
