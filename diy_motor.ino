#include "Arduino.h"
#include <Adafruit_SSD1306.h>
#include <Wire.h>

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

#define LOW_LOCK_TIME 0

#define POWER_LEVEL_DEFAULT 80
#define POWER_LEVEL_MIN 30
#define POWER_LEVEL_MAX 255

uint16_t powerLevel = POWER_LEVEL_DEFAULT;

uint32_t smooth(uint32_t data, float filterVal, float smoothedVal)
{
  if (filterVal > 1)
  {
    filterVal = .99;
  }
  else if (filterVal <= 0)
  {
    filterVal = 0;
  }

  smoothedVal = (data * (1 - filterVal)) + (smoothedVal * filterVal);

  return (uint32_t)smoothedVal;
}

void setup()
{
	pinMode(IR_LED_PIN, OUTPUT);
    digitalWrite(IR_LED_PIN, HIGH);

    pinMode(IR_RECEVER_PIN, INPUT);
    pinMode(POWER_OUTPUT_PIN, OUTPUT);
    digitalWrite(POWER_OUTPUT_PIN, LOW);

    pinMode(VOLTAGE_MONITORING_PIN, INPUT);

    pinMode(BUTTON_1, INPUT_PULLUP);
    pinMode(BUTTON_2, INPUT_PULLUP);

    Serial.begin(115200);

    Wire.setClock(400000);

    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.clearDisplay();
    display.display();
}

float frequency;
uint16_t rpm;   

uint8_t button1PrevState = HIGH;
uint8_t button1State = HIGH;

uint8_t button2PrevState = HIGH;
uint8_t button2State = HIGH;

void loop()
{
    
    button1State = digitalRead(BUTTON_1);
    if (button1State == LOW && button1PrevState == HIGH)
    {
        powerLevel -= 5;
        if (powerLevel < POWER_LEVEL_MIN) {
            powerLevel = POWER_LEVEL_MIN;
        }
    }
    button1PrevState = button1State;

    button2State = digitalRead(BUTTON_2);
    if (button2State == LOW && button2PrevState == HIGH)
    {
        powerLevel += 5;
        if (powerLevel >= POWER_LEVEL_MAX) {
            powerLevel = POWER_LEVEL_MAX;
        }
    }
    button2PrevState = button2State;

    static uint32_t lastOledUpdate = 0;
    static uint8_t prevStatus;
    static bool electromagnetEnabled = false;

    static uint32_t edgeMicros = 0;
    static uint32_t prevEdgeMicros = 0;
    static uint32_t smoothEdge = 0;

    int vBatAdc = analogRead(VOLTAGE_MONITORING_PIN);
    int vBat = map(vBatAdc, 0, 430, 0, 121);

    bool oledOpen = false;

    static uint8_t shaftSensorStatus = LOW;
    static uint32_t lastLowInput = millis();
    
    if (digitalRead(IR_RECEVER_PIN) == LOW) {
        lastLowInput = millis();
    }

    if (millis() - lastLowInput > LOW_LOCK_TIME) {
        shaftSensorStatus = LOW;
    } else {
        shaftSensorStatus = HIGH;
    }

    if (shaftSensorStatus == LOW) {
        Serial.println(millis());
    }
    
    oledOpen = false;

    // Magnet is in closest position to electromagnet 
    if (prevStatus == LOW && shaftSensorStatus == HIGH) {
       oledOpen = true; // Allow OLED to update

       //Disable electromagnet
       electromagnetEnabled = false;
    }

    // Shaft is in furthest dedectable distance from the electromagnet
    if (prevStatus == HIGH && shaftSensorStatus == LOW) {
        electromagnetEnabled = true;

        prevEdgeMicros = edgeMicros;
        edgeMicros = micros();

        smoothEdge = smooth(edgeMicros - prevEdgeMicros, 0.90, smoothEdge);

        frequency = 1000000.0f / smoothEdge;
        rpm = (frequency * 60) / 4;
    }

    if (prevEdgeMicros == 0 || micros() - prevEdgeMicros > 1000000) {
        smoothEdge = 0;
        frequency = 0;
        rpm = 0;
        oledOpen = true;
    }

    prevStatus = shaftSensorStatus;

    // Cycle electromagnet if needed
    if (electromagnetEnabled) {
        analogWrite(POWER_OUTPUT_PIN, powerLevel);
    } else {
        analogWrite(POWER_OUTPUT_PIN, 0);
    }

    if (millis() - lastOledUpdate > 100 && oledOpen) {
        display.clearDisplay();

        display.setTextSize(1);
        display.setCursor(0, 0);
        display.print("Vbat: ");
        display.print(vBat / 10.f);
        display.print("V");

        display.setTextSize(1);
        display.setCursor(0, 10);
        display.print("RPM: ");
        display.print(rpm);

        display.setTextSize(1);
        display.setCursor(0, 20);
        display.print("Power: ");
        display.print(powerLevel);

        display.display();

        lastOledUpdate = millis();
    }

}
