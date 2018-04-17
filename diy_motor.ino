#include "Arduino.h"
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include "pid.h"

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

PIDController pidController(1.5, 0.3, 0, 0.25f);

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
#define POWER_LEVEL_MIN 25
#define POWER_LEVEL_MAX 255

int powerLevel = POWER_LEVEL_DEFAULT;
int setpoint = 500;

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
    pidController.setProperties(0, 255);
    pidController.setItermProperties(-100, 100);

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

    pidController.setSetpoint(setpoint);
}

enum {
    MOTOR_STATE_STOP,
    MOTOR_STATE_SPINUP,
    MOTOR_STATE_WORKING
};

enum {
    OPERATION_MODE_MANUAL,
    OPERATION_MODE_PID
};

uint8_t operationMode = OPERATION_MODE_PID;

float frequency;
int rpm;   
int prevRpm; 
uint8_t motorState = MOTOR_STATE_STOP;

uint8_t button1PrevState = HIGH;
uint8_t button1State = HIGH;

uint8_t button2PrevState = HIGH;
uint8_t button2State = HIGH;

void loop()
{
    
    button1State = digitalRead(BUTTON_1);
    if (button1State == LOW && button1PrevState == HIGH)
    {

        if (operationMode == OPERATION_MODE_MANUAL) {
            powerLevel -= 5;
            if (powerLevel < POWER_LEVEL_MIN) {
                powerLevel = POWER_LEVEL_MIN;
            }
        } else if (operationMode == OPERATION_MODE_PID) {
            setpoint -= 10;
            if (setpoint < 200) {
                setpoint = 200;
            }
            pidController.setSetpoint(setpoint);
        }
    }
    button1PrevState = button1State;

    button2State = digitalRead(BUTTON_2);
    if (button2State == LOW && button2PrevState == HIGH)
    {
        if (operationMode == OPERATION_MODE_MANUAL) {
            powerLevel += 5;
            if (powerLevel >= POWER_LEVEL_MAX) {
                powerLevel = POWER_LEVEL_MAX;
            }
        } else if (operationMode == OPERATION_MODE_PID) {
            setpoint += 10;
            if (setpoint > 1200) {
                setpoint = 1200;
            }
            pidController.setSetpoint(setpoint);
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

    // if (shaftSensorStatus == LOW) {
        // Serial.println(millis());
    // }
    
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

        uint32_t diff = edgeMicros - prevEdgeMicros;

        if (diff < 1000000 && diff > 0) {
            //readouts makes sense

            smoothEdge = smooth(diff, 0.97, smoothEdge);
        
            frequency = 1000000.0f / smoothEdge;
            rpm = frequency * 15; // x * 60 / 15 = x * 15
        
            static uint32_t spinupLeaveTime = 0;

            //Define current motor state
            if (rpm == 0) {
                motorState = MOTOR_STATE_STOP;
            } else if (prevRpm == 0 && rpm > 0) {
                motorState = MOTOR_STATE_SPINUP;
                spinupLeaveTime = millis() + 3000; //Lock spinup for one second
            } else if (motorState == MOTOR_STATE_SPINUP && millis() < spinupLeaveTime) {
                /*
                 * This is dummy condition that prevents motor from entering working state
                 * too early
                 */
            } else {
                //In other cases we assume motor is just working
                motorState = MOTOR_STATE_WORKING;
            }

            prevRpm = rpm;
        }
    }

    if (prevEdgeMicros == 0 || micros() - prevEdgeMicros > 1000000) {
        smoothEdge = 0;
        frequency = 0;
        rpm = 0;
        prevRpm = 0;
        oledOpen = true;
        motorState = MOTOR_STATE_STOP;
    }

    prevStatus = shaftSensorStatus;

    static uint32_t nextPowerUpdateMillis = millis();

    if (operationMode == OPERATION_MODE_PID && millis() > nextPowerUpdateMillis) {

        if (motorState == MOTOR_STATE_WORKING) {
            //Calculate using PID controller 
            powerLevel = pidController.compute(rpm, millis());
        } else {
            pidController.resetIterm();
            powerLevel = 168;
        }
        nextPowerUpdateMillis = millis() + 100;
    }

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

        if (operationMode == OPERATION_MODE_PID) {

            display.setTextSize(1);
            display.setCursor(0, 36);
            display.print("Setpoint: ");
            display.print(setpoint);

            display.setTextSize(1);
            display.setCursor(0, 55);
            display.print("I: ");
            display.print(pidController.getIterm());

            display.setTextSize(1);
            display.setCursor(64, 55);
            display.print("D: ");
            display.print(pidController.getDterm());
        }

        display.display();

        lastOledUpdate = millis();
    }

}
