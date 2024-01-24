// -----Includes-----
#include <Arduino.h>

#include "pins.h"
#include "BasicStepperDriver.h"

// -----Function prototypes-----

// -----Global variables-----
BasicStepperDriver Motor_G(200, DIR_MOTOR_G, STEP_MOTOR_G, MOTORS_SLEEP);
BasicStepperDriver Motor_D(200, DIR_MOTOR_D, STEP_MOTOR_D, MOTORS_SLEEP);

// -----Main-----
void setup() 
{
    /* Init Pins and Buses*/
    pinMode(LED_BUILTIN, OUTPUT);
    
    Serial.begin(9600);

    /* Init software */
}

void loop() 
{
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
}

// -----Functions-----