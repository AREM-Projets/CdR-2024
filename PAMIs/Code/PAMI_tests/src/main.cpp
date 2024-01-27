// -----Includes-----
#include <Arduino.h>


#include "pins.h"
#include "BasicStepperDriver.h"

// -----Function prototypes-----
void Update_IT_callback(void); // Timer interrupt
void startMatch();

// -----Global variables-----

/* Match timer */
TIM_TypeDef *Instance = TIM7;
HardwareTimer *MyTim = new HardwareTimer(Instance);
volatile uint8_t match_timer_s = 0;
volatile bool match_ongoing = false;

/* Motors */
BasicStepperDriver Motor_G(200, DIR_MOTOR_G, STEP_MOTOR_G, SLEEP);
BasicStepperDriver Motor_D(200, DIR_MOTOR_D, STEP_MOTOR_D, SLEEP);

// -----Main-----
void setup() 
{
    /* Init Pins and Buses*/
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    Serial.begin(9600);

    /* Init Motors */
    

    /* Init Timer7 */
    MyTim->setOverflow(1, HERTZ_FORMAT);
    MyTim->attachInterrupt(Update_IT_callback);
    MyTim->resume();

    /* Other */
    startMatch();
}

void loop() 
{
    delay(1000);
    Serial.println(millis());
}

// -----Functions-----

/* Call this function to start a match */
void startMatch()
{
    match_timer_s = 0;
    match_ongoing = true;
    MyTim->setCount(0);
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(SLEEP, HIGH);
}

void Update_IT_callback(void)
{
    // Only Timer7 has an interrupt that ends here. 
    match_timer_s++;

    if(match_timer_s == 10)
    {
        match_ongoing = false;
        digitalWrite(LED_BUILTIN, LOW);
        digitalWrite(SLEEP, LOW); // turns off the motors among other things
    }
}
