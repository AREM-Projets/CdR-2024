/* pins.h : liste des pins utilisés sur la carte
 * Permet de modifier seulement ce fichier lorsqu'une modification du pinout est faite.
 */
#include <Arduino.h>

/* ---UART2 : Serial--- */

#define PIN_SERIAL_RX A2 // output
#define PIN_SERIAL_TX A7 // output

/* ---Stepper driver A4988--- */

#define MOTORS_SLEEP D6 // output
#define DIR_MOTOR_G D2  // output
#define DIR_MOTOR_D D4  // output

#define STEP_MOTOR_G D3 // output PWM
#define STEP_MOTOR_D D5 // output PWM

/* ---Capteurs--- */

#define SDA_TOF D0 // SDA (I2C) 
#define SCL_TOF D1 // SCL (I2C)

/* ---Autres--- */

#define TIRETTE D11 // input Pullup
#define TEAM_SELECT D12 // input Pullup
