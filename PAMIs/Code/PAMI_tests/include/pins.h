/* pins.h : liste des pins utilisés sur la carte
 * Permet de modifier seulement ce fichier lorsqu'une modification du pinout est faite.
 */
#include <Arduino.h>

/* ---UART2 : Serial--- */

#define USART_RX A2 // output
#define USART_TX A7 // output

/* ---I2C--- */

#define I2C_SDA D4 // SDA  
#define I2C_SCL D5 // SCL 

/* ---SPI--- */

#define SPI_MOSI D2 
#define SPI_MISO D10
#define SPI_SCLK A1
#define SPI_SSEL D3

/* ---Stepper driver A4988--- */

#define DIR_MOTOR_G A3  // output
#define DIR_MOTOR_D A4  // output

#define STEP_MOTOR_G D0 // output PWM
#define STEP_MOTOR_D D1 // output PWM

/* ---Autres--- */

#define TIRETTE D11 // input Pullup
#define TEAM_SELECT D12 // input Pullup
#define SLEEP A0 // output