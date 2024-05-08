/*
AREM
(Antoine C)


Code de l'actionneur du robot CDR:
Composants:
- Servo moteur 360° (asservi en vitesse) avec pins: 5V, GND, PWM
// - 2 TOFs VL53L1X avec les pins: SDA, SCL, GND, 5V, XSDN

Avec COEF_EXTENSION_BRAS = 1.7 actionner prends 8s
*/

#include <Wire.h>
#include <Servo.h>
#include <VL53L1X.h>


#define COEF_EXTENSION_BRAS 1.7 //1.7 permet une extension de 7cm environ avec une derive de 0.5mm vers l'arriere par extension complete

Servo servo;

int pos = 0; //"position" du servo moteur qui n'en est pas un en fait (servo 360° = moteur à courant continu asservi en vitesse et pilotable en vitesse et direction au lieu de pilotable en angle )
int flag_extension = 0; //flag d'autorisation extension bras (la raspi autorise quand elle est devant un panneau)



void setup()
{
  while (!Serial) {} //on attends que le port serie soit dispo

  Serial.begin(9600);

  Serial.println("<Actionneur en fonction>");
  Serial.println("\t<Scan>");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}



void loop()
{
  //verification du port serie
  if (Serial.available()) {
    //decodage instruction
    if (Serial.read() == 'e') {

        digitalWrite(LED_BUILTIN, HIGH);
        servo.attach(D11); //demarrage du servomoteur
        //le pilotage du servo est cahotique mais la sequence ci dessous fonctionne bien
        //on deploie le bras
        for (pos = 0; pos <= 100; pos += 1)
        {
          // Serial.println(pos); //du debug
          servo.write(pos);
          delay(19*COEF_EXTENSION_BRAS); //delay(19*COEF); //autre reglage plus efficace pour des COEFS proche de 1
        }
        //on retracte le bras
        for (pos = 100; pos <= 200; pos += 1)
        {                   
          servo.write(pos);
          delay(17*COEF_EXTENSION_BRAS); //delay(15*COEF); //autre reglage plus efficace pour des COEFS proche de 1
        }

        servo.detach(); //arret moteur
        digitalWrite(LED_BUILTIN, LOW);
    }
  }
}