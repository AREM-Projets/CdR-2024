/*
AREM
(Antoine C)


Code de l'actionneur du robot CDR:
Composants:
- Servo moteur 360° (asservi en vitesse) avec pins: 5V, GND, PWM
- 2 TOFs VL53L1X avec les pins: SDA, SCL, GND, 5V, XSDN

Avec COEF_EXTENSION_BRAS = 1.7 actionner prends 8s
*/

#include <Wire.h>
#include <Servo.h>
#include <VL53L1X.h>

#define TOF1_XSDN PA9 //pin XSDN du TOF1
#define TOF2_XSDN PA8 //pin XSDN du TOF2

#define COEF_EXTENSION_BRAS 1.7 //1.7 permet une extension de 7cm environ avec une derive de 0.5mm vers l'arriere par extension complete
#define INSTRUCTION_SIZE 30


VL53L1X sensor1;
VL53L1X sensor2;

Servo servo;

int pos = 0; //"position" du servo moteur qui n'en est pas un en fait (servo 360° = moteur à courant continu asservi en vitesse et pilotable en vitesse et direction au lieu de pilotable en angle )
int flag_extension = 0; //flag d'autorisation extension bras (la raspi autorise quand elle est devant un panneau)

void setup()
{
  //sequence pour reset le TOF1
  pinMode(TOF1_XSDN, OUTPUT);
  digitalWrite(TOF1_XSDN, 0);
  // pinMode(TOF1_XSDN, INPUT); //normalement on s'attends a un digitalWrite(TOF1_XSDN, 1) mais sombre histoire la ca fonctionne.
  //en fait il semble que le pin XSDN attende du 5V pour s'activer sur le TOF or la STM sort du 3,7V... RIP


  //sequence pour reset le TOF2
  pinMode(TOF2_XSDN, OUTPUT);
  digitalWrite(TOF2_XSDN, 0);
  

  while (!Serial) {} //on attends que le port serie soit dispo

  Serial.begin(9600);
  
  //setup du bus i2c pour la communication avec le TOF
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C


  //setup du TOF1
  pinMode(TOF1_XSDN, INPUT);
  sensor1.stopContinuous();
  sensor1.setTimeout(500);
  if (!sensor1.init())
  {
    Serial.println("Failed to detect and initialize sensor1!");
    while (1) {}
  }

  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor1, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  sensor1.setDistanceMode(VL53L1X::Long);
  sensor1.setMeasurementTimingBudget(50000);
  sensor1.setAddress(10);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  sensor1.startContinuous(50); //demarre les mesures en continu




  //setup du TOF2
  pinMode(TOF2_XSDN, INPUT);
  sensor2.stopContinuous();
  sensor2.setTimeout(500);
  if (!sensor2.init())
  {
    Serial.println("Failed to detect and initialize sensor2!");
    while (1) {}
  }

  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor1, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  sensor2.setDistanceMode(VL53L1X::Long);
  sensor2.setMeasurementTimingBudget(50000);
  sensor2.setAddress(12);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  sensor2.startContinuous(50); //demarre les mesures en continu








  Serial.println("<Actionneur en fonction>");
  Serial.println("\t<Scan>");
}







void loop()
{

  //verification du port serie
  if (Serial.available()) {
    //si un ordre est envoyé
    char instruction[INSTRUCTION_SIZE];
    int i = 0;

    while (Serial.available() && i<INSTRUCTION_SIZE) {
      //on recupere tout les caracteres
      instruction[i] = Serial.read();
      i++;

      //decodage instruction
      if (instruction[0] == 'e') {
        //on autorise l'extension
        flag_extension = 1;
        Serial.println("<<autorisation extension");
      }
    }

  }



  if (sensor1.timeoutOccurred())
  {
    //procedure si le TOF deconne, on le redemarre tout simplement
    Serial.print(" TIMEOUT");
    Serial.println("; Redemarrage actionneur..");

    //je ne suis pas sur de l'utilite des delais mais ils sont la
    sensor1.stopContinuous();
    delay(10);
    pinMode(TOF1_XSDN, OUTPUT);
    digitalWrite(TOF1_XSDN, 0);
    pinMode(TOF1_XSDN, INPUT);
    sensor1.startContinuous(200);
    delay(10);
  }

  else if (flag_extension)
  {
    //quelques prompts utiles au debug
    // Serial.print("distance: ");
    // Serial.print(sensor1.read());
    // Serial.print(", angle moteur: ");
    // Serial.print(servo.read());
    // Serial.println();

    //test de proximite d'un obstacle (normalement d'un panneau solaire)
    if (sensor1.read() < 100)
    {
      digitalWrite(LED_BUILTIN, 1);
      Serial.println("\t<Servo moteur en fonction>");

      servo.attach(D9); //demarrage du servomoteur

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

      Serial.println("\t<Servo moteur arrete>");
      Serial.println("\t<Scan>");

      digitalWrite(LED_BUILTIN, 0);


      flag_extension = 0;

      //on envoie le message comme quoi on a fini
      Serial.println(">>fin");
    }
  }
}