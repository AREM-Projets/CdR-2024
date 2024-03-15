#include <Arduino.h>
#define LED LED_BUILTIN

String message; // Close enough to std::string, but why...

void setup() 
{
    Serial.begin(9600);
    message = "";
    Serial.println("\nSerial data transmission initialised");

    pinMode(LED, OUTPUT);

    digitalWrite(LED, LOW);
}

void loop() 
{
    if(Serial.available()) 
    {
        message = Serial.readStringUntil('\n');
        message.trim(); // remove trailing whitespace

        Serial.print("Received : ");
        Serial.println(message);
        digitalWrite(LED, HIGH);
        delay(100);
        digitalWrite(LED, LOW);
        

        // if(message == "q") // Exemple de commande simple valide
        // {
        //     Serial.println("\nDoing something...\n");
        // }
        // if(message == "a")
        // {
        //   digitalWrite(LED, HIGH);
        // }
        // if(message == "b")
        // {
        //   digitalWrite(LED, LOW);
        // }

        Serial.flush();
    }
}