#include "SingleHeaterController.h"

// Create two independent heater controllers
SingleHeaterController heater1(A0, 3, 150.0, 5000); // A0 input, D3 SSR, 150°C
SingleHeaterController heater2(A1, 4, 180.0, 5000); // A1 input, D4 SSR, 180°C

void setup() {
    Serial.begin(9600);
    heater1.begin();
    heater2.begin();
}

void loop() {
    heater1.update();
    heater2.update();

    // Optional: Debug info
    Serial.print("Heater1: ");
    Serial.print(heater1.getTemperature());
    Serial.print("°C, Output: ");
    Serial.print(heater1.getOutput());

    Serial.print(" | Heater2: ");
    Serial.print(heater2.getTemperature());
    Serial.print("°C, Output: ");
    Serial.println(heater2.getOutput());
}
