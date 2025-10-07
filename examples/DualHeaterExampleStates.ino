#include "SingleHeaterController.h"

SingleHeaterController heater1(A0, 3, 150.0, 5000);
SingleHeaterController heater2(A1, 4, 180.0, 5000);

void setup() {
    Serial.begin(9600);
    heater1.begin();
    heater2.begin();

    // Optional: set tighter or looser temp tolerance (± range around setpoint)
    heater1.setTempTolerance(-3.0, 2.0); // 3°C below to 2°C above setpoint
    heater2.setTempTolerance(-2.5, 2.5);
}

void loop() {
    TempStatus status1 = heater1.update();
    TempStatus status2 = heater2.update();

    Serial.print("Heater1: ");
    Serial.print(heater1.getTemperature());
    Serial.print("°C (");
    Serial.print(status1 == TempStatus::BELOW_RANGE ? "LOW" :
                 status1 == TempStatus::WITHIN_RANGE ? "OK" : "HIGH");
    Serial.print(") | Output: ");
    Serial.print(heater1.getOutput());

    Serial.print(" || Heater2: ");
    Serial.print(heater2.getTemperature());
    Serial.print("°C (");
    Serial.print(status2 == TempStatus::BELOW_RANGE ? "LOW" :
                 status2 == TempStatus::WITHIN_RANGE ? "OK" : "HIGH");
    Serial.print(") | Output: ");
    Serial.println(heater2.getOutput());

}
