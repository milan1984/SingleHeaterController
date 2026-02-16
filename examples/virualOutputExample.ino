#include "SingleHeaterController.h"

// Virtual output bit (instead of physical SSR pin)
bool heaterBit = false;

/**
 * Custom output function.
 * This replaces digitalWrite().
 * It will be called internally by the heater controller.
 */
void myVirtualOutput(bool state)
{
    heaterBit = state;
}

// Create heater controller using virtual output
// A0  -> temperature input
// 150 -> setpoint
// 5000 ms -> window size
SingleHeaterController heater1(
    A0,                    // ADC input pin (temperature sensor)
    150.0,                 // Setpoint
    5000,                  // PID window size
    ProbeType::NTC,        // Probe type
    myVirtualOutput        // Custom output callback
);

void setup()
{
    Serial.begin(115200);
    heater1.begin();
}

void loop()
{
    heater1.update();

    // Example: use virtual output bit
    // (for example drive another logic or print it)

    Serial.print("Heater1: ");
    Serial.print(heater1.getTemperature());
    Serial.print(" °C, Output window value: ");
    Serial.print(heater1.getOutput());
    Serial.print(" | Virtual SSR state: ");
    Serial.println(heaterBit ? "ON" : "OFF");

    delay(500);
}
