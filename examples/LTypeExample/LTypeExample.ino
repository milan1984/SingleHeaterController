/*
  L-type (linear) Example for SingleHeaterController
  Demonstrates using the L_TYPE probe and readTemperatureTypeL().
*/

#include <Arduino.h>
#include <SingleHeaterController.h>

// Pins (adjust as needed)
const int TEMP_PIN = A1;
const int SSR_PIN = 12;

// Pass ProbeType::L_TYPE as the last constructor parameter
SingleHeaterController heaterL(TEMP_PIN, SSR_PIN, 200.0, 2000, ProbeType::L_TYPE);

const char* statusToString(TempStatus s)
{
    switch (s)
    {
    case TempStatus::BELOW_RANGE:
        return "BELOW_RANGE";
    case TempStatus::WITHIN_RANGE:
        return "WITHIN_RANGE";
    case TempStatus::ABOVE_RANGE:
        return "ABOVE_RANGE";
    default:
        return "SENSOR_FAULT";
    }
}

void setup()
{
    Serial.begin(115200);
    heaterL.begin();
}

void loop()
{
    TempStatus status = heaterL.update();
    Serial.print("L-Probe Temp: ");
    Serial.print(heaterL.getTemperature());
    Serial.print(" C, Output: ");
    Serial.print(heaterL.getOutput());
    Serial.print(" ms, Status: ");
    Serial.println(statusToString(status));

    delay(500);
}
