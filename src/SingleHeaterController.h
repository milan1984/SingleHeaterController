#ifndef SINGLE_HEATER_CONTROLLER_H
#define SINGLE_HEATER_CONTROLLER_H

#include <Arduino.h>
#include <PID_v1.h>

#define RESOLUTION_ADC_12BIT 4096.0
#define RESOLUTION_ADC_10BIT 1023.0

#define NTC_10K 10000.0
#define NTC_100K 100000.0

enum class TempStatus
{
    BELOW_RANGE,
    WITHIN_RANGE,
    ABOVE_RANGE,
    SENSOR_FAULT
};

class SingleHeaterController
{
public:
    SingleHeaterController(int tempPin, int ssrPin, double setpoint, unsigned long windowSize = 2000);

    void begin();
    TempStatus update(); // <-- now returns TempStatus

    double getTemperature() const;
    double getOutput() const;
    void setSetpoint(double setpoint);
    void setTempTolerance(double minOffset, double maxOffset);
    void setTunings(double Kp, double Ki, double Kd, int POn = P_ON_M);
    void setResistorValue(double resitorValue);
    void setWindowSize(unsigned long windowSize);
    void setADCResoultion(double resolution);

    void outputEnable(bool onOff = true); //Enabled by default

    bool isEnabled(void);

private:
    int _tempPin;
    int _ssrPin;
    unsigned long _windowSize;
    unsigned long _windowStartTime;
    double _dividerResistor = NTC_100K;
    double _input;
    double _output;
    double _setpoint;
    bool _outputEnabled;

    double _adcResoulution = RESOLUTION_ADC_12BIT;

    double _bCoefficient = 3950.0; // Default B coefficient
    double _toleranceMin = -2.0;   // Default: 2°C below setpoint allowed
    double _toleranceMax = 2.0;    // Default: 2°C above setpoint allowed

    PID _pid;

    double readTemperature(int pin);
};

#endif
