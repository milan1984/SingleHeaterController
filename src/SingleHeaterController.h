#ifndef SINGLE_HEATER_CONTROLLER_H
#define SINGLE_HEATER_CONTROLLER_H

#include <Arduino.h>
#include <PID_v1.h>

#define V_REF_MCU 3.3
#define NTC_10K 10000.0
#define NTC_100K 100000.0

/* Default values for linear scaling */
#define TEMP_L_PROBE_MIN  100.0
#define TEMP_L_PROBE_MAX  300.0

/* Default voltage for L-probe type (linear) */
#define V_MIN_ADC  1.122
#define V_MAX_ADC  3.3

enum class TempStatus
{
    BELOW_RANGE,
    WITHIN_RANGE,
    ABOVE_RANGE,
    SENSOR_FAULT
};

enum class ProbeType
{
    NTC,
    L_TYPE
};

/**
 * Output write callback prototype.
 * User can provide custom output handling (virtual output, IO expander, etc.)
 */
using OutputWriteCallback = void (*)(bool state);

class SingleHeaterController
{
public:

    /**
     * Standard constructor using physical SSR pin
     */
    SingleHeaterController(int tempPin,
                           int ssrPin,
                           double setpoint,
                           unsigned long windowSize = 2000,
                           ProbeType probe = ProbeType::NTC);

    /**
     * Constructor using custom output callback (virtual output)
     */
    SingleHeaterController(int tempPin,
                           double setpoint,
                           unsigned long windowSize,
                           ProbeType probe,
                           OutputWriteCallback callback);

    void begin();
    TempStatus update();

    double getTemperature() const;
    double getOutput() const;

    void setSetpoint(double setpoint);
    void setTempTolerance(double minOffset, double maxOffset);
    void setTunings(double Kp, double Ki, double Kd, int POn = P_ON_M);
    void setResistorValue(double resistorValue);
    void setWindowSize(unsigned long windowSize);

    void outputEnable(bool onOff = true);
    bool isEnabled(void);

    void setErrorDelay(unsigned long delayMs);
    void setMCUVoltageReference(double voltage);
    void setProbeTypeLParameters(double vMinADC, double vMaxADC, double tempMin, double tempMax);

private:

    void writeOutput(bool state);   // unified output handler

    int _tempPin;
    int _ssrPin = -1;               // -1 means no physical pin used
    OutputWriteCallback _outputCallback = nullptr;

    unsigned long _windowSize;
    unsigned long _windowStartTime;

    double _dividerResistor = NTC_100K;
    double _input = 0.0;
    double _output = 0.0;
    double _setpoint;
    bool _outputEnabled = true;

    double _mcuVoltageReference = V_REF_MCU;

    // L-type probe parameters
    double _vMinADC = V_MIN_ADC;
    double _vMaxADC = V_MAX_ADC;
    double _tempMinProbe = TEMP_L_PROBE_MIN;
    double _tempMaxProbe = TEMP_L_PROBE_MAX;

    double _bCoefficient = 3950.0;
    double _toleranceMin = -2.0;
    double _toleranceMax = 2.0;

    PID _pid;

    unsigned long _errorStartTime = 0;
    bool _errorActive = false;
    unsigned long _errorDelay = 0;
    TempStatus _lastStableStatus = TempStatus::WITHIN_RANGE;

    ProbeType _probeType = ProbeType::NTC;

    double readTemperature(int pin);
    double readTemperatureTypeL(int pin);
    TempStatus validateTemperatureError(float input);
};

#endif
