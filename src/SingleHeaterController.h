#ifndef SINGLE_HEATER_CONTROLLER_H
#define SINGLE_HEATER_CONTROLLER_H

#include <Arduino.h>
#include <PID_v1.h>

#define V_REF_MCU 3.3 // Reference voltage for ADC
#define NTC_10K 10000.0
#define NTC_100K 100000.0


/*Default values for linear scaling */
#define TEMP_L_PROBE_MIN  100.0   // 100°C
#define TEMP_L_PROBE_MAX  300.0   // 300°C

/* Default voltage for L-probe type (linear) */
#define V_MIN_ADC          1.122    // 100°C  -> 1.7V * (3.3 / 5)
#define V_MAX_ADC          3.3     // 300°C  -> 5.0V * (3.3 / 5)


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

class SingleHeaterController
{
public:
    SingleHeaterController(int tempPin, int ssrPin, double setpoint, unsigned long windowSize = 2000, ProbeType probe = ProbeType::NTC);

    void begin();
    TempStatus update(); // <-- now returns TempStatus

    double getTemperature() const;
    double getOutput() const;
    void setSetpoint(double setpoint);
    void setTempTolerance(double minOffset, double maxOffset);
    void setTunings(double Kp, double Ki, double Kd, int POn = P_ON_M);
    void setResistorValue(double resitorValue);
    void setWindowSize(unsigned long windowSize);

    void outputEnable(bool onOff = true); // Enabled by default

    bool isEnabled(void);

    /**
     * @brief Set the error detection delay (timeout) in milliseconds.
     *
     * Defines how long a temperature must stay outside the valid range
     * before an error is confirmed.
     *
     * @param delayMs Delay time in milliseconds (e.g., 100 = 0.1s)
     */
    void setErrorDelay(unsigned long delayMs);

    void setMCUVoltageReference(double voltage);

    void setProbeTypeLParameters(double vMinADC, double vMaxADC, double tempMin, double tempMax);

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

    double _mcuVoltageReference = V_REF_MCU; // Default MCU reference voltage

    //L-type probe parameters
    double _vMinADC = V_MIN_ADC;
    double _vMaxADC = V_MAX_ADC;
    double _tempMinProbe = TEMP_L_PROBE_MIN;
    double _tempMaxProbe = TEMP_L_PROBE_MAX;


    double _bCoefficient = 3950.0; // Default B coefficient
    double _toleranceMin = -2.0;   // Default: 2°C below setpoint allowed
    double _toleranceMax = 2.0;    // Default: 2°C above setpoint allowed

    PID _pid;

    unsigned long _errorStartTime = 0;
    bool _errorActive = false;
    unsigned long _errorDelay = 0; // ms
    TempStatus _lastStableStatus = TempStatus::WITHIN_RANGE;

    ProbeType _probeType = ProbeType::NTC;

    double readTemperature(int pin);
    double readTemperatureTypeL(int pin);

    TempStatus validateTemperatureError(float input);
};

#endif
