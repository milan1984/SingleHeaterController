#include "SingleHeaterController.h"

SingleHeaterController::SingleHeaterController(int tempPin, int ssrPin, double setpoint, unsigned long windowSize)
    : _tempPin(tempPin), _ssrPin(ssrPin), _setpoint(setpoint), _windowSize(windowSize), _outputEnabled(true),
      _pid(&_input, &_output, &_setpoint, 2.0, 5.0, 1.0, P_ON_M, DIRECT)
{
    _windowStartTime = millis();
}

void SingleHeaterController::begin()
{
    pinMode(_ssrPin, OUTPUT);
    analogReadResolution(12);
    _pid.SetMode(AUTOMATIC);
    _pid.SetOutputLimits(0, _windowSize);
}

TempStatus SingleHeaterController::update()
{
    TempStatus status = TempStatus::SENSOR_FAULT;

    if (millis() - _windowStartTime > _windowSize)
    {
        _windowStartTime += _windowSize;
    }

    _input = readTemperature(_tempPin);
    _pid.Compute();

    if (_outputEnabled)
    {
        digitalWrite(_ssrPin, (_output > millis() - _windowStartTime) ? HIGH : LOW);
    }
    else
    {
        digitalWrite(_ssrPin, LOW);
    }

    if (_input < _setpoint + _toleranceMin)
        status = TempStatus::BELOW_RANGE;
    else if (_input > _setpoint + _toleranceMax)
        status = TempStatus::ABOVE_RANGE;
    else
        status = TempStatus::WITHIN_RANGE;

    return status;
}

double SingleHeaterController::getTemperature() const
{
    return _input;
}

double SingleHeaterController::getOutput() const
{
    return _output;
}

void SingleHeaterController::setSetpoint(double setpoint)
{
    _setpoint = setpoint;
}

void SingleHeaterController::setTempTolerance(double minOffset, double maxOffset)
{
    _toleranceMin = minOffset;
    _toleranceMax = maxOffset;
}

void SingleHeaterController::setTunings(double Kp, double Ki, double Kd, int POn)
{
    _pid.SetTunings(Kp, Ki, Kd, POn);
}

void SingleHeaterController::setResistorValue(double resitorValue)
{
    _dividerResistor = resitorValue;
}

void SingleHeaterController::setWindowSize(unsigned long windowSize)
{
    _windowSize = windowSize;
}

void SingleHeaterController::setADCResoultion(double resolution)
{
    _adcResoulution = resolution;
}

void SingleHeaterController::outputEnable(bool onOff)
{
    _outputEnabled = onOff;
}

bool SingleHeaterController::isEnabled(void)
{
    return _outputEnabled;
}

// Steinhart-Hart approximation for 100k NTC thermistor, B=3950
double SingleHeaterController::readTemperature(int pin)
{
    // Read analog value from the specified pin (12-bit ADC: 0–4095)
    int adcValue = analogRead(pin);

    if (adcValue <= 0 || adcValue >= _adcResoulution)
    {
        return -273.15; // Error
    }

    // Calculate thermistor resistance based on voltage divider formula
    double resistance = _dividerResistor / (_adcResoulution / adcValue - 1.0);

    // Apply simplified Steinhart-Hart equation
    double steinhart;
    steinhart = resistance / _dividerResistor; // R / R₀ ( at 25°C)
    steinhart = log(steinhart);                // Natural logarithm of (R/R₀)
    steinhart /= _bCoefficient;                // Divide by B coefficient
    steinhart += 1.0 / (25.0 + 273.15);        // Add inverse of reference temp (25°C in Kelvin)
    steinhart = 1.0 / steinhart;               // Take inverse to get absolute temperature in Kelvin

    return steinhart - 273.15; // Convert Kelvin to Celsius
}