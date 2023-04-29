#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include "AS5600.h"
#include "Adafruit_INA219.h"
#include "AccelStepper.h"
#include "PCB_config.h"

// create class for the controller
class Controller
{
    public:
    // Construction with 3 parameters, stepper, encoder and current sensor
    Controller();

    // Variables
    AccelStepper* _stepper;
    AS5600* _encoder;
    Adafruit_INA219* _currentSensor;

    void init();

    void disableStepper();
    void enableStepper();

    void moveAngle(int angle);
    void moveStep(long steps);
    void setSpeed(int speed);
    void setAcceleration(int acceleration);

    void run();

    float getCurrent();
    float getVoltage();
    float getPower();

    void enableClosedLoop();
    void disableClosedLoop();
    float getAngle();
    float getPosition();



    private:

    // Variables
    bool _closedLoop = false;

    long _currentPosition = 0;
    long _targetPosition = 0;
};

#endif
