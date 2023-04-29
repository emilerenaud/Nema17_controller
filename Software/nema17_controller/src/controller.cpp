#include <controller.h>


Controller::Controller()
{
    _stepper = new AccelStepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
    _encoder = new AS5600();
    _currentSensor = new Adafruit_INA219();

}

void Controller::init()
{
    Wire.begin(SDA_PIN, SCL_PIN);

    _encoder->begin();
    if(!_encoder->isConnected())
    {
        Serial.println("Encoder not connected");
    }
    
    if(!_currentSensor->begin())
    {
        Serial.println("Current sensor not connected");
    }

    _stepper->setEnablePin(EN_PIN);
    _stepper->setPinsInverted(false, false, true);
    _stepper->setMaxSpeed(5000.0);
    _stepper->setAcceleration(10000.0);
    _stepper->disableOutputs();

}

void Controller::disableStepper()
{
    
    _stepper->disableOutputs();
}

void Controller::enableStepper()
{
    _stepper->enableOutputs();
}


void Controller::moveAngle(int angle)
{
    // _stepper->moveTo(position);
}

void Controller::moveStep(long steps)
{
    _stepper->move(steps);
}

void Controller::setSpeed(int speed)
{
    _stepper->setMaxSpeed(speed);
}

void Controller::setAcceleration(int acceleration)
{
    _stepper->setAcceleration(acceleration);
}

void Controller::run()
{
    if(_stepper->run() == false)
    {
        // Serial.println("Stepper stopped");
        disableStepper();
    }
    else
    {
        // Serial.println("Stepper running");
        enableStepper();
    }
}

float Controller::getCurrent()
{
    return _currentSensor->getCurrent_mA();
}

float Controller::getVoltage()
{
    return _currentSensor->getBusVoltage_V();
}

float Controller::getPower()
{
    return _currentSensor->getPower_mW();
}

void Controller::enableClosedLoop()
{
    _closedLoop = true;
}

void Controller::disableClosedLoop()
{
    _closedLoop = false;
}

float Controller::getAngle()
{
    // return encoder->getAngle();
}

float Controller::getPosition()
{
    // return encoder->getPosition();
}