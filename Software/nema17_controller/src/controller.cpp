#include <controller.h>


Controller::Controller()
{
    _stepper = new AccelStepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
    _encoder = new AS5600();
    _currentSensor = new Adafruit_INA219();

}

void Controller::init()
{
    Serial.begin(115200);

    Wire.begin(SDA_PIN, SCL_PIN);

    _encoder->begin();
    delay(100);
    Serial.println("Encoder magnet detected : " + String(_encoder->detectMagnet()));
    Serial.println("Encoder magnet too strong : " + String(_encoder->magnetTooStrong()));
    Serial.println("Encoder magnet too weak : " + String(_encoder->magnetTooWeak()));
    
    _encoder->setDirection(AS5600_CLOCK_WISE);
    setOffsetAngle(getAngle());
    _currentAngle = 0;
    if(!_encoder->isConnected())
    {
        Serial.println("Encoder not connected");
    }
    
    if(!_currentSensor->begin())
    {
        Serial.println("Current sensor not connected");
    }

    _closedLoop = true;

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


void Controller::moveAngle(float angle)
{
    // convert angle to steps
    long steps = (abs(angle) * _microstepping * _stepsPerRevolution) / 360.0;
    if(angle > 0)
    {
        _stepper->move(steps);
    }
    else
    {
        _stepper->move(-steps);
    }
}

void Controller::moveToAngle(float angle)
{
    if(_closedLoop)
    {
        // check if the motor is at the _currentAngle
        if(abs(_currentAngle - getAngle()) < 3)
        {
            Serial.println("Motor at angle");
        }
        else
        {
            Serial.println("Motor not at angle");
            // _currentAngle = getAngle();
        }
        // calculate delta for the two sides and check which is smaller
        float deltaAngle = angle - _currentAngle;
        // Serial.println("Delta angle : " + String(deltaAngle));
        if(abs(deltaAngle) > 180)
        {
            if(deltaAngle > 0)
                deltaAngle = deltaAngle - 360;
            else 
                deltaAngle = deltaAngle + 360;
        }
        _currentAngle =  _currentAngle + deltaAngle;
        _deltaAngle = deltaAngle;
        moveAngle(deltaAngle); 
        Serial.println("Closed loop enabled");
    }
    else
    {
        // calculate delta
        float deltaAngle = angle - _currentAngle;
        moveAngle(deltaAngle); 
        Serial.println("Closed loop disabled");
    }
}

void Controller::moveStep(long steps)
{
    _stepper->moveTo(steps);
}

void Controller::moveToStep(long steps)
{
    // if(_closedLoop)
    // {
    //     _stepper->moveTo(steps);

    // }
    // else
    // {
    //     _stepper->moveTo(steps);
    //     _currentPosition = _currentPosition + steps;
    //     // Serial.println("Closed loop disabled");
    // }
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
    // if(_deltaAngle != 0)
    // {
    //     moveAngle(_deltaAngle);
    //     // _deltaAngle = 0;
    // }

    if(_stepper->run() == false)
    {
        // Serial.println("Stepper stopped");
        disableStepper();
    }
    else
    {
        // Serial.println("Stepper running");
        enableStepper();

        // Serial.println("Angle : " + String(getAngle()));
    }

    // debug encoder
    if(_debug)
    {
        if(_lastLoopTime - millis() > 750)
        {
            Serial.println("Angle : " + String(getAngle()));
            // Serial.println("Position : " + String(getPosition()));
            // Serial.println("Current : " + String(getCurrent()));
            // Serial.println("Voltage : " + String(getVoltage()));
            // Serial.println("Power : " + String(getPower()));
            _lastLoopTime = millis();
        }
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
    // translate raw angle to degrees
    uint16_t rawAngle = _encoder->readAngle();
    float angle = (rawAngle * 360.0) / 4096.0;

    if(_offsetAngle > 0) angle = (angle - _offsetAngle);
    if(angle < 0) angle = 360.0 + angle;
    return angle;
    // return encoder->getAngle();
}

void Controller::setOffsetAngle(float angle)
{
    _offsetAngle = angle;
}

void Controller::resetEncoder()
{
    uint16_t rawAngle = _encoder->readAngle();
    float angle = (rawAngle * 360.0) / 4096.0;
    setOffsetAngle(angle);
    _currentAngle = 0;
}

float Controller::getPosition()
{
    // return encoder->getPosition();
}

void Controller::enableDebug()
{
    _debug = true;
}

void Controller::disableDebug()
{
    _debug = false;
}