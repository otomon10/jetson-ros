#include <cmath>
#include "unistd.h"
#include "motor.h"

Motor::Motor(int pin1, int pin2) :
    speed(0.0),
    state(Stop),
    in1(new Gpio(pin1, Gpio::Out)),
    in2(new Gpio(pin2, Gpio::Out)),
    th(&Motor::motorRunning, this)
{
    th.detach();
}

Motor::~Motor()
{
    delete in1;
    delete in2;
}

Motor::MotorState Motor::getState()
{
    return state;
}

float Motor::getSpeed()
{
    return speed;
}

void Motor::run(float speed)
{
    if(std::fabs(speed) > 1.0f){
        this->speed = speed / std::fabs(speed);
    }
    else {
        this->speed = speed;
    }

    if(0 < speed){
        state = RunningForward;
    }
    else if(0 > speed) {
        state = RunningReverse;
    }
    else {
        state = Stop;
    }
}

void Motor::stop()
{
    this->speed = 0.0;
}

void Motor::motorRunning()
{
    const float Tus = 20000;   // T = 20 [ms]

    while(true)
    {
        float duty = std::fabs(speed);

        // low
        in1->write(0);
        in2->write(0);
        usleep(static_cast<unsigned int>(Tus * (1.0f - duty)));

        // high
        if(0 < speed){
            // forward
            in1->write(1);
        }
        else if(0 > speed) {
            // reverse
            in2->write(1);
        }
        usleep(static_cast<unsigned int>(Tus * duty));
    }
}
