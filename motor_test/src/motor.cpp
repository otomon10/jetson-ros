#include <math.h>
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

double Motor::getSpeed()
{
    return speed;
}

void Motor::run(double speed)
{
    this->speed = speed;

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
    while(true)
    {
        // low
        in1->write(0);
        in2->write(0);
        double abs_speed = abs(speed);
        int wait_time = static_cast<int>((1.0 - abs_speed) * 10 * 1000);
        usleep(wait_time);

        // high
        if(0 < speed){
            // forward
            in1->write(1);
        }
        else if(0 > speed) {
            // reverse
            in2->write(1);
        }
        usleep(1000);
    }
}
