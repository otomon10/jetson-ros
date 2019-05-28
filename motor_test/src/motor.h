#ifndef MOTOR_H
#define MOTOR_H

#include <thread>
#include "gpio.h"

class Motor
{
public:
    enum MotorState{
        Stop,
        RunningForward,
        RunningReverse
    };
    explicit Motor(int pin1, int pin2);
    ~Motor();
    void run(double speed); // -1.0 ~ 1.0
    void stop();
    MotorState getState();
    double getSpeed();

private:
    void motorRunning();
    double speed;
    MotorState state;
    Gpio *in1;
    Gpio *in2;
    std::thread th;
};

#endif // MOTOR_H
