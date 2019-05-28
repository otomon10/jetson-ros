#ifndef GPIO_H
#define GPIO_H

#include <string>
#include <fstream>

class Gpio
{

public:
    enum Direction{
        In,
        Out
    };
    explicit Gpio(int pin, Direction dir);
    ~Gpio();
    void write(int value);
    int read();

private:
    int pin;
    Direction direction;
};

#endif // GPIO_H
