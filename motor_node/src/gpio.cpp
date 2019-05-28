#include "gpio.h"
#include <iostream>

Gpio::Gpio(int pin, Direction dir) :
    pin(pin),
    direction(dir)
{
}

Gpio::~Gpio()
{
}

void Gpio::write(int value)
{
    std::string fname1 = "/sys/class/gpio/gpio";
    std::string fname2 = "/value";
    std::ofstream fout(fname1 + std::to_string(pin) +fname2);

    fout << std::to_string(value);
}

int Gpio::read()
{
    std::string fname1 = "/sys/class/gpio/gpio";
    std::string fname2 = "/value";
    std::ifstream fin(fname1 + std::to_string(pin) +fname2);

    std::string str;
    fin >> str;
    return atoi(str.c_str());
}
