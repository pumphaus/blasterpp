#include <chrono>
#include <iostream>
#include <fstream>
#include <sstream>
#include "blasterpp.h"

#include <unistd.h>
#include <signal.h>

constexpr unsigned int FreqMult = 8;
volatile bool should_quit = false;

void terminate(int)
{
    should_quit = true;
}

void setPinToOutput(int pin)
{
    {
        std::ofstream exportFile("/sys/class/gpio/export");
        exportFile << pin << std::endl;
    }

    {
        std::stringstream gpioDirectionFileName;
        gpioDirectionFileName << "/sys/class/gpio/gpio" << pin << "/direction";
        std::ofstream gpioDirectionFile(gpioDirectionFileName.str());
        gpioDirectionFile << "out" << std::endl;
    }
}

int main(int argc, char **argv)
{
    using namespace std::chrono_literals;

    setPinToOutput(17);
    setPinToOutput(22);
    setPinToOutput(27);

    // Properly quit if possible
    signal(SIGHUP, terminate);
    signal(SIGQUIT, terminate);
    signal(SIGTERM, terminate);
    signal(SIGKILL, terminate);
    signal(SIGABRT, terminate);

    // 20'000 samples * 1us each = 20'000 us = 20 ms cycle time (i.e. 50 Hz)
    BlasterPP::DmaChannel channel(14, 20000, 1us);

    // Set a PWM pattern for a maximum frequency of FreqMult * cycleTime
    // (i.e. 8 * 50 Hz = 400 Hz in this case)
    channel.setPwmPattern(FreqMult);

    // 20 % duty cycle on pin 17 @ 50 Hz
    channel.setPwmDutyCycle(17, 0.2, 1 /* no multiplier */);

    // 30 % duty cycle on pin 27 @ 200 Hz
    channel.setPwmDutyCycle(27, 0.3, FreqMult / 2);

    // 40 % duty cycle on pin 22 @ 400 Hz
    channel.setPwmDutyCycle(22, 0.4, FreqMult);

    std::cout << "DMA channel cycle frequency is " << channel.cycleFrequency()
              << " Hz" << std::endl;
    std::cout << "Frequency multiplier is " << FreqMult
              << " for a maximum PWM frequency of "
              << (channel.cycleFrequency() * FreqMult) << " Hz" << std::endl;

    channel.start();

    while (!should_quit) {
        usleep(1000);
    }

    std::cout << "Exiting." << std::endl;
}
