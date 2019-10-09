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
    setPinToOutput(27);

    // Properly quit if possible
    signal(SIGHUP, terminate);
    signal(SIGQUIT, terminate);
    signal(SIGTERM, terminate);
    signal(SIGKILL, terminate);
    signal(SIGABRT, terminate);

    // 10'000 samples * 2us each = 20'000 us = 20 ms cycle time (i.e. 50 Hz),
    // 2 subchannels
    BlasterPP::DmaChannel channel(14, 10000, 2us, 2);

    // PWM on both subchannels
    channel.setPwmPattern(0);
    channel.setPwmPattern(1, FreqMult);

    // 50% duty cycle on both subchannels, but one is upmixed to 8x the frequency
    channel.setPwmDutyCycle(0, 17, 0.5);
    channel.setPwmDutyCycle(1, 27, 0.5, FreqMult);

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
