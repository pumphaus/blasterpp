#include <chrono>
#include <iostream>
#include <random>

#include <unistd.h>
#include <signal.h>

#include "blasterpp.h"

constexpr int ServoPin = 21;

volatile bool should_quit = false;

constexpr int MinWidth = 500;
constexpr int MaxWidth = 2000;

void terminate(int)
{
    should_quit = true;
}

int main(int argc, char **argv)
{
    using namespace std::chrono_literals;
    using namespace BlasterPP;

    setGpioMode(ServoPin, ModeOutput);

    // Properly quit if possible
    signal(SIGHUP, terminate);
    signal(SIGQUIT, terminate);
    signal(SIGTERM, terminate);
    signal(SIGKILL, terminate);
    signal(SIGABRT, terminate);

    // 10'000 samples * 2us each = 20'000 us = 20 ms cycle time (i.e. 50 Hz),
    DmaChannel channel(14, 10000, 2us, 1);

    // PWM on both subchannels
    channel.setPwmPattern(0);

    channel.start();

    // Erratically move the servo
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dist(MinWidth, MaxWidth);

    for (int i = 0; i < 100; ++i) {
        channel.setPulseWidth(0, ServoPin, dist(gen) * 1us);
        usleep(.05e6);
    }

    std::cout << "Exiting." << std::endl;
}
