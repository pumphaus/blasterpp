/*
 * This example shows how to read the RCW-0001 ultrasonic distance sensor with
 * BlasterPP.
 *
 * The sensor works by initiating an ultrasonic measurement via a short HIGH
 * level signal on the trigger pin. When the ultrasonic echo is received, the
 * sensor emits a HIGH pulse on the echo pin. The length of the echo pulse is
 * the time it took to detect the echo. The distance can be easily calculated
 * from this time-of-flight measurement by multiplying it with the speed of
 * sound and dividing it by two.
 *
 * The sensor has a maximum range of about 450 cm, which would make for a
 * maximum time-of-flight of about 27 ms. We're being generous and provide a
 * 40 ms input buffer (20'000 samples at 2µs each).
 *
 * Copyright (c) 2019 Arno Rehn <arno@arnorehn.de>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

#include "blasterpp.h"
#include <iostream>
#include <algorithm>
#include <chrono>
#include <thread>

constexpr unsigned int TriggerPin = 17;
constexpr unsigned int EchoPin = 27;

constexpr double SpeedOfSound = 340.27;  // meters per second

int main(int argc, char **argv)
{
    using namespace BlasterPP;
    using namespace std;
    using namespace std::chrono;
    using namespace std::chrono_literals;

    setGpioMode(TriggerPin, ModeOutput);
    setGpioMode(EchoPin, ModeInput);

    DmaChannel channel(14,    // use DMA channel 14
                       20000, // 20000 samples @ 2us per sample = 40ms cycle time
                       2us,   // duration of one sample
                       2,     // two subchannels
                       1,     // subchannel 1 is the input channel (0 is output)
                       DmaChannel::DelayViaPwm,
                       DmaChannel::SingleShot);

    // Emit a 10us pulse on pin 17 at start. This is the trigger pulse.
    channel.setPwmPattern(0);
    channel.setPulseWidth(0, TriggerPin, 10us);

    channel.start();

    // Wait for the acquisition to complete
    while (!channel.atEnd()) {
        this_thread::sleep_for(1ms);
    }

    // GPIO Mask for pin 27 (where we get the echo)
    constexpr int Mask = (1 << EchoPin);
    const auto samps = channel.samples(1);  // input samples

    auto rising_it = adjacent_find(samps.begin(), samps.end(), [](int a, int b) {
        return !(a & Mask) && (b & Mask);
    });
    auto falling_it = adjacent_find(samps.begin(), samps.end(), [](int a, int b) {
        return (a & Mask) && !(b & Mask);
    });

    const auto time = distance(rising_it, falling_it) * channel.sampleTime();

    std::cout << "Got echo after "
              << duration_cast<nanoseconds>(time).count() / 1000.0 << " µs"
              << std::endl;

    const double distance =
            duration_cast<duration<double>>(time).count() * SpeedOfSound / 2.0;

    std::cout << "Distance: " << distance * 100 << " cm" << std::endl;
}
