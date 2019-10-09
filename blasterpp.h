/*
 * blaster++ Flexibla DMA transfers to GPIO for the Raspberry Pi
 *
 * Copyright (c) 2019 Arno Rehn <arno@arnorehn.de
 * Copyright (c) 2013 Thomas Sarlandie <thomas@sarlandie.net>
 * Copyright (c) 2013 Richard Hirst <richardghirst@gmail.com>
 *
 * Based on the pi-blaster fork of servod.c by Richard Hirst.
 */
/*
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

#ifndef BLASTERPP_H
#define BLASTERPP_H

#include <chrono>
#include <vector>
#include <memory>
#include "span.hpp"

namespace BlasterPP {

enum GpioMode {
    ModeInput = 0,
    ModeOutput = 1
};

void setGpioMode(unsigned int pin, GpioMode mode);
bool gpioValue(unsigned int pin);
std::uint32_t gpioValues();
void setGpioValue(unsigned int pin, bool value);

struct vc_mem;

struct dma_cb_t {
  uint32_t info, src, dst, length,
     stride, next, pad[2];
};

struct Control
{
    tcb::span<uint32_t> sample;
    tcb::span<dma_cb_t> cb;
};

/**
 * @brief DmaChannel provides the functionality to push data to the GPIOs
 *        via the Raspberry Pi's DMA engine
 *
 * Note that this does not install any signal handlers. You should make sure
 * that all destructors run and that the app exits cleanly.
 *
 * The lowest sample period is 1 microsecond, giving a GPIO update frequency
 * of 1 MHz. With tighter timing settings one might reach a very unstable
 * 1.5 MHz output frequency on the GPIOs. I didn't manager to get it any
 * faster, though. 1 MHz seems to be a limit for stable operation. This class
 * enforces the maximum data rate of 1 MHz.
 */
class DmaChannel
{
public:
    enum DelayHardware {
        DelayViaPwm,
        DelayViaPcm,
    };

    enum LoopMode {
        Loop,
        SingleShot
    };

    explicit DmaChannel();

    /**
     * @brief Constructor which already configures the channel
     *
     * @sa reconfigure()
     */
    explicit DmaChannel(unsigned int channelNumber, unsigned int sampleCount,
                        const std::chrono::nanoseconds &sampleTime,
                        unsigned int subchannelCount,
                        DelayHardware delayHardware = DelayViaPwm,
                        LoopMode loopMode = Loop);
    ~DmaChannel();

    /// @brief Resets the DMA channel (i.e. turns it off)
    void reset();

    /// @brief (Re-)starts DMA
    bool restart();

    /// @brief Alias for restart()
    bool start() { return restart(); }

    /**
     * @brief Reconfigures the DMA channel
     *
     * Keep in mind that the maximum stable output frequency is about 1 MHz
     * (i.e. 1 us sample time). When you have more than one subchannel,
     * the minimum sample time that gives stable operation increases accordingly:
     * 2 subchannels => 2 us, 3 subchannels => 3 us etc.
     *
     * Resolution of the sample time is 100 ns, rounded down (i.e when you
     * specify 1150 ns, it gets rounded down to 1100 ns).
     */
    void reconfigure(unsigned int channelNumber, unsigned int sampleCount,
                     std::chrono::nanoseconds sampleTime,
                     unsigned int subchannelCount = 1,
                     DelayHardware delayHardware = DelayViaPwm,
                     LoopMode loopMode = Loop);

    /// @brief The DMA channel number
    unsigned int channelNumber() const { return m_channelNumber; }

    /// @brief The number of samples per subchannel
    unsigned int sampleCount() const;

    ///  @brief The number of subchannels
    unsigned int subchannelCount() const { return m_patterns.size(); }

    /**
     * @brief The on/off pattern as a boolean vector
     *
     * The size of the vector should match the number of samples. It is resized
     * to that number, potentially dropping or appending elements.
     */
    void setPattern(unsigned int subChannel, std::vector<bool> pattern);

    /**
     * @brief sets a PWM pattern on the DMA channel
     *
     * A PWM pattern in its most basic form is "first sample on, all others off".
     * By settings the @p frequencyMultiplier larger than 1, the frequency can
     * be increased by setting more samples to "on". I.e. for
     * frequencyMultiplier = 2, the first and the "middle" sample are "on",
     * all others are off.
     *
     * Using this method you can output different PWM frequencies on a number
     * of pins while only using a single DMA channel.
     *
     * @sa setPulseWidth()
     */
    void setPwmPattern(unsigned int subChannel, unsigned int frequencyMultiplier = 1);

    /**
     * @brief Sets the pulse width on @p pin to @p length, optionally honoring
     * the frequency multiplier @p mult.
     *
     * The multiplier you specify here has to be an integer divisor of the
     * multiplier you've specified in setPwmPattern(). If the pattern multiplier
     * is 8, you can use 8, 4, 2 and 1 as multipliers in this method.
     *
     * @sa setPwmPattern()
     */
    void setPulseWidth(unsigned int subChannel, unsigned int pin,
                       const std::chrono::microseconds &length, unsigned int mult = 1);

    /**
     * @brief Convenience method to set a duty cycle @p duty on @pin
     *
     * Calculates the correct pulse width and calls setPulseWidth().
     */
    void setPwmDutyCycle(unsigned int subChannel, unsigned int pin,
                         float duty, unsigned int mult = 1);

    /// @brief Returns the current output on/off pattern
    std::vector<bool> pattern(unsigned int subChannel) const;

    /// @brief Returns the sample time
    std::chrono::nanoseconds sampleTime() const { return m_sampleTime; }

    /// @brief Returns the full cycle time
    std::chrono::nanoseconds cycleTime() const { return sampleCount() * sampleTime(); }

    /// @brief Returns the cycle frequency
    double cycleFrequency() const {
        using namespace std::chrono;
        return 1.0 / duration_cast<duration<double>>(cycleTime()).count();
    }

    /**
     * @brief Returns the samples that are written to the GPIOs
     *
     * Each bit in a sample corresponds to a pin. If the pin-bit is set, the
     * pattern's on/off value will be written to the corresponding pin.
     */
    tcb::span<uint32_t> samples(unsigned int subChannel);

    /**
     * @brief Returns all the samples that are written to the GPIOs
     *
     * The subchannels are laid out one after the other in the returned span.
     */
    tcb::span<uint32_t> allSamples();

    int currentSampleIndex() const;

private:
    void init_hardware();

    unsigned int controlBlockCount() const;
    unsigned int controlBlockStride() const;
    unsigned int pageCount() const;

    volatile uint32_t *dma_reg = nullptr;  // pointer to the DMA Channel registers we are using

    unsigned int m_channelNumber = 0;
    std::vector<std::vector<bool>> m_patterns;
    std::chrono::nanoseconds m_sampleTime;
    std::unique_ptr<vc_mem> m_vcMem;
    Control m_ctl;
    DelayHardware m_delayHardware;
    LoopMode m_loopMode = Loop;
};

} // namespace BlasterPP

#endif // BLASTERPP_H
