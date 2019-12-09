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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <getopt.h>
#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "mailbox.h"

#include "span.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <sstream>
#include <vector>

#include "blasterpp.h"

namespace BlasterPP {

#define DEVFILE_MBOX  "vc-mbox"
#define DEVFILE_VCIO  "/dev/vcio"

#define PAGE_SIZE     4096
#define PAGE_SHIFT    12

#define DMA_BASE        (periph_virt_base + 0x00007000)
#define DMA_CHAN_MAX    14
#define DMA_CHAN_SIZE	  0x100 /* size of register space for a single DMA channel */
#define PWM_BASE_OFFSET 0x0020C000
#define PWM_BASE        (periph_virt_base + PWM_BASE_OFFSET)
#define PWM_PHYS_BASE   (periph_phys_base + PWM_BASE_OFFSET)
#define PWM_LEN         0x28
#define CLK_BASE_OFFSET 0x00101000
#define CLK_BASE        (periph_virt_base + CLK_BASE_OFFSET)
#define CLK_LEN         0xA8
#define GPIO_BASE_OFFSET  0x00200000
#define GPIO_BASE       (periph_virt_base + GPIO_BASE_OFFSET)
#define GPIO_PHYS_BASE  (periph_phys_base + GPIO_BASE_OFFSET)
#define GPIO_LEN        0x100
#define PCM_BASE_OFFSET 0x00203000
#define PCM_BASE        (periph_virt_base + PCM_BASE_OFFSET)
#define PCM_PHYS_BASE   (periph_phys_base + PCM_BASE_OFFSET)
#define PCM_LEN         0x24

#define DMA_NO_WIDE_BURSTS  (1<<26)
#define DMA_WAIT_RESP       (1<<3)
#define DMA_D_DREQ          (1<<6)
#define DMA_PER_MAP(x)      ((x)<<16)
#define DMA_END             (1<<1)
#define DMA_RESET           (1<<31)
#define DMA_INT             (1<<2)

#define DMA_CS              (0x00/4)
#define DMA_CONBLK_AD       (0x04/4)
#define DMA_DEBUG           (0x20/4)

#define GPIO_FSEL0          (0x00/4)
#define GPIO_SET0           (0x1c/4)
#define GPIO_CLR0           (0x28/4)
#define GPIO_LEV0           (0x34/4)
#define GPIO_PULLEN         (0x94/4)
#define GPIO_PULLCLK        (0x98/4)

#define GPIO_MODE_IN        0
#define GPIO_MODE_OUT       1

#define PWM_CTL     (0x00/4)
#define PWM_DMAC    (0x08/4)
#define PWM_RNG1    (0x10/4)
#define PWM_FIFO    (0x18/4)

#define PWMCLK_CNTL  40
#define PWMCLK_DIV   41

#define PWMCTL_MODE1  (1<<1)
#define PWMCTL_PWEN1  (1<<0)
#define PWMCTL_CLRF   (1<<6)
#define PWMCTL_USEF1  (1<<5)

#define PWMDMAC_ENAB    (1<<31)
#define PWMDMAC_THRSHLD ((15<<8)|(15<<0))

#define PCM_CS_A      (0x00/4)
#define PCM_FIFO_A    (0x04/4)
#define PCM_MODE_A    (0x08/4)
#define PCM_RXC_A     (0x0c/4)
#define PCM_TXC_A     (0x10/4)
#define PCM_DREQ_A    (0x14/4)
#define PCM_INTEN_A   (0x18/4)
#define PCM_INT_STC_A (0x1c/4)
#define PCM_GRAY      (0x20/4)

#define PCMCLK_CNTL   38
#define PCMCLK_DIV    39

/* New Board Revision format:
SRRR MMMM PPPP TTTT TTTT VVVV

S scheme (0=old, 1=new)
R RAM (0=256, 1=512, 2=1024)
M manufacturer (0='SONY',1='EGOMAN',2='EMBEST',3='UNKNOWN',4='EMBEST')
P processor (0=2835, 1=2836)
T type (0='A', 1='B', 2='A+', 3='B+', 4='Pi 2 B', 5='Alpha', 6='Compute Module')
V revision (0-15)
*/
#define BOARD_REVISION_SCHEME_MASK (0x1 << 23)
#define BOARD_REVISION_SCHEME_OLD (0x0 << 23)
#define BOARD_REVISION_SCHEME_NEW (0x1 << 23)
#define BOARD_REVISION_RAM_MASK (0x7 << 20)
#define BOARD_REVISION_MANUFACTURER_MASK (0xF << 16)
#define BOARD_REVISION_MANUFACTURER_SONY (0 << 16)
#define BOARD_REVISION_MANUFACTURER_EGOMAN (1 << 16)
#define BOARD_REVISION_MANUFACTURER_EMBEST (2 << 16)
#define BOARD_REVISION_MANUFACTURER_UNKNOWN (3 << 16)
#define BOARD_REVISION_MANUFACTURER_EMBEST2 (4 << 16)
#define BOARD_REVISION_PROCESSOR_MASK (0xF << 12)
#define BOARD_REVISION_PROCESSOR_2835 (0 << 12)
#define BOARD_REVISION_PROCESSOR_2836 (1 << 12)
#define BOARD_REVISION_TYPE_MASK (0xFF << 4)
#define BOARD_REVISION_TYPE_PI1_A (0 << 4)
#define BOARD_REVISION_TYPE_PI1_B (1 << 4)
#define BOARD_REVISION_TYPE_PI1_A_PLUS (2 << 4)
#define BOARD_REVISION_TYPE_PI1_B_PLUS (3 << 4)
#define BOARD_REVISION_TYPE_PI2_B (4 << 4)
#define BOARD_REVISION_TYPE_ALPHA (5 << 4)
#define BOARD_REVISION_TYPE_PI3_B (8 << 4)
#define BOARD_REVISION_TYPE_PI3_BP (0xD << 4)
#define BOARD_REVISION_TYPE_PI3_AP (0x7 << 5)
#define BOARD_REVISION_TYPE_CM (6 << 4)
#define BOARD_REVISION_TYPE_CM3 (10 << 4)
#define BOARD_REVISION_REV_MASK (0xF)

#define BUS_TO_PHYS(x) ((x)&~0xC0000000)

static uint32_t periph_virt_base;
static uint32_t periph_phys_base;
static uint32_t mem_flag;

static volatile uint32_t *pwm_reg = nullptr;
static volatile uint32_t *pcm_reg = nullptr;
static volatile uint32_t *clk_reg = nullptr;
static volatile uint32_t *dma_virt_base = nullptr; /* base address of all DMA Channels */
static volatile uint32_t *gpio_reg = nullptr;

static void static_initialize();

// open a char device file used for communicating with kernel mbox driver
static int mbox_open() {
  // try to use /dev/vcio first (kernel 4.1+)
  int file_desc = open(DEVFILE_VCIO, 0);
  if (file_desc < 0) {
    /* initialize mbox */
    unlink(DEVFILE_MBOX);
    if (mknod(DEVFILE_MBOX, S_IFCHR|0600, makedev(MAJOR_NUM, 0)) < 0) {
        std::cerr << "Failed to create mailbox device!" << std::endl;
        return 0;
    }
    file_desc = open(DEVFILE_MBOX, 0);
    if (file_desc < 0) {
        std::cerr << "Can't open device file: " << DEVFILE_MBOX << std::endl;
        return 0;
    }
  }
  return file_desc;
}

static void mbox_close(int file_desc) {
  close(file_desc);
}

void setGpioMode(unsigned int pin, GpioMode mode)
{
    static_initialize();
    uint32_t fsel = gpio_reg[GPIO_FSEL0 + pin/10];

    fsel &= ~(7 << ((pin % 10) * 3));
    fsel |= mode << ((pin % 10) * 3);
    gpio_reg[GPIO_FSEL0 + pin/10] = fsel;
}

bool gpioValue(unsigned int pin)
{
    static_initialize();

    return gpio_reg[GPIO_LEV0] & (1 << pin);
}

uint32_t gpioValues()
{
    static_initialize();
    return gpio_reg[GPIO_LEV0];
}

void setGpioValue(unsigned int pin, bool value)
{
    static_initialize();

    if (value) {
        gpio_reg[GPIO_SET0] = 1 << pin;
    } else {
        gpio_reg[GPIO_CLR0] = 1 << pin;
    }
}


static void udelay(int us) {
  struct timespec ts = { 0, us * 1000 };

  nanosleep(&ts, NULL);
}

struct SafeMboxFile
{
    bool initialized = false;
    int fd = -1;

    ~SafeMboxFile() {
        mbox_close(fd);
    }

    void init() {
        if (initialized) {
            return;
        }

        fd = mbox_open();
        if (fd < 0) {
            throw std::runtime_error("Failed to open mbox!");
        }
        initialized = true;
    }

    operator int() {
        init();
        return fd;
    }
};

static SafeMboxFile mbox_handle;

struct vc_mem {
    vc_mem(unsigned int num_pages) : num_pages(num_pages)
    {
        std::cout << "Allocating " << num_pages * PAGE_SIZE << " of memory" << std::endl;
        /* Use the mailbox interface to the VC to ask for physical memory */
        mem_ref = mem_alloc(mbox_handle, num_pages * PAGE_SIZE, PAGE_SIZE, mem_flag);
        /* TODO: How do we know that succeeded? */
        std::cout << "mem_ref " << mem_ref << std::endl;
        bus_addr = mem_lock(mbox_handle, mem_ref);
        std::cout << "bus_addr " << (void*)bus_addr << std::endl;
        virt_addr = (uint8_t*) mapmem(BUS_TO_PHYS(bus_addr), num_pages * PAGE_SIZE);
        std::cout << "virt_addr " << (void*)virt_addr << std::endl;

        if ((unsigned long)virt_addr & (PAGE_SIZE-1)) {
          std::cerr << "vc_mem: Virtual address is not page aligned!" << std::endl;
        }
    }

    ~vc_mem() {
        if (virt_addr) {
          unmapmem(virt_addr, num_pages * PAGE_SIZE);
          mem_unlock(mbox_handle, mem_ref);
          mem_free(mbox_handle, mem_ref);
        }
    }

    uint32_t virt_to_phys(void *virt) {
      uint32_t offset = (uint8_t *)virt - virt_addr;

      return bus_addr + offset;
    }

    void *phys_to_virt(uint32_t phys) {
        return virt_addr + (phys - bus_addr);
    }

    unsigned int mem_ref = 0;	/* From mem_alloc() */
    unsigned int bus_addr = 0;	/* From mem_lock() */
    uint8_t *virt_addr = nullptr;	/* From mapmem() */
    unsigned int num_pages = 0;
};

/*
 * determine which pi model we are running on
 */
static void get_model(unsigned mbox_board_rev) {
  int board_model = 0;

  if ((mbox_board_rev & BOARD_REVISION_SCHEME_MASK) == BOARD_REVISION_SCHEME_NEW) {
      printf("board rev: %d, type: %d\n", mbox_board_rev, mbox_board_rev & BOARD_REVISION_TYPE_MASK);
    if ((mbox_board_rev & BOARD_REVISION_TYPE_MASK) == BOARD_REVISION_TYPE_PI2_B) {
      board_model = 2;
    } else if ((mbox_board_rev & BOARD_REVISION_TYPE_MASK) == BOARD_REVISION_TYPE_PI3_B) {
      board_model = 3;
    } else if ((mbox_board_rev & BOARD_REVISION_TYPE_MASK) == BOARD_REVISION_TYPE_PI3_BP) {
      board_model = 3;
    } else if ((mbox_board_rev & BOARD_REVISION_TYPE_MASK) == BOARD_REVISION_TYPE_PI3_AP) {
      board_model = 3;
    } else if ((mbox_board_rev & BOARD_REVISION_TYPE_MASK) == BOARD_REVISION_TYPE_CM3) {
      board_model = 3;
    } else {
      // no Pi 2, we assume a Pi 1
      board_model = 1;
    }
  } else {
    // if revision scheme is old, we assume a Pi 1
    board_model = 1;
  }

  switch(board_model) {
    case 1:
      periph_virt_base = 0x20000000;
      periph_phys_base = 0x7e000000;
      mem_flag         = MEM_FLAG_L1_NONALLOCATING | MEM_FLAG_ZERO;
      break;
    case 2:
    case 3:
      periph_virt_base = 0x3f000000;
      periph_phys_base = 0x7e000000;
      mem_flag         = MEM_FLAG_L1_NONALLOCATING | MEM_FLAG_ZERO;
      break;
    default:
      std::ostream cerr(std::cerr.rdbuf());
      cerr << "Unable to detect Board Model from board revision: 0x" << std::hex << mbox_board_rev << std::endl;
      break;
  }
}

static void* map_peripheral(uint32_t base, uint32_t len) {
  int fd = open("/dev/mem", O_RDWR | O_SYNC);

  if (fd < 0) {
    throw std::runtime_error("blaster++: Failed to open /dev/mem!");
  }
  void *vaddr = mmap(nullptr, len, PROT_READ|PROT_WRITE, MAP_SHARED, fd, base);
  if (vaddr == MAP_FAILED) {
    throw std::runtime_error("blaster++: Failed to open /dev/mem!");
  }
  close(fd);

  return vaddr;
}

static void static_initialize()
{
    static bool initialized = false;
    if (initialized) {
        return;
    }

    std::ostream cout(std::cout.rdbuf());
    unsigned mbox_board_rev = get_board_revision(mbox_handle);
    cout << "MBox Board Revision: 0x" << std::hex << mbox_board_rev << std::endl;
    get_model(mbox_board_rev);
    unsigned mbox_dma_channels = get_dma_channels(mbox_handle);
    cout << "DMA Channels Info: 0x" << std::hex << mbox_dma_channels << std::endl
         << "DMA Base: 0x" << std::hex << DMA_BASE << std::endl;

    dma_virt_base = (volatile uint32_t*) map_peripheral(DMA_BASE, (DMA_CHAN_SIZE * (DMA_CHAN_MAX + 1)));
    pwm_reg = (volatile uint32_t*) map_peripheral(PWM_BASE, PWM_LEN);
    pcm_reg = (volatile uint32_t*) map_peripheral(PCM_BASE, PCM_LEN);
    clk_reg = (volatile uint32_t*) map_peripheral(CLK_BASE, CLK_LEN);
    gpio_reg = (volatile uint32_t*) map_peripheral(GPIO_BASE, GPIO_LEN);

    initialized = true;
}

DmaChannel::DmaChannel()
{

}

DmaChannel::DmaChannel(unsigned int channelNumber,
                       unsigned int sampleCount,
                       const std::chrono::nanoseconds &sampleTime,
                       unsigned int subchannelCount, int inputSubChannelIndex,
                       DelayHardware delayHardware, LoopMode loopMode)
{
    reconfigure(channelNumber, sampleCount, sampleTime, subchannelCount,
                inputSubChannelIndex, delayHardware, loopMode);
}

DmaChannel::~DmaChannel()
{
    reset();
}

void DmaChannel::reset()
{
    if (dma_reg) {
        dma_reg[DMA_CS] = DMA_RESET;
    }
}

bool DmaChannel::restart()
{
    if (!dma_reg) {
        return false;
    }

    // Initialise the DMA
    dma_reg[DMA_CS] = DMA_RESET;
    udelay(10);
    dma_reg[DMA_CS] = DMA_INT | DMA_END;
    dma_reg[DMA_CONBLK_AD] = m_vcMem->virt_to_phys(m_ctl.cb.begin());
    dma_reg[DMA_DEBUG] = 7; // clear debug error flags
    dma_reg[DMA_CS] = 0x10880001;	// go, mid priority, wait for outstanding writes

    if (m_delayHardware == DelayViaPcm) {
        pcm_reg[PCM_CS_A] |= 1<<2;			// Enable Tx
    }

    return true;
}

unsigned int DmaChannel::pageCount() const {
    return (controlBlockCount() * sizeof(dma_cb_t)
            + subchannelCount() * sampleCount() * sizeof(uint32_t) + PAGE_SIZE - 1) >> PAGE_SHIFT;
}

void DmaChannel::reconfigure(unsigned int channelNumber,
                             unsigned int sampleCount,
                             std::chrono::nanoseconds sampleTime,
                             unsigned int subchannelCount, int inputSubChannelIndex,
                             DelayHardware delayHardware, LoopMode loopMode)
{
    using namespace std::chrono_literals;

    static_initialize();

    reset();

    m_channelNumber = channelNumber;
    m_patterns.clear();
    m_patterns.resize(subchannelCount, std::vector<bool>(sampleCount));
    m_pulseWidths.resize(subchannelCount);
    std::fill(m_pulseWidths.begin(), m_pulseWidths.end(), 0);
    m_sampleTime = sampleTime;
    m_delayHardware = delayHardware;
    m_loopMode = loopMode;
    m_inputSubChannelIndex = inputSubChannelIndex;

    std::cout << "Initializing DMA channel " << m_channelNumber << std::endl
              << "  subchannels: " << this->subchannelCount() << std::endl
              << "  samples per channel: " << this->sampleCount() << std::endl;

    m_vcMem = std::make_unique<vc_mem>(pageCount());
    dma_reg = (volatile uint32_t*) dma_virt_base + m_channelNumber * (DMA_CHAN_SIZE / sizeof(dma_reg));

    if (!m_vcMem->virt_addr) {
        std::cerr << "Failed to get VC memory!" << std::endl;
        return;
    }

    m_ctl.sample = tcb::span<uint32_t>((uint32_t*) m_vcMem->virt_addr, this->sampleCount() * this->subchannelCount());
    m_ctl.cb = tcb::span<dma_cb_t>((dma_cb_t*) (m_vcMem->virt_addr + this->sampleCount() * this->subchannelCount() * sizeof(uint32_t)), controlBlockCount());

    const size_t cbStride = controlBlockStride();

    uint32_t phys_fifo_addr;
    const uint32_t phys_gpclr0 = GPIO_PHYS_BASE + 0x28;
    const uint32_t phys_gpset0 = GPIO_PHYS_BASE + 0x1c;
    const uint32_t phys_gplev0 = GPIO_PHYS_BASE + 0x34;

    uint32_t dmaFlags;

    if (m_delayHardware == DelayViaPwm) {
        phys_fifo_addr = PWM_PHYS_BASE + 0x18;
        dmaFlags = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_D_DREQ | DMA_PER_MAP(5);
    } else {
        phys_fifo_addr = PCM_PHYS_BASE + 0x04;
        dmaFlags = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_D_DREQ | DMA_PER_MAP(2);
    }
    std::fill(m_ctl.sample.begin(), m_ctl.sample.end(), 0);

    /* Initialize all the DMA commands. They come in batches.
     *  - The fist commands copy a value from the sample memory to a destination
     *    address which can be either the gpclr0 register or the gpset0 register
     *  - The last command waits for a trigger from an external source (PWM or PCM)
     */
    for (size_t i = 0; i < this->sampleCount(); ++i) {
        auto cbp = &m_ctl.cb[i * cbStride];

        // For each subchannel, copy a sample to the destination register. The
        // destination register can be changed in setPattern().
        // For input subchannels, the sample is copied from the GPIO level
        // register to the sample memory chunk.
        for (size_t j = 0; j < this->subchannelCount(); ++j) {
            cbp->info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP;
            if (int(j) == m_inputSubChannelIndex) {
                cbp->src = phys_gplev0;
                cbp->dst = m_vcMem->virt_to_phys(&samples(j)[i]);
            } else {
                cbp->src = m_vcMem->virt_to_phys(&samples(j)[i]);
                cbp->dst = phys_gpclr0;
            }
            cbp->length = 4;
            cbp->stride = 0;
            cbp->next = m_vcMem->virt_to_phys(cbp + 1);
            cbp++;
        }

        /* Last DMA command */
        cbp->info = dmaFlags;
        cbp->src = phys_gplev0;	// Any data will do. Use a register address instead of slow RAM for more speed.
        cbp->dst = phys_fifo_addr;
        cbp->length = 4;
        cbp->stride = 0;
        cbp->next = m_vcMem->virt_to_phys(cbp + 1);
    }

    if (m_loopMode == Loop) {
        m_ctl.cb.back().next = m_vcMem->virt_to_phys(m_ctl.cb.begin());
    } else {
        m_ctl.cb.back().next = m_vcMem->virt_to_phys(&m_ctl.cb.back());
    }

    init_hardware();
}

unsigned int DmaChannel::sampleCount() const
{
    return m_patterns[0].size();
}

void DmaChannel::setPattern(unsigned int subChannel, std::vector<bool> pattern)
{
    if (subChannel >= subchannelCount()) {
        std::stringstream ss;
        ss << "Subchannel " << subChannel
                  << " out of bounds (subchannel count " << subchannelCount()
                  << ")";
        throw std::runtime_error(ss.str());
    } else if (int(subChannel) == m_inputSubChannelIndex) {
        std::stringstream ss;
        ss << "Subchannel " << subChannel
           << " is an input channel and can't have an output pattern!";
        throw std::runtime_error(ss.str());
    }

    pattern.resize(sampleCount());

    const uint32_t phys_gpclr0 = GPIO_PHYS_BASE + 0x28;
    const uint32_t phys_gpset0 = GPIO_PHYS_BASE + 0x1c;

    const size_t cbStride = controlBlockStride();

    for (size_t i = 0; i < this->sampleCount(); ++i) {
        auto cbp = &m_ctl.cb[i * cbStride + subChannel];

        cbp->dst = pattern[i] ? phys_gpset0 : phys_gpclr0;
    }

    m_patterns[subChannel] = std::move(pattern);
}

void DmaChannel::setPwmPattern(unsigned int subChannel, unsigned int frequencyMultiplier)
{
    if (subChannel >= subchannelCount()) {
        std::stringstream ss;
        ss << "Subchannel " << subChannel
                  << " out of bounds (subchannel count " << subchannelCount()
                  << ")";
        throw std::runtime_error(ss.str());
    }

    auto pattern = this->pattern(subChannel);
    const size_t subcycle = pattern.size() / frequencyMultiplier;

    for (size_t i = 0; i < pattern.size(); ++i) {
        pattern[i] = (i % subcycle == 0);
    }

    setPattern(subChannel, pattern);
}

#define clearbit(x, bit) (x) &= ~(1u << bit)
#define setbit(x, bit) (x) |= (1u << bit)

void DmaChannel::setPulseWidth(unsigned int subChannel,
                               unsigned int pin,
                               const std::chrono::nanoseconds &length,
                               unsigned int mult, SetMode mode)
{
    if (subChannel >= subchannelCount()) {
        std::stringstream ss;
        ss << "Subchannel " << subChannel
                  << " out of bounds (subchannel count " << subchannelCount()
                  << ")";
        throw std::runtime_error(ss.str());
    } else if (int(subChannel) == m_inputSubChannelIndex) {
        std::stringstream ss;
        ss << "Subchannel " << subChannel
           << " is an input channel and can't have an output pattern!";
        throw std::runtime_error(ss.str());
    }

    const unsigned subcycleLength = sampleCount() / mult;
    unsigned int width = std::clamp<long>(length / sampleTime(), 0, subcycleLength);
    const auto pattern = this->pattern(subChannel);

    if (width && pattern[width]) {
        width--;
    }

    auto oldWidth = m_pulseWidths[subChannel];
    m_pulseWidths[subChannel] = width;

    auto samps = samples(subChannel);

    if (!width || width == subcycleLength) {
        // Zero-width or full-scale: deactivate PWM and set static value
        if (mode == SetDifferential) {
            for (size_t i = 0; i < samps.size(); i += subcycleLength) {
                clearbit(samps[i], pin);
                clearbit(samps[i + oldWidth], pin);
            }
        } else {
            for (auto &samp : samps) {
                clearbit(samp, pin);
            }
        }
        setGpioValue(pin, width == subcycleLength);
        return;
    }

    if (mode == SetDifferential) {
        for (size_t i = 0; i < samps.size();i += subcycleLength) {
            clearbit(samps[i + oldWidth], pin);
            setbit(samps[i], pin);
            setbit(samps[i + width], pin);
        }
    } else {
        for (size_t i = 0; i < samps.size(); ++i) {
            if ((i % subcycleLength == 0) || (i % subcycleLength == width)) {
                setbit(samps[i], pin);
            } else {
                clearbit(samps[i], pin);
            }
        }
    }
}

void DmaChannel::setPwmDutyCycle(unsigned int subChannel, unsigned int pin, float duty, unsigned int mult, SetMode mode)
{
    using namespace std::chrono;
    setPulseWidth(subChannel, pin,
                  duration_cast<microseconds>(duty / mult * cycleTime()), mult,
                  mode);
}

std::vector<bool> DmaChannel::pattern(unsigned int subChannel) const
{
    if (subChannel >= subchannelCount()) {
        std::stringstream ss;
        ss << "Subchannel " << subChannel
                  << " out of bounds (subchannel count " << subchannelCount()
                  << ")";
        throw std::runtime_error(ss.str());
    }

    return m_patterns[subChannel];
}

tcb::span<uint32_t> DmaChannel::samples(unsigned int subChannel)
{
    if (subChannel >= subchannelCount()) {
        std::stringstream ss;
        ss << "Subchannel " << subChannel
                  << " out of bounds (subchannel count " << subchannelCount()
                  << ")";
        throw std::runtime_error(ss.str());
    }

    return tcb::span<uint32_t>(m_ctl.sample.begin() + subChannel * sampleCount(),
                               sampleCount());
}

tcb::span<uint32_t> DmaChannel::allSamples()
{
    return m_ctl.sample;
}

int DmaChannel::currentSampleIndex() const
{
    auto cur_cb = reinterpret_cast<dma_cb_t*>(m_vcMem->phys_to_virt(dma_reg[DMA_CONBLK_AD]));
    return std::distance(m_ctl.cb.begin(), cur_cb) / controlBlockStride();
}

void DmaChannel::init_hardware() {
    const uint32_t nCycles = m_sampleTime.count() / 100;

    if (m_delayHardware == DelayViaPwm) {
        // Initialise PWM
        std::cout << "Initializing PWM HW" << std::endl;

        pwm_reg[PWM_CTL] = 0;
        udelay(10);
        clk_reg[PWMCLK_CNTL] = 0x5A000006;		// Source=PLLD (500MHz)
        udelay(100);
        clk_reg[PWMCLK_DIV] = 0x5A000000 | (50<<12);	// set pwm div to 50, giving 10MHz
        udelay(100);
        clk_reg[PWMCLK_CNTL] = 0x5A000016;		// Source=PLLD and enable
        udelay(100);
        pwm_reg[PWM_RNG1] = nCycles;  // how many cycles of the clock a sample should last
        udelay(10);
        pwm_reg[PWM_DMAC] = PWMDMAC_ENAB | PWMDMAC_THRSHLD;
        udelay(10);
        pwm_reg[PWM_CTL] = PWMCTL_CLRF;
        udelay(10);
        pwm_reg[PWM_CTL] = PWMCTL_USEF1 | PWMCTL_PWEN1;
        udelay(10);
    } else {
        // Initialise PCM
        std::cout << "Initializing PCM HW" << std::endl;

        pcm_reg[PCM_CS_A] = 1;				// Disable Rx+Tx, Enable PCM block
        udelay(100);
        clk_reg[PCMCLK_CNTL] = 0x5A000006;		// Source=PLLD (500MHz)
        udelay(100);
        clk_reg[PCMCLK_DIV] = 0x5A000000 | (50<<12);	// Set pcm div to 50, giving 10MHz
        udelay(100);
        clk_reg[PCMCLK_CNTL] = 0x5A000016;		// Source=PLLD and enable
        udelay(100);
        pcm_reg[PCM_TXC_A] = 0<<31 | 1<<30 | 0<<20 | 0<<16; // 1 channel, 8 bits
        udelay(100);
        pcm_reg[PCM_MODE_A] = (nCycles - 1) << 10;
        udelay(100);
        pcm_reg[PCM_CS_A] |= 1<<4 | 1<<3;		// Clear FIFOs
        udelay(100);
        pcm_reg[PCM_DREQ_A] = 64<<24 | 64<<8;		// DMA Req when one slot is free?
        udelay(100);
        pcm_reg[PCM_CS_A] |= 1<<9;			// Enable DMA
        udelay(100);
    }

    std::cout << "  one sample = " << nCycles << " cycles at 10 MHz" << std::endl;
}

unsigned int BlasterPP::DmaChannel::controlBlockCount() const
{
    return sampleCount() * controlBlockStride();
}

unsigned int DmaChannel::controlBlockStride() const
{
    return 1 + subchannelCount();
}

}  // namespace BlasterPP
