# blasterpp

`blasterpp` provides a C++ class (`BlasterPP::DmaChannel`) to easily push data
to the GPIOs of a Raspberry Pi via DMA. This allows for precise hardware PWM
of different frequencies on different GPIOs. Besides precise PWM, you can
actually push any data you like to the GPIOs (although BlasterPP's interface is
optimized for cyclic stuff like PWM).

`BlasterPP` supports frequency multiplication, so you can use a single DMA channel
to generate PWM at harmonics of the base frequency. For example, you could set
your base cycle frequency to 50 Hz and use a multiplier of 8. With this, you can
generate PWM at 50, 100, 200 and 400 Hz, all within the same DMA channel but on
different pins.
Further, two simultaneous DMA channels using the PWM and PCM hardware are
supported.

## Credits

BlasterPP is based on [pi-blaster](https://github.com/sarfata/pi-blaster), which is
in turn based on Richard Hirst's excellent [ServoBlaster](https://github.com/sarfata/pi-blaster).
