# nRF24 Component for PSoC4, PSoC5LP and PSoC6

The nRF24 component was initially developed as a personal project, so it can be improved, pull request are always welcome!.

The nRF24 radios communicate via SPI with their external controller, so it needs MISO, MOSI, SCLK and SS signals (for SPI communication), a digital output pin to control the CE pin and (the optional use of) a digital input pin for the IRQ pin to communicate interrupts to the controller.

This component use the SS (also known as CS or Chip Select) controlled via software to avoid problems with hardware-controlled SS pins, so your SPI Master component can have any TX and RX FIFO depths.

The SPI Master component, SS, CE and IRQ pins must be provided by the user on the project schematic. See the image below for an example.

![nRF24_sch_example](img/nRF24_sch_example.png)

## Example projects

See ![nRF24 Example Projects](https://github.com/C47D/nRF24_Example_Projects) for basic example projects using this component!.

## Current version: 1.6

The current component symbol is shown below:

![Component](img/v1_6.png)

The *customizer* or configuration window is separated in two tabs:
- nRF Configuration

![nRF_Configuration](img/nRF24_conf.png)

- SPI Interface (TODO: Find a more appropiate name)

![nRF_SPI](img/nRF24_spi.png)

## TODO for version 1.6 [WiP]
- [ ] Update documentation.
- [ ] Component Datasheet.
- [x] PSoC6 support (WiP).

## TODO for version 2.0 [WiP]
- [ ] API compatible with the nRF24 component of Erich Styger (mcuoneclipse)
- [ ] Design the customizer using Visual Studio.

## Coding style [WiP]

The code style in the v2.0 will be similar to the ![Linux kernel coding style](https://www.kernel.org/doc/html/v4.10/process/coding-style.html), for this the tool clang-format is used, the repo contains the .clang-format file.
