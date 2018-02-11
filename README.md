# nRF24 Component for PSoC4, PSoC5LP and PSoC6

The nRF24 component is developed as a personal project, it surely can be improved, pull request are always welcome!.


The nRF24 radios communicate via SPI with their controller, so it needs MISO, MOSI, SCLK and SS signals (for SPI communication), a digital output pin to control the CE pin and (the optional use of) a digital input pin for the IRQ pin to communicate interrupts back to the controller.


The component control the SS pin (also known as CS or Chip Select) via software to avoid problems related to hardware-controlled SS pins, the biggest problem solved with this is that now your SPI Master component can have any TX and RX FIFO depths, also this leave the hardware-controlled SS pin of the component for the user if more than one SPI slave needs to be controlled.


The SPI Master component, SS, CE and IRQ (IRQ is optional) pins must be provided by the user on the project schematic. See the image below for an example.


![nRF24_sch_example](img/nRF24_sch_example.png)

## Example projects

See [nRF24 Example Projects](https://github.com/C47D/nRF24_Example_Projects) for basic example projects using this component!.

## How to use this component on your PSoC project?

You can directly clone this repo inside your project directory and update it as you want. After that you have to include it into your project dependencies.

If you want you can also include this repo as a git submodule.

## API

The functions provided by this component try to be as clear as possible, so the user should not need to check the component documentation to know what a function does.

All the functions on the NRF_FUNCS files are intended to be used directly by the user, so no need to use the functions on the NRF_LL_SPI and NRF_COMMANDS files.

The following code style where used on the NRF_REGS file to ease finding the available registers and bitfields:

- NRF_REG_x means this is a register, where x is the register name as documented on the nRF24 datasheet.

- NRF_x_BIT_y means this is a bit field on a register, where x is the register name and y is the name of the bitfield as documented on the nRF24 datasheet.

More symbols are provided, but with less "cryptic" names.

## Current version: 2.0

The current component symbol is shown below:

![Component](img/v2_0.png)

The *customizer* or configuration window is where the PSoC Creator "magic" happens, the custom *customizers* are ugly so i'm also trying to write a customizer in C# and add it to the component (Work in Progress), this is not really well documented and i'm pretty bad at desining GUI's so if you know C# and UI you can help me out.


The component configuration is separated in two tabs:


- nRF Configuration

In this tab all the configurable nRF24 registers are layed out, at the right of the window is a small description on each register field, it's better if you have a copy of the nRF24 datasheet at hand.

![nRF_Configuration](img/nRF24_conf.png)

- Hardware Interface

In this tab the user need to provide the names of the SPI Master component used for communication with the nRF24 radio, the name of the pin used to control the SS (slave select) and the pin used to control the CE pin.

![nRF_SPI](img/nRF24_spi_v2.png)

## Coding style

The code style in the v2.0 will be similar to the [Linux kernel coding style](https://www.kernel.org/doc/html/v4.10/process/coding-style.html), for this the tool clang-format is used, the repo contains the .clang-format file.
