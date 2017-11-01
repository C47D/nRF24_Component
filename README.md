# nRF24 Component for PSoC4, PSoC5LP and PSoC6 (when available).

The SPI Master component, CS and CE pins nedeed to control the nRF24 chip must be provided by the user on the project schematic. This component only provide the functions to control the nRF24 radio.

Current version: 1.5

![Component](img/v1_3.png)

## TODO for version 1.5 [WIP]
- [ ] Update functions documentation.
- [ ] Component Datasheet.

## TODO for version 2.0 [WIP]
- [] API compatible with the nRF24 component of Erich Styger (mcuoneclipse)

## TODO
- [ ] PSoC6 support.
- [ ] Design a customizer using Visual Studio.

## PSoC6 support
With PSoC6 you can use the component and PDL APIs (see [PSoC6 Components and PDL](http://www.cypress.com/blog/psoc-creator-news-and-information/psoc-6-components-and-pdl-drivers)), given that PSoC6 is like a PSoC4 on steroids, it might be "easy" to port code from PSoC4 to the PSoC6.
Tests for PSoC6 will be done when PSoC 4.1 and the PSoC6 are available.
