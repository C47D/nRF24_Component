# nRF24 Component for PSoC4, PSoC5LP and PSoC6 (work in progress).

The SPI Master component, CS and CE pins needed to control the nRF24 chip must be provided by the user on the project schematic. This component only provide the functions to control the nRF24 radio.

See ![nRF24 Example Projects](github.com/C47D/nRF24_Example_Projects) for multiple example projects using this component.

Current version: 1.5

![Component](img/v1_5.png)

## TODO for version 1.5 [WIP]
- [ ] Update functions documentation.
- [ ] Component Datasheet.
- [ ] PSoC6 support.

## TODO for version 2.0 [WIP]
- [ ] API compatible with the nRF24 component of Erich Styger (mcuoneclipse)

## TODO
- [ ] Design a customizer using Visual Studio.

## PSoC6 support
With PSoC6 you can use the component and PDL APIs (see [PSoC6 Components and PDL](http://www.cypress.com/blog/psoc-creator-news-and-information/psoc-6-components-and-pdl-drivers)), given that PSoC6 is like a PSoC4 on steroids, it might be "easy" to port code from PSoC4 to the PSoC6.
