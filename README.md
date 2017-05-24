# nRF24 Component for PSoC4, 5LP and 6.

This component aims to be usable on PSoC4, PSoC5LP (and PSoC6 when available).
The SPI and other pins necessary to control the nRF24 chip must be provided by the user "outside" this component.

## TODO
- [x] v1.0 Finish the component configuration tab on the customizer.
- [x] v1.0 Improve the SPI data transactions, use the whole SPI FIFO instead of transferring byte by byte. 
- [ ] vX.X Add macros and examples for PSoC6
- [ ] vX.X Add a proper customizer using Visual Studio to design it.
