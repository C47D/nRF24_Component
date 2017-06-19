# nRF24 Component for PSoC4, 5LP and 6.

This component aims to be usable on PSoC4, PSoC5LP (and PSoC6 when available).
The SPI and other pins necessary to control the nRF24 chip must be provided by the user "outside" this component.

PSoC6: The API on PSoC6 is based on the PDL library, the SPI calls are done on the HAL_SPI.c file, so the changes made to support PSoC6 as SPI Master will be done in that file.

## TODO
- [x] Finish the component configuration tab on the customizer.
- [x] Improve the SPI data transactions, use the whole SPI FIFO instead of transferring byte by byte. 
- [x] Check SCB SPI version, it fails on some transfers.
- [ ] Improve set and clear bit functions, now they work on single bits.
- [ ] Add macros and examples for PSoC6.
- [ ] Add a proper customizer using Visual Studio to design it.


Notes:
- [x] meaning fixed.
