## nRF24 Component v2.0

### News


### Changes

- Change name from NRF_REGS.h to NRF_DEFS.h
- Change name from NRF_LL_SPI.h to NRF_HAL.h
- Replaced all the set_rx_pipex_address (where x is the pipe number from 0 to 5) with only one funtion set_rx_pipe_address. Same for the get_rx_pipex_address function (replaced with a unique get_rx_pipe_address), both new functions now need an extra parameter, the pipe number you want to set/get the address (enum nrf_addr_rx_pipe).
- Register names start with NRF_REG so it's easier for the user to found "valid" registers, same happens with the commands, they start with NRF_CMD.
- Similarly now the nrf_addr_rx_pipe enum start with NRF_ADDR_PIPEx (where x is the pipe number) to ease its use.
- The nrf_pipe_payload_size enum is now renamed to nrf_pld_size enum, all the values start with NRF_PLD_SIZE_PIPEx (where x is the pipe number) to ease its use.
- The nrf_setup_address_width enum is now renamed to nrf_addr_width.
- The nrf_pipe_address_width enum is now renamed to nrf_pipe_addr_width.
