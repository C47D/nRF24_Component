/****************************************************************************
* This file is part of the nRF24 Custom Component for PSoC Devices.
*
* Copyright (C) 2017 Carlos Diaz <carlos.santiago.diaz@gmail.com>
*
* Permission to use, copy, modify, and/or distribute this software for any
* purpose with or without fee is hereby granted, provided that the above
* copyright notice and this permission notice appear in all copies.
*
* THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
* WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
* ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
* WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
* ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
* OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
****************************************************************************/

/**
* @file `$INSTANCE_NAME`_NRF_FUNCS.c
*
* @brief This file define all the functions available to the user.
*/

#include <`$CE_PIN`.h>
#include <`$SPI_INTERFACE`.h>
#include <`$SS_PIN`.h>

#if defined(CY_SCB_`$SPI_INTERFACE`_H)
#include <`$SPI_INTERFACE`_SPI_UART.h>
#endif

#include "`$INSTANCE_NAME`_LL_SPI.h"
#include "`$INSTANCE_NAME`_NRF_COMMANDS.h"
#include "`$INSTANCE_NAME`_NRF_CONFIG.h"
#include "`$INSTANCE_NAME`_NRF_FUNCS.h"
#include "`$INSTANCE_NAME`_NRF_REGS.h"

// nRF24 Power-on-reset delay
#define `$INSTANCE_NAME`_POR_DELAY 100

/**
 * @brief Configure the radio and clear IRQs, TX and RX FIFOs.
 */
void `$INSTANCE_NAME`_start(void)
{
    // Recommended delay to start using the nRF24
    CyDelay(`$INSTANCE_NAME`_POR_DELAY);
    
    // Now the radio is in Power Down mode
    
    // Set `$SS_PIN` to logic 1, `$CE_PIN` to logic 0 and Start the `$SPI_INTERFACE`
    `$CE_PIN`_Write(0);
    `$SS_PIN`_Write(1);
    `$SPI_INTERFACE`_Start();

    // Flush both nRF24 FIFOs
    `$INSTANCE_NAME`_FlushRxCmd();
    `$INSTANCE_NAME`_FlushTxCmd();
    // Clear IRQ flags
    `$INSTANCE_NAME`_clearAllIRQs();
    
    // Configure the nRF24 with the data from the customizer
    `$INSTANCE_NAME`_init();
    
    // After PWR_UP = 1 the radio is in Standby-I mode, 130us of delay for settling
    CyDelayUs(150);
}

/**
 * @brief Configure the nRF24 registers with the data from the customizer.
 */
void `$INSTANCE_NAME`_init(void)
{
    `$INSTANCE_NAME`_writeRegister(NRF_EN_AA_REG,
        (`$ENAA_P5` << NRF_EN_AA_ENAA_P5) | (`$ENAA_P4` << NRF_EN_AA_ENAA_P4) |
        (`$ENAA_P3` << NRF_EN_AA_ENAA_P3) | (`$ENAA_P2` << NRF_EN_AA_ENAA_P2) |
        (`$ENAA_P1` << NRF_EN_AA_ENAA_P1) | (`$ENAA_P0` << NRF_EN_AA_ENAA_P0));
    `$INSTANCE_NAME`_writeRegister(NRF_EN_RXADDR_REG,
        (`$ERX_P5` << NRF_EN_RXADDR_ERX_P5) | (`$ERX_P4` << NRF_EN_RXADDR_ERX_P4) |
        (`$ERX_P3` << NRF_EN_RXADDR_ERX_P3) | (`$ERX_P2` << NRF_EN_RXADDR_ERX_P2) |
        (`$ERX_P1` << NRF_EN_RXADDR_ERX_P1) | (`$ERX_P0` << NRF_EN_RXADDR_ERX_P0));
    `$INSTANCE_NAME`_writeRegister(NRF_SETUP_AW_REG, `$AW`);
    `$INSTANCE_NAME`_writeRegister(NRF_SETUP_RETR_REG,
        (`$ARD` << NRF_SETUP_RETR_ARD_SHIFT) | `$ARC`);
    `$INSTANCE_NAME`_writeRegister(NRF_RF_CH_REG, `$RF_CH`);
    `$INSTANCE_NAME`_writeRegister(NRF_RF_SETUP_REG,
        (`$CONT_WAVE` << NRF_RF_SETUP_CONT_WAVE) | (`$RF_DATA_RATE` << NRF_RF_SETUP_RF_DR) |
        (`$RF_PWR` << NRF_RF_SETUP_RF_PWR));
    `$INSTANCE_NAME`_writeRegister(NRF_DYNPD_REG,
        (`$DPL_P5` << NRF_DYNPD_DPL_P5) | (`$DPL_P4` << NRF_DYNPD_DPL_P4) |
        (`$DPL_P3` << NRF_DYNPD_DPL_P3) | (`$DPL_P2` << NRF_DYNPD_DPL_P2) |
        (`$DPL_P1` << NRF_DYNPD_DPL_P1) | (`$DPL_P0` << NRF_DYNPD_DPL_P0));
    `$INSTANCE_NAME`_writeRegister(NRF_FEATURE_REG,
        (`$EN_DPL` << NRF_FEATURE_EN_DPL) | (`$EN_ACK_PAY` << NRF_FEATURE_EN_ACK_PAY) |
        (`$EN_DYN_ACK` << NRF_FEATURE_EN_DYN_ACK));

// Configuring data pipes
#if (ENABLE_PIPE0 == 1)
    // Set data pipe 0 bytes in rx payload
    `$INSTANCE_NAME`_writeRegister(NRF_RX_PW_P0_REG, `@RX_PW_P0`);
#endif
#if (ENABLE_PIPE1 == 1)
    // Set data pipe 1 bytes in rx payload
    `$INSTANCE_NAME`_writeRegister(NRF_RX_PW_P1_REG, `@RX_PW_P1`);
#endif
#if (ENABLE_PIPE2 == 1)
    // Set data pipe 2 address (LSB)
    `$INSTANCE_NAME`_writeRegister(NRF_RX_ADDR_P2_REG, `@RX_ADDR_P2`);
    // Set data pipe 2 bytes in rx payload
    `$INSTANCE_NAME`_writeRegister(NRF_RX_PW_P2_REG, `@RX_PW_P2`);
#endif
#if (ENABLE_PIPE3 == 1)
    // Set data pipe 3 address (LSB)
    `$INSTANCE_NAME`_writeRegister(NRF_RX_ADDR_P3_REG, `@RX_ADDR_P3`);
    // Set data pipe 3 bytes in rx payload
    `$INSTANCE_NAME`_writeRegister(NRF_RX_PW_P3_REG, `@RX_PW_P3`);
#endif
#if (ENABLE_PIPE4 == 1)
    // Set data pipe 4 address (LSB)
    `$INSTANCE_NAME`_writeRegister(NRF_RX_ADDR_P4_REG, `@RX_ADDR_P4`);
    // Set data pipe 4 bytes in rx payload
    `$INSTANCE_NAME`_writeRegister(NRF_RX_PW_P4_REG, `@RX_PW_P4`);
#endif
#if (ENABLE_PIPE5 == 1)
    // Set data pipe 5 address (LSB)
    `$INSTANCE_NAME`_writeRegister(NRF_RX_ADDR_P5_REG, `@RX_ADDR_P5`);
    // Set data pipe 5 bytes in rx payload
    `$INSTANCE_NAME`_writeRegister(NRF_RX_PW_P5_REG, `@RX_PW_P5`);
#endif

    `$INSTANCE_NAME`_writeRegister( NRF_CONFIG_REG,
        (`$MASK_RX_DR` << NRF_CONFIG_MASK_RX_DR) | (`$MASK_TX_DS` << NRF_CONFIG_MASK_TX_DS) |
        (`$MASK_MAX_RT` << NRF_CONFIG_MASK_MAX_RT) | (`$EN_CRC` << NRF_CONFIG_EN_CRC) |
        (`$CRCO` << NRF_CONFIG_CRCO) | (`$PWR_UP`<< NRF_CONFIG_PWR_UP) |
        (`$PRIM_RX` << NRF_CONFIG_PRIM_RX));
}

/**
 * @brief Enable the nRF24 radio.
 *
 * @todo Implement this function
 */
void `$INSTANCE_NAME`_enable(void)
{
    // TODO
}

/**
 * @brief Stop the nRF24 radio.
 *
 * @todo Implement this function
 */
void `$INSTANCE_NAME`_stop(void)
{
    // TODO
}

/**
 * @brief Put the nRF24 radio on Sleep mode.
 *
 * @todo Implement this function.
 */
void `$INSTANCE_NAME`_sleep(void)
{
    // TODO
}

/**
 * @brief Wakeup the nRF24 radio, and restore the configuration.
 *
 * @todo Implement this function.
 */
void `$INSTANCE_NAME`_wakeup(void)
{
    // TODO
}

/**
 * @brief Save the nRF24 radio configuration.
 *
 * @todo Implement this function.
 */
void `$INSTANCE_NAME`_saveConfig(void)
{
    // TODO
}

/**
 * @brief Restore the nRF24 radio configuration.
 *
 * @todo Implement this function.
 */
void `$INSTANCE_NAME`_restoreConfig(void)
{
    // TODO
}

/**
 * @brief Configure the radio as Receiver or Transmitter.
 *
 * @param const NrfMode mode: The radio can be configured as Receiver (PRX)
 * or Transmitter (PTX).
 */
void `$INSTANCE_NAME`_setMode(const NrfMode mode)
{
    mode ? `$INSTANCE_NAME`_setRxMode() : `$INSTANCE_NAME`_setTxMode();
}

/**
 * @brief Set radio in Power Down Mode.
 *
 * In power down mode nRF24 is disabled using minimal current consumption.
 * All register values available are maintained and the SPI is kept active,
 * enabling change of configuration and the uploading/downloading of data
 * registers.
 * Power down mode is entered by setting the PWR_UP bit in the CONFIG register
 * low.
 */
void `$INSTANCE_NAME`_setPowerDownMode(void)
{
    `$INSTANCE_NAME`_clearBit(NRF_CONFIG_REG, NRF_CONFIG_PWR_UP);
}

/**
 * @brief Set the radio in Standby I Mode.
 *
 * By setting the PWR_UP bit in the CONFIG register to 1, the device enters
 * standby-I mode. Standby-I mode is used to minimize average current
 * consumption while maintaining short start up times.
 * In this mode only part of the crystal oscillator is active. Change to
 * active modes only happens if CE is set high and when CE is set low,
 * the nRF24 returns to standby-I mode from both the TX and RX modes.
 */
void `$INSTANCE_NAME`_setStandbyIMode(void)
{
    `$INSTANCE_NAME`_listen(false);
    `$INSTANCE_NAME`_clearBit(NRF_CONFIG_REG, NRF_CONFIG_PWR_UP);
}

/**
 * @brief Set the radio in Standby II Mode.
 *
 * In standby-II mode extra clock buffers are active and more current is used
 * compared to standby-I mode. nRF24 enters standby-II mode if CE is held high
 * on a TX device with an empty TX FIFO.
 * If a new packet is uploaded to the TX FIFO, the PLL immediately starts and
 * the packet is transmitted after the normal PLL settling  delay (130us).
 */
void `$INSTANCE_NAME`_setStandbyIIMode(void)
{
    `$INSTANCE_NAME`_listen(true);
    `$INSTANCE_NAME`_setBit(NRF_CONFIG_REG, NRF_CONFIG_PWR_UP);
}

/**
 * @brief Set the radio in Receiver (Rx) Mode.
 *
 * RX mode is an active mode where the nRF24 radio is used as receiver. To
 * enter this mode, the nRF24 must have the PWR_UP bit, PRIM_RX bit and the CE
 * pin set high.
 */
void `$INSTANCE_NAME`_setRxMode(void)
{
    `$INSTANCE_NAME`_setBit(NRF_CONFIG_REG, NRF_CONFIG_PRIM_RX);
}

/**
 * @brief Set the radio in Transmitted (Tx) Mode.
 *
 * TX mode is an active mode for transmitting packets. To enter this mode,
 * the nRF24 must have the PWR_UP bit set high, PRIM_RX bit set low, a payload
 * in the TX FIFO and a high pulse on the CE for more than 10us.
 */
void `$INSTANCE_NAME`_setTxMode(void)
{
    `$INSTANCE_NAME`_clearBit(NRF_CONFIG_REG, NRF_CONFIG_PRIM_RX);
}

/**
 * @brief Enable AutoACK in the given pipe.
 *
 * @param pipe: Enable AutoACK in the given pipe.
 */
void `$INSTANCE_NAME`_enableAutoACK(const NrfDataPipe pipe)
{
    `$INSTANCE_NAME`_setBit(NRF_EN_AA_REG, pipe);
}

/**
 * @brief Disable AutoACK in the given pipe.
 *
 * @param pipe: Disable AutoACK in the given pipe.
 */
void `$INSTANCE_NAME`_disableAutoACK(const NrfDataPipe pipe)
{
    `$INSTANCE_NAME`_clearBit(NRF_EN_AA_REG, pipe);
}

/**
 * @brief Set the channel where the radio will work.
 *
 * @param uint8_t channel: Channel where the radio will work.
 */
void `$INSTANCE_NAME`_setChannel(uint8_t channel)
{
    // There's only 125 channels available
    if (125 < channel) {
        channel = 125;
    }

    `$INSTANCE_NAME`_writeRegister(NRF_RF_CH_REG, channel);
}

/**
 * @brief Set the data pipes address width.
 *
 * @param const NrfSetupAddressWidth addr_width:
 */
void `$INSTANCE_NAME`_setAddressWidth(const NrfSetupAddressWidth addr_width)
{
    `$INSTANCE_NAME`_writeRegister(NRF_SETUP_AW_REG, (uint8_t)addr_width);
}

/**
 * @brief Get the data pipes address width.
 *
 * @return uint8_t: Address width in bytes.
 */
uint8_t `$INSTANCE_NAME`_getAddressWidth(void)
{
    uint8_t addr_width = 0;
    uint8_t reg = `$INSTANCE_NAME`_readRegister(NRF_SETUP_AW_REG);

    switch (reg) {
    case NRF_SETUP_AW_3BYTES:
        addr_width = 3;
        break;
    case NRF_SETUP_AW_4BYTES:
        addr_width = 4;
        break;
    case NRF_SETUP_AW_5BYTES:
        addr_width = 5;
        break;
    default:
        break;
    }
    
    return addr_width;
}

/**
 * @brief Set the Rx Address of the radio.
 *
 * This function configure the address of the Rx Pipe 0 of the radio.
 *
 * @param const uint8_t* addr:
 * @param size_t size:
 */
void `$INSTANCE_NAME`_setRxAddress(const uint8_t* addr, size_t size)
{
    if (NULL == addr) {
        return;
    }

    if (5 < size) {
        size = 5;
    }
    
    `$INSTANCE_NAME`_writeLongRegister(NRF_RX_ADDR_P0_REG, addr, size);
}

/**
 * @brief Get the Rx Address of the radio.
 *
 * @param const uint8_t* addr:
 * @param size_t size:
 */
void `$INSTANCE_NAME`_getRxAddress(uint8_t* addr, size_t size)
{
    if (NULL == addr) {
        return;
    }

    if (5 < size) {
        return;
    }
    
    `$INSTANCE_NAME`_readLongRegister(NRF_RX_ADDR_P0_REG, addr, size);
}

/**
 * @brief Set the address of the RX Pipe 0 in the radio.
 *
 * This function configure the address of the Rx Pipe 0 of the radio.
 *
 * @param const uint8_t* addr:
 * @param size_t size:
 */
void `$INSTANCE_NAME`_setRxPipe0Address(const uint8_t* addr, size_t size)
{
    if (NULL == addr) {
        return;
    }

    // 5 bytes is the maximum address width, here we are limiting the
    // size of the address to 5 bytes but the radio can be configured
    // with 3, 4 or 5 bytes.
    // Add a way to get the address width from the radio configuration?
    if (5 < size) {
        size = 5;
    }
    
    `$INSTANCE_NAME`_writeLongRegister(NRF_RX_ADDR_P0_REG, addr, size);
}

/**
 * @brief Get the address of the RX Pipe 0 in the radio.
 *
 * @param uint8_t* addr:
 * @param size_t size:
 */
void `$INSTANCE_NAME`_getRxPipe0Address(uint8_t* addr, size_t size)
{
    if (NULL == addr) {
        return;
    }

    if (5 < size) {
        return;
    }
    
    `$INSTANCE_NAME`_readLongRegister(NRF_RX_ADDR_P0_REG, addr, size);
}

/**
 * @brief Set the address of the RX Pipe 1 in the radio.
 *
 * This function configure the address of the Rx Pipe 1 of the radio.
 *
 * @param const uint8_t* addr:
 * @param size_t size:
 */
void `$INSTANCE_NAME`_setRxPipe1Address(const uint8_t* addr, size_t size)
{
    if (NULL == addr) {
        return;
    }

    if (5 < size) {
        size = 5;
    }
    
    `$INSTANCE_NAME`_writeLongRegister(NRF_RX_ADDR_P1_REG, addr, size);
}

/**
 * @brief Get the address of the RX Pipe 1 in the radio.
 *
 * @param uint8_t* addr:
 * @param size_t size:
 */
void `$INSTANCE_NAME`_getRxPipe1Address(uint8_t* addr, size_t size)
{
    if (NULL == addr) {
        return;
    }

    if (5 < size) {
        return;
    }
    
    `$INSTANCE_NAME`_readLongRegister(NRF_RX_ADDR_P1_REG, addr, size);
}

#if 0
/**
 * Set the least significant byte of the addresses of pipe 2, 3, 4 and 5.
 *
 * @param const NrfPipeAddress pipe:
 * @param const uint8_t addr_lsb:
 * @return RET_SUCESS in case of reading sucesfully the address, RET_PARAM in
 * case of an invalid parameter.
 */
uint8_t `$INSTANCE_NAME`_setRxAddress(const NrfRxPipeAddress pipe, const uint8_t* addr,
                                        const size_t size)
{
    uint8_t ret = RET_SUCCESS;
    
    if (NULL == addr) {
        ret = RET_NULL_PTR;
    }
    
    if (!(`$INSTANCE_NAME`_is_valid_rx_pipe_address(pipe))) {
        ret = RET_BAD_PARAM;    
    }
    
    // Check if it's trying to set the address of the data pipes 0 or 1.
    if ((NRF_P0_ADDR == pipe) || (NRF_P1_ADDR == pipe)) {
        
        // if so we can set their address to a size of 3, 4 or 5 bytes
        if (`$INSTANCE_NAME`_addr_width < size) {
            ret = RET_BAD_PARAM;
        }
        
        `$INSTANCE_NAME`_writeLongRegister(pipe, addr, size);
        
    } else {
        
        // We can only set the least significant byte of the address of the
        // data pipes 2, 3, 4 and 5.
        if (1 < size) {
            ret = RET_BAD_PARAM;
        } else {
            // Write addr to pipe in case of a valid size
            `$INSTANCE_NAME`_writeRegister(pipe, addr[0]);
        }
    }
    
    return ret;
}

/**
 * @brief Get the address of Rx data pipe 2, 3, 4, 5.
 *
 * @param const NrfPipeAddress pipe:
 * @param uint8_t* addr:
 * @param size_t size:
 */
uint8_t `$INSTANCE_NAME`_getRxAddress(const NrfPipeAddress pipe, uint8_t* addr, size_t size)
{
    uint8_t ret = RET_SUCCESS;
    
    if (NULL == addr) {
        ret = RET_NULL_PTR;
    }

    if (`$INSTANCE_NAME`_addr_width < size) {
        ret = RET_BAD_PARAM;
    }
    
    if (!(`$INSTANCE_NAME`_is_valid_rx_pipe_address(pipe))) {
        ret = RET_BAD_PARAM;    
    }
    
    // Read the pipe if it's trying to get the address of the data pipes 0 or 1.
    if ((NRF_P0_ADDR == pipe) || (NRF_P1_ADDR == pipe)) {
        `$INSTANCE_NAME`_readLongRegister(pipe, addr, size);
    } else {
        // The pipe2,3,4,5 addresses are the same as the pipe1 address except the LSB
        nRF24_readLongRegister(NRF_P1_ADDR, addr, size - 1);
        addr[size - 1] = nRF24_readRegister(pipe);
    }
    
    return ret;
}

// return 1 if pipe is a NrfPipeAddress, 0 if it's not.
uint8_t `$INSTANCE_NAME`_is_valid_rx_pipe_address(const NrfPipeAddress pipe)
{
    // check if pipe value is between NRF_P0_ADDR and NRF_P5_ADDR range.
    return ((NRF_P0_ADDR <= pipe) && (NRF_P5_ADDR >= pipe)) ? 1 : 0;
}
#endif

/**
 * Set the address of the least significant byte of the Rx Pipe 2 in the radio.
 *
 * @param const uint8_t addr_lsb:
 */
void `$INSTANCE_NAME`_setRxPipe2Address(const uint8_t addr_lsb)
{
    `$INSTANCE_NAME`_writeRegister(NRF_RX_ADDR_P2_REG, addr_lsb);
}

/**
 * Get the address of the Rx Pipe 2.
 *
 * @param uint8_t* addr:
 * @param size_t size:
 */
void `$INSTANCE_NAME`_getRxPipe2Address(uint8_t* addr, size_t size)
{
    if (NULL == addr) {
        return;
    }

    if (5 < size) {
        return;
    }
    
    // The pipe2 address is the same as the pipe1 address except the LSB
    nRF24_readLongRegister(NRF_RX_ADDR_P1_REG, addr, size - 1);
    addr[size - 1] = nRF24_readRegister(NRF_RX_ADDR_P2_REG);
}

/**
 * Set the address of the least significant byte of the Rx Pipe 3 in the radio.
 *
 * @param const uint8_t addr_lsb:
 */
void `$INSTANCE_NAME`_setRxPipe3Address(const uint8_t addr_lsb)
{
    `$INSTANCE_NAME`_writeRegister(NRF_RX_ADDR_P3_REG, addr_lsb);
}

/**
 * Get the address of the Rx Pipe 3.
 *
 * @param uint8_t* addr:
 * @param size_t size:
 */
void `$INSTANCE_NAME`_getRxPipe3Address(uint8_t* addr, size_t size)
{
    if (NULL == addr) {
        return;
    }

    if (5 < size) {
        return;
    }
    
    // The pipe3 address is the same as the pipe1 address except the LSB
    nRF24_readLongRegister(NRF_RX_ADDR_P1_REG, addr, size - 1);
    addr[size - 1] = nRF24_readRegister(NRF_RX_ADDR_P3_REG);
}

/**
 * Set the address of the least significant byte of the Rx Pipe 4 in the radio.
 *
 * @param const uint8_t addr_lsb:
 */
void `$INSTANCE_NAME`_setRxPipe4Address(const uint8_t addr_lsb)
{
    `$INSTANCE_NAME`_writeRegister(NRF_RX_ADDR_P4_REG, addr_lsb);
}

/**
 * Get the address of the Rx Pipe 4.
 *
 * @param uint8_t* addr:
 * @param size_t size:
 */
void `$INSTANCE_NAME`_getRxPipe4Address(uint8_t* addr, size_t size)
{
    if (NULL == addr) {
        return;
    }

    if (5 < size) {
        return;
    }
    
    // The pipe4 address is the same as the pipe1 address except the LSB
    nRF24_readLongRegister(NRF_RX_ADDR_P1_REG, addr, size - 1);
    addr[size - 1] = nRF24_readRegister(NRF_RX_ADDR_P4_REG);
}

/**
 * Set the address of the least significant byte of the Rx Pipe 5 in the radio.
 *
 * @param const uint8_t addr_lsb:
 */
void `$INSTANCE_NAME`_setRxPipe5Address(const uint8_t addr_lsb)
{
    `$INSTANCE_NAME`_writeRegister(NRF_RX_ADDR_P5_REG, addr_lsb);
}

/**
 * Get the address of the Rx Pipe 5.
 *
 * @param uint8_t* addr:
 * @param size_t size:
 */
void `$INSTANCE_NAME`_getRxPipe5Address(uint8_t* addr, size_t size)
{
    if (NULL == addr) {
        return;
    }

    if (5 < size) {
        return;
    }
    
    // The pipe5 address is the same as the pipe1 address except the LSB
    nRF24_readLongRegister(NRF_RX_ADDR_P1_REG, addr, size - 1);
    addr[size - 1] = nRF24_readRegister(NRF_RX_ADDR_P5_REG);
}

/**
 * @brief Set the TX Address of the radio.
 *
 * @param const uint8_t* addr:
 * @param size_t size:
 */
void `$INSTANCE_NAME`_setTxAddress(const uint8_t* addr, size_t size)
{
    if (NULL == addr) {
        return;
    }

    if (5 < size) {
        size = 5;
    }
    
    `$INSTANCE_NAME`_writeLongRegister(NRF_TX_ADDR_REG, addr, size);
}

/**
 * @brief Get the TX Address of the radio.
 *
 * @param const uint8_t* addr:
 * @param const size_t size:
 */
void `$INSTANCE_NAME`_getTxAddress(uint8_t* addr, const size_t size)
{
    if (NULL == addr) {
        return;
    }

    if (5 < size) {
        return;
    }
    
    `$INSTANCE_NAME`_readLongRegister(NRF_TX_ADDR_REG, addr, size);
}

/**
 * @brief Set the payload size of the given pipe.
 *
 * Configure the payload size of the given pipe.
 *
 * @param const NrfDataPipe pipe:
 * @param uint8_t size:
 */
void `$INSTANCE_NAME`_setPayloadSize(const NrfDataPipe pipe, uint8_t size)
{
    if (32 < size) {
        size = 32;
    }

    switch (pipe) {
    case NRF_DATA_PIPE0:
        `$INSTANCE_NAME`_writeRegister(NRF_RX_PW_P0_REG, size);
        break;
    case NRF_DATA_PIPE1:
        `$INSTANCE_NAME`_writeRegister(NRF_RX_PW_P1_REG, size);
        break;
    case NRF_DATA_PIPE2:
        `$INSTANCE_NAME`_writeRegister(NRF_RX_PW_P2_REG, size);
        break;
    case NRF_DATA_PIPE3:
        `$INSTANCE_NAME`_writeRegister(NRF_RX_PW_P3_REG, size);
        break;
    case NRF_DATA_PIPE4:
        `$INSTANCE_NAME`_writeRegister(NRF_RX_PW_P4_REG, size);
        break;
    case NRF_DATA_PIPE5:
        `$INSTANCE_NAME`_writeRegister(NRF_RX_PW_P5_REG, size);
        break;
    default:
        break;
    }
}

/**
 * Get the payload size of the given pipe.
 *
 * @param const NrfDataPipe pipe: Pipe to be read.
 *
 * @return uint8_t: Configured payload size of the given pipe.
 */
uint8_t `$INSTANCE_NAME`_getPayloadSize(const NrfDataPipe pipe)
{
    switch (pipe) {
    case NRF_DATA_PIPE0:
        return `$INSTANCE_NAME`_readRegister(NRF_RX_PW_P0_REG);
        break;
    case NRF_DATA_PIPE1:
        return `$INSTANCE_NAME`_readRegister(NRF_RX_PW_P1_REG);
        break;
    case NRF_DATA_PIPE2:
        return `$INSTANCE_NAME`_readRegister(NRF_RX_PW_P2_REG);
        break;
    case NRF_DATA_PIPE3:
        return `$INSTANCE_NAME`_readRegister(NRF_RX_PW_P3_REG);
        break;
    case NRF_DATA_PIPE4:
        return `$INSTANCE_NAME`_readRegister(NRF_RX_PW_P4_REG);
        break;
    case NRF_DATA_PIPE5:
        return `$INSTANCE_NAME`_readRegister(NRF_RX_PW_P5_REG);
        break;
    default:
        return 0;
    }
}

/**
 * @brief Reuse last transmitted payload.
 *
 * This function issue the command ReuseTxPayload and then toggle the CE pin
 * to transmit the last transmitted payload.
 */
void `$INSTANCE_NAME`_PTX_reuseLastTransmittedPayload(void)
{
    `$INSTANCE_NAME`_PTX_ReuseTxPayloadCmd();
    `$INSTANCE_NAME`_transmitPulse();
}

/**
 * @brief Enable dynamic payload on the given pipe.
 *
 * @param const NrfDataPipe pipe:
 */
void `$INSTANCE_NAME`_enableDynamicPayload(const NrfDataPipe pipe)
{
    `$INSTANCE_NAME`_setBit(NRF_EN_AA_REG, pipe);
    `$INSTANCE_NAME`_setBit(NRF_FEATURE_REG, NRF_FEATURE_EN_ACK_PAY);
    `$INSTANCE_NAME`_setBit(NRF_FEATURE_REG, NRF_FEATURE_EN_DPL);
    `$INSTANCE_NAME`_setBit(NRF_DYNPD_REG, pipe);
}

/**
 * @brief Disable dynamic payload on the given pipe.
 *
 * @param const NrfDataPipe pipe:
 */
void `$INSTANCE_NAME`_disableDynamicPayload(const NrfDataPipe pipe)
{
    `$INSTANCE_NAME`_clearBit(NRF_EN_AA_REG, pipe);
    `$INSTANCE_NAME`_clearBit(NRF_FEATURE_REG, NRF_FEATURE_EN_ACK_PAY);
    `$INSTANCE_NAME`_clearBit(NRF_FEATURE_REG, NRF_FEATURE_EN_DPL);
    `$INSTANCE_NAME`_clearBit(NRF_DYNPD_REG, pipe);
}

/**
 * @brief Enable dinamic payload length.
 */
void `$INSTANCE_NAME`_enableDynamicPayloadLength(void)
{
    `$INSTANCE_NAME`_setBit(NRF_FEATURE_REG, NRF_FEATURE_EN_DPL);
}

/**
 * @brief Disable dynamic payload length.
 */
void `$INSTANCE_NAME`_disableDynamicPayloadLength(void)
{
    `$INSTANCE_NAME`_clearBit(NRF_FEATURE_REG, NRF_FEATURE_EN_DPL);
}

/**
 * @brief Enable payload with ACK.
 */
void `$INSTANCE_NAME`_enablePayloadWithACK(void)
{
    `$INSTANCE_NAME`_setBit(NRF_FEATURE_REG, NRF_FEATURE_EN_ACK_PAY);
}

/**
 * @brief Disable Payload with ACK.
 */
void `$INSTANCE_NAME`_disablePayloadWithACK(void)
{
    `$INSTANCE_NAME`_clearBit(NRF_FEATURE_REG, NRF_FEATURE_EN_ACK_PAY);
}

/**
 * @brief Enable dynamic payload length.
 */
void `$INSTANCE_NAME`_enablePayloadWithNoACKCmd(void)
{
    `$INSTANCE_NAME`_setBit(NRF_FEATURE_REG, NRF_FEATURE_EN_DYN_ACK);
}

/**
 * @brief Disable Payload with no ACK.
 */
void `$INSTANCE_NAME`_disablePayloadWithNoACKCmd(void)
{
    `$INSTANCE_NAME`_clearBit(NRF_FEATURE_REG, NRF_FEATURE_EN_DYN_ACK);
}

/**
 * @brief Set the value of the CE pin to listen or not.
 *
 * @param const bool listen: If @listen is set to true, then the radio will
 * listen, if set to false the radio will stop listening.
 */
void `$INSTANCE_NAME`_listen(const bool listen)
{
    listen ? `$CE_PIN`_Write(1) : `$CE_PIN`_Write(0);
}

/**
 * @brief The nRF24 radio will start listening.
 *
 * This function set the pin CE to logic high, this enable the radio for
 * listening.
 */
void `$INSTANCE_NAME`_startListening(void) {
    `$INSTANCE_NAME`_listen(true);
}

/**
 * @brief The nRF24 radio will stop listening.
 *
 * This function the pin CE of the nRF24 radio will be set to logic low,
 * this disable the radio for listening.
 */
void `$INSTANCE_NAME`_stopListening(void) {
    `$INSTANCE_NAME`_listen(false);
}

/**
 * @brief Transmit pulse on the CE pin.
 *
 * With this function the CE pin of the nRF24 radio will have a pulse of 15us,
 * this pulse trigger a transmission of the content of the TX FIFO.
 */
void `$INSTANCE_NAME`_transmitPulse(void)
{
    `$CE_PIN`_Write(1);
    CyDelayUs(15);
    `$CE_PIN`_Write(0);
}

/**
 * @brief Get the STATUS register of the nRF24.
 *
 * @return uint8_t: STATUS register of the nRF24.
 */
uint8_t `$INSTANCE_NAME`_getStatus(void)
{
    return `$INSTANCE_NAME`_NOPCmd();
}

/**
 * @brief Get the number of retransmissions.
 *
 * @return uint8_t: Retransmissions count.
 */
uint8_t `$INSTANCE_NAME`_getRetransmissionsCount(void)
{
    uint8_t count = `$INSTANCE_NAME`_readRegister(NRF_OBSERVE_TX_REG);
    return count & NRF_OBSERVE_TX_ARC_CNT_MASK;
}

/**
 * @brief Get the number of lost packets.
 *
 * @return uint8_t: Lost packets.
 */
uint8_t `$INSTANCE_NAME`_getLostPacketsCount(void)
{
    uint8_t lostPackets = `$INSTANCE_NAME`_readRegister(NRF_OBSERVE_TX_REG);
    lostPackets = lostPackets & NRF_OBSERVE_TX_PLOS_CNT_MASK;
    return lostPackets >> NRF_OBSERVE_TX_PLOS_CNT_POS;
}

/**
 * Put data into the TX FIFO wihout sending it.
 *
 * @param const uint8_t* data:
 * @param const size_t size:
 */
void `$INSTANCE_NAME`_putInTXFIFO(const uint8_t* data, const size_t size)
{
    if (NULL == data) {
        return;
    }

    if (32 < size) {
        return;
    }
    
    `$INSTANCE_NAME`_WriteTXPayloadCmd(data, size);
}

/**
 * Put data in TX FIFO and transmit it.
 *
 * @param const uint8_t* data:
 * @param const size_t size:
 */
void `$INSTANCE_NAME`_PTX_Transmit(const uint8_t* data, const size_t size)
{
    if (NULL == data) {
        return;
    }
    
    if (32 < size) {
        return;
    }

    `$INSTANCE_NAME`_putInTXFIFO(data, size);
    `$INSTANCE_NAME`_transmitPulse();
}

/**
 * @brief
 *
 * @return bool: True if there's data ready.
 */
bool `$INSTANCE_NAME`_isDataReady(void)
{
    return NRF_STATUS_RX_DR_MASK & `$INSTANCE_NAME`_getStatus();
}

/**
 * @brief
 *
 * @param uint8_t* data:
 * @param const size_t size:
 */
void `$INSTANCE_NAME`_getRxPayload(uint8_t* data, const size_t size)
{
    if (NULL == data) {
        return;
    }
    
    `$CE_PIN`_Write(0);
    `$INSTANCE_NAME`_PRX_ReadRXPayloadCmd(data, size);
    `$CE_PIN`_Write(1);
}

/**
 * @brief
 *
 * @param const uint8_t* data:
 * @param size_t size:
 */
void `$INSTANCE_NAME`_txTransmitWaitNoACK(const uint8_t* data, const size_t size)
{
    if (NULL == data) {
        return;
    }

    if (32 < size) {
        return;
    }
    
    `$INSTANCE_NAME`_PTX_NoACKPayloadCmd(data, size);
    `$INSTANCE_NAME`_transmitPulse();
}

/**
 * @brief
 *
 * @param const NrfDataPipe pipe:
 * @param const uint8_t* data:
 * @param size_t size:
 */
void `$INSTANCE_NAME`_rxWritePayload(const NrfDataPipe pipe, const uint8_t* data,
                                     const size_t size)
{
    if (NULL == data) {
        return;
    }

    if (32 < size) {
        return;
    }
    
    `$INSTANCE_NAME`_PRX_WriteACKPayloadCmd(pipe, data, size);
}

/**
 * Return the pipe number with data.
 *
 * @return uint8_t:
 */
uint8_t `$INSTANCE_NAME`_getDataPipeWithPayload(void)
{
    uint8_t pipe = `$INSTANCE_NAME`_readRegister(NRF_STATUS_REG);
    return (pipe & 0x0Eu) >> 1;
}

/**
 * @brief
 *
 * @return uint8_t:
 */
uint8_t `$INSTANCE_NAME`_PRX_receivedPowerDetector(void)
{
    return `$INSTANCE_NAME`_readBit(NRF_RPD_REG, NRF_RPD_RPD);
}

/**
 * @return bool: true if the TX FIFO is full, false if it have available
 * locations.
 */
bool `$INSTANCE_NAME`_isTXFIFOFull(void)
{
    return `$INSTANCE_NAME`_readBit(NRF_FIFO_STATUS_REG, NRF_FIFO_STATUS_TX_FULL);
}

/**
 * @return bool: true if the RX FIFO es empty, false if any pipe have data.
 */
bool `$INSTANCE_NAME`_isRXFIFOEmpty(void)
{
    return `$INSTANCE_NAME`_readBit(NRF_FIFO_STATUS_REG, NRF_FIFO_STATUS_RX_EMPTY);
}

/**
 * This function will write 1 to all the three IRQ "flag" bits on the
 * STATUS register.
 */
void `$INSTANCE_NAME`_clearAllIRQs(void)
{
    `$INSTANCE_NAME`_writeRegister(NRF_STATUS_REG, 0x70);
}

/**
 * @brief Clears the specific IRQ flag.
 *
 * Clear the flag by writing 1 to the interrupt flag bit.
 *
 * @param NrfIRQ irq_flag: Interrupt flag to clear.
 */
void `$INSTANCE_NAME`_clearIRQFlag(const NrfIRQ irq_flag)
{
    `$INSTANCE_NAME`_writeRegister(NRF_STATUS_REG, (1 << irq_flag));
}

/**
 * This method is used to get if any of the IRQ "flag" bits on the STATUS
 * register is set to 1. This function returns 0 is none of the flag bits
 * is not set to 1.
 *
 * @return NrfIRQ: Asserted bit of the interrupt flags.
 */
NrfIRQ `$INSTANCE_NAME`_getIRQFlag(void)
{
    // Get the STATUS register
    uint8_t sts = `$INSTANCE_NAME`_NOPCmd();

    // We only care if any of the bits 4, 5 and 6 are set, so we mask
    // the STATUS register with 0x0111_0000
    switch (sts & 0x70) {
    case NRF_STATUS_RX_DR_MASK:
        return NRF_RX_DR_IRQ;
        break;
    case NRF_STATUS_TX_DS_MASK:
        return NRF_TX_DS_IRQ;
        break;
    case NRF_STATUS_MAX_RT_MASK:
        return NRF_MAX_RT_IRQ;
        break;
    default:
        return 0;
        break;
    }
}

/* [] END OF FILE */
