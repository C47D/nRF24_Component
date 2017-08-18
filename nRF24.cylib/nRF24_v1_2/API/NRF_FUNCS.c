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
 * @brief Configure the radio and clear TX and RX FIFOs.
 *
 * @param None.
 *
 * @return None.
 */
void `$INSTANCE_NAME`_start(void)
{
    `$SS_PIN`_Write(1);
    `$SPI_INTERFACE`_Start();
    // Recommended delay to start using the nRF24
    CyDelay(`$INSTANCE_NAME`_POR_DELAY);
    // Configure the nRF24 with the configuration
    // taken from the customizer.
    `$INSTANCE_NAME`_init();
    // Flush both nRF24 FIFOs
    `$INSTANCE_NAME`_FlushRxCmd();
    `$INSTANCE_NAME`_FlushTxCmd();
    // Clear IRQ flags
    `$INSTANCE_NAME`_clearAllIRQs();
}

/**
 * @brief Configure the nRF24 registers with the data of the customizer.
 *
 * @param None.
 *
 * @return None.
 */
void `$INSTANCE_NAME`_init(void)
{
    `$INSTANCE_NAME`_writeRegister(
        NRF_CONFIG_REG,
        (`$MASK_RX_DR` << NRF_CONFIG_MASK_RX_DR) |
            (`$MASK_TX_DS` << NRF_CONFIG_MASK_TX_DS) |
            (`$MASK_MAX_RT` << NRF_CONFIG_MASK_MAX_RT) |
            (`$EN_CRC` << NRF_CONFIG_EN_CRC) | (`$CRCO` << NRF_CONFIG_CRCO) |
            ( `$PWR_UP`<< NRF_CONFIG_PWR_UP) |
            (`$PRIM_RX` << NRF_CONFIG_PRIM_RX));
    `$INSTANCE_NAME`_writeRegister(NRF_EN_AA_REG,
                                   (`$ENAA_P5` << NRF_EN_AA_ENAA_P5) |
                                       (`$ENAA_P4` << NRF_EN_AA_ENAA_P4) |
                                       (`$ENAA_P3` << NRF_EN_AA_ENAA_P3) |
                                       (`$ENAA_P2` << NRF_EN_AA_ENAA_P2) |
                                       (`$ENAA_P1` << NRF_EN_AA_ENAA_P1) |
                                       (`$ENAA_P0` << NRF_EN_AA_ENAA_P0));
    `$INSTANCE_NAME`_writeRegister(NRF_EN_RXADDR_REG,
                                   (`$ERX_P5` << NRF_EN_RXADDR_ERX_P5) |
                                       (`$ERX_P4` << NRF_EN_RXADDR_ERX_P4) |
                                       (`$ERX_P3` << NRF_EN_RXADDR_ERX_P3) |
                                       (`$ERX_P2` << NRF_EN_RXADDR_ERX_P2) |
                                       (`$ERX_P1` << NRF_EN_RXADDR_ERX_P1) |
                                       (`$ERX_P0` << NRF_EN_RXADDR_ERX_P0));
    `$INSTANCE_NAME`_writeRegister(NRF_SETUP_AW_REG, `$AW`);
    `$INSTANCE_NAME`_writeRegister(
        NRF_SETUP_RETR_REG, (`$ARD` << NRF_SETUP_RETR_ARD_SHIFT) | `$ARC`);
    `$INSTANCE_NAME`_writeRegister(NRF_RF_CH_REG, `$RF_CH`);
    `$INSTANCE_NAME`_writeRegister(NRF_RF_SETUP_REG,
                                   (`$CONT_WAVE` << NRF_RF_SETUP_CONT_WAVE) |
                                       (`$RF_DATA_RATE` << NRF_RF_SETUP_RF_DR) |
                                       (`$RF_PWR` << NRF_RF_SETUP_RF_PWR));
    `$INSTANCE_NAME`_writeRegister(
        NRF_DYNPD_REG,
        (`$DPL_P5` << NRF_DYNPD_DPL_P5) | (`$DPL_P4` << NRF_DYNPD_DPL_P4) |
            (`$DPL_P3` << NRF_DYNPD_DPL_P3) | (`$DPL_P2` << NRF_DYNPD_DPL_P2) |
            (`$DPL_P1` << NRF_DYNPD_DPL_P1) | (`$DPL_P0` << NRF_DYNPD_DPL_P0));
    `$INSTANCE_NAME`_writeRegister(
        NRF_FEATURE_REG,
        (`$EN_DPL` << NRF_FEATURE_EN_DPL) |
            (`$EN_ACK_PAY` << NRF_FEATURE_EN_ACK_PAY) |
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
}

/**
 * @brief Enable the nRF24 component.
 *
 * @todo Implement this function
 *
 * @param None.
 *
 * @return None.
 */
void `$INSTANCE_NAME`_enable(void)
{
    // TODO
}

/**
 * @brief Stop the nRF24 component, and all the internal components.
 *
 * @todo Implement this function
 *
 * @param None.
 *
 * @return None.
 */
void `$INSTANCE_NAME`_stop(void)
{
    // TODO
}

/**
 * @brief Put the nRF24 component on Sleep mode also all the internal
 * components.
 *
 * @todo Implement this function.
 *
 * @param None.
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_sleep(void)
{
    // TODO
}

/**
 * @brief Wakeup the nRF24 components, and restore the configuration.
 *
 * @todo Implement this function.
 *
 * @param None.
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_wakeup(void)
{
    // TODO
}

/**
 * @brief Save the nRF24 component configuration.
 *
 * @todo Implement this function.
 *
 * @param None.
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_saveConfig(void)
{
    // TODO
}

/**
 * @brief Restore the nRF24 component configuration.
 *
 * @todo Implement this function.
 *
 * @param None.
 *
 * @return None.
 *
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
 *
 * @param None.
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_setMode(const NrfMode mode)
{
    NRF_MODE_TX == mode ? `$INSTANCE_NAME`_setTxMode() :
                            `$INSTANCE_NAME`_setRxMode();
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
 *
 * @param None.
 *
 * @return None.
 *
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
 *
 * @param None.
 *
 * @return None.
 *
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
 *
 * @param None.
 *
 * @return None.
 *
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
 *
 * @param None.
 *
 * @return None.
 *
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
 *
 * @param None.
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_setTxMode(void)
{
    `$INSTANCE_NAME`_clearBit(NRF_CONFIG_REG, NRF_CONFIG_PRIM_RX);
}

/**
 * @brief Enable AutoACK in the given pipe.
 *
 *
 * @param pipe: Enable AutoACK in the given pipe.
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_enableAutoACK(const NrfDataPipe pipe)
{
    `$INSTANCE_NAME`_setBit(NRF_EN_AA_REG, pipe);
}

/**
 * @brief Disable AutoACK in the given pipe.
 *
 *
 * @param pipe: Disable AutoACK in the given pipe.
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_disableAutoACK(const NrfDataPipe pipe)
{
    `$INSTANCE_NAME`_clearBit(NRF_EN_AA_REG, pipe);
}

/**
 * @brief Set the channel where the radio will work.
 *
 *
 * @param uint8_t channel: Channel where the radio will work.
 *
 * @return None.
 *
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
 * @brief Set the address width common for all the pipes.
 *
 * @param const NrfSetupAddressWidth addr_width:
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_setPipesAddressWidth(
    const NrfSetupAddressWidth addr_width)
{
    `$INSTANCE_NAME`_writeRegister(NRF_SETUP_AW_REG, (uint8_t)addr_width);
}

/**
 * @brief Get the address width common for all the pipes.
 *
 * @param None.
 *
 * @return NrfSetupAddressWidth:
 *
 */
uint8_t `$INSTANCE_NAME`_getPipesAddressWidth(void)
{
    uint8_t reg = `$INSTANCE_NAME`_readRegister(NRF_SETUP_AW_REG);

    switch (reg) {
    case NRF_SETUP_AW_3BYTES:
        return 3;
        break;
    case NRF_SETUP_AW_4BYTES:
        return 4;
        break;
    case NRF_SETUP_AW_5BYTES:
        return 5;
        break;
    default:
        return 0;
        break;
    }
}

/**
 * @brief Set the Rx Address of the radio.
 *
 * This function configure the address of the Rx Pipe 0 of the radio.
 *
 * @param const uint8_t* addr:
 * @param size_t size:
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_setRxAddress(const uint8_t* addr, size_t size)
{
    `$INSTANCE_NAME`_writeLongRegister(NRF_RX_ADDR_P0_REG, addr, size);
}

/**
 * @brief Get the Rx Address of the radio.
 *
 * @param const uint8_t* addr:
 * @param size_t size:
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_getRxAddress(uint8_t* addr, size_t size)
{
    `$INSTANCE_NAME`_readLongRegister(NRF_RX_ADDR_P0_REG, addr, size);
}

/**
 * @brief Set the address of the RX Pipe 0 in the radio.
 *
 * This function configure the address of the Rx Pipe 0 of the radio.
 *
 * @param addr:
 * @param size:
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_setRxPipe0Address(const uint8_t* addr, size_t size)
{
    `$INSTANCE_NAME`_writeLongRegister(NRF_RX_ADDR_P0_REG, addr, size);
}

/**
 * @brief Get the address of the RX Pipe 0 in the radio.
 *
 * @param addr:
 * @param size:
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_getRxPipe0Address(uint8_t* addr, size_t size)
{
    `$INSTANCE_NAME`_readLongRegister(NRF_RX_ADDR_P0_REG, addr, size);
}

/**
 * @brief Set the address of the RX Pipe 1 in the radio.
 *
 * This function configure the address of the Rx Pipe 1 of the radio.
 *
 * @param addr:
 * @param size:
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_setRxPipe1Address(const uint8_t* addr, size_t size)
{
    `$INSTANCE_NAME`_writeLongRegister(NRF_RX_ADDR_P1_REG, addr, size);
}

/**
 * @brief Get the address of the RX Pipe 1 in the radio.
 *
 * @param addr:
 * @param size:
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_getRxPipe1Address(uint8_t* addr, size_t size)
{
    `$INSTANCE_NAME`_readLongRegister(NRF_RX_ADDR_P1_REG, addr, size);
}

/**
 * @brief Set the address of the RX Pipe 2 in the radio.
 *
 * This function configure the address of the Rx Pipe 2 of the radio.
 *
 * @param addr:
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_setRxPipe2Address(const uint8_t addr_lsb)
{
    `$INSTANCE_NAME`_writeRegister(NRF_RX_ADDR_P2_REG, addr_lsb);
}

/**
 * @brief Get the address of the RX Pipe 2 in the radio.
 *
 * @param addr:
 * @param size:
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_getRxPipe2Address(uint8_t* addr, size_t size)
{
    nRF24_readLongRegister(NRF_RX_ADDR_P1_REG, addr, size - 1);
    addr[size - 1] = nRF24_readRegister(NRF_RX_ADDR_P2_REG);
}

/**
 * @brief Set the address of the RX Pipe 3 in the radio.
 *
 * This function configure the address of the Rx Pipe 3 of the radio.
 *
 * @param addr:
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_setRxPipe3Address(const uint8_t addr_lsb)
{
    `$INSTANCE_NAME`_writeRegister(NRF_RX_ADDR_P3_REG, addr_lsb);
}

/**
 * @brief Get the address of the RX Pipe 3 in the radio.
 *
 * @param addr:
 * @param size:
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_getRxPipe3Address(uint8_t* addr, size_t size)
{
    nRF24_readLongRegister(NRF_RX_ADDR_P1_REG, addr, size - 1);
    addr[size - 1] = nRF24_readRegister(NRF_RX_ADDR_P3_REG);
}

/**
 * @brief Set the address of the RX Pipe 4 in the radio.
 *
 * This function configure the address of the Rx Pipe 4 of the radio.
 *
 * @param addr:
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_setRxPipe4Address(const uint8_t addr_lsb)
{
    `$INSTANCE_NAME`_writeRegister(NRF_RX_ADDR_P4_REG, addr_lsb);
}

/**
 * @brief Get the address of the RX Pipe 4 in the radio.
 *
 * @param addr:
 * @param size:
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_getRxPipe4Address(uint8_t* addr, size_t size)
{
    nRF24_readLongRegister(NRF_RX_ADDR_P1_REG, addr, size - 1);
    addr[size - 1] = nRF24_readRegister(NRF_RX_ADDR_P4_REG);
}

/**
 * @brief Set the address of the RX Pipe 5 in the radio.
 *
 * This function configure the address of the Rx Pipe 5 of the radio.
 *
 * @param const uint8_t* addr:
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_setRxPipe5Address(const uint8_t addr_lsb)
{
    `$INSTANCE_NAME`_writeRegister(NRF_RX_ADDR_P5_REG, addr_lsb);
}

/**
 * @brief Get the address of the RX Pipe 5 in the radio.
 *
 * @param addr:
 * @param size:
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_getRxPipe5Address(uint8_t* addr, size_t size)
{
    nRF24_readLongRegister(NRF_RX_ADDR_P1_REG, addr, size - 1);
    addr[size - 1] = nRF24_readRegister(NRF_RX_ADDR_P5_REG);
}

/**
 * @brief Set the TX Address of the radio.
 *
 *
 * @param const uint8_t* addr:
 * @param size_t size:
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_setTxAddress(const uint8_t* addr, size_t size)
{
    `$INSTANCE_NAME`_writeLongRegister(NRF_TX_ADDR_REG, addr, size);
}

/**
 * @brief Get the TX Address of the radio.
 *
 *
 * @param const uint8_t* addr:
 * @param size_t size:
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_getTxAddress(uint8_t* addr, size_t size)
{
    `$INSTANCE_NAME`_readLongRegister(NRF_TX_ADDR_REG, addr, size);
}

/**
 * @brief Set the payload size of the given pipe.
 *
 * Every pipe can have up to 32 bytes of payload, this function configure the
 * payload size of the given pipe.
 * It also checks that the parameter @size is not larger than 32 bytes.
 *
 * @param const NrfDataPipe pipe:
 * @param uint8_t size:
 *
 * @return None.
 *
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
 * @brief Get the payload size of the given pipe.
 *
 *
 * @param const NrfDataPipe pipe:
 *
 * @return uint8_t:
 *
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
 *
 * @param None.
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_PTX_reuseLastTransmittedPayload(void)
{
    `$INSTANCE_NAME`_PTX_ReuseTxPayloadCmd();
    `$INSTANCE_NAME`_transmitPulse();
}

/**
 * @brief Enable dynamic payload on the given pipe.
 *
 *
 * @param const NrfDataPipe pipe:
 *
 * @return None.
 *
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
 *
 * @param const NrfDataPipe pipe:
 *
 * @return None.
 *
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
 *
 *
 * @param None.
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_enableDynamicPayloadLength(void)
{
    `$INSTANCE_NAME`_setBit(NRF_FEATURE_REG, NRF_FEATURE_EN_DPL);
}

/**
 * @brief Disable dynamic payload length.
 *
 *
 * @param None.
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_disableDynamicPayloadLength(void)
{
    `$INSTANCE_NAME`_clearBit(NRF_FEATURE_REG, NRF_FEATURE_EN_DPL);
}

/**
 * @brief Enable payload with ACK.
 *
 *
 * @param None.
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_enablePayloadWithACK(void)
{
    `$INSTANCE_NAME`_setBit(NRF_FEATURE_REG, NRF_FEATURE_EN_ACK_PAY);
}

/**
 * @brief Disable Payload with ACK.
 *
 *
 * @param None.
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_disablePayloadWithACK(void)
{
    `$INSTANCE_NAME`_clearBit(NRF_FEATURE_REG, NRF_FEATURE_EN_ACK_PAY);
}

/**
 * @brief Enable dynamic payload length.
 *
 *
 * @param None.
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_enablePayloadWithNoACKCmd(void)
{
    `$INSTANCE_NAME`_setBit(NRF_FEATURE_REG, NRF_FEATURE_EN_DYN_ACK);
}

/**
 * @brief Disable Payload with no ACK.
 *
 * @param None.
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_disablePayloadWithNoACKCmd(void)
{
    `$INSTANCE_NAME`_clearBit(NRF_FEATURE_REG, NRF_FEATURE_EN_DYN_ACK);
}

/**
 * @brief Set the value of the CE pin to listen or not.
 *
 *
 * @param const bool listen: If @listen is set to true, then the radio will
 * listen, if set to false the radio will stop listening.
 *
 * @return None.
 *
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
 *
 * @param None.
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_startListening(void) {
    `$INSTANCE_NAME`_listen(true); }

/**
 * @brief The nRF24 radio will stop listening.
 *
 * This function the pin CE of the nRF24 radio will be set to logic low,
 * this disable the radio for listening.
 *
 * @param None.
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_stopListening(void) {
    `$INSTANCE_NAME`_listen(false); }

/**
 * @brief Transmit pulse on the CE pin.
 *
 * With this function the CE pin of the nRF24 radio will have a pulse of 15us,
 * this pulse trigger a transmission of the content of the TX FIFO.
 *
 * @param None.
 *
 * @return None.
 *
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
 * @param None.
 *
 * @return uint8_t: Content of the STATUS register of the radio.
 *
 */
uint8_t `$INSTANCE_NAME`_getStatus(void) { return `$INSTANCE_NAME`_NOPCmd(); }

/**
 * @brief Get the number of retransmissions.
 *
 *
 * @param None.
 *
 * @return uint8_t:
 *
 */
uint8_t `$INSTANCE_NAME`_getRetransmissionsCount(void)
{
    uint8_t count = `$INSTANCE_NAME`_readRegister(NRF_OBSERVE_TX_REG);
    return count & NRF_OBSERVE_TX_ARC_CNT_MASK;
    ;
}

/**
 * @brief Get the number of lost packets.
 *
 *
 * @param None.
 *
 * @return uint8_t:
 *
 */
uint8_t `$INSTANCE_NAME`_getLostPacketsCount(void)
{
    uint8_t lostPackets = `$INSTANCE_NAME`_readRegister(NRF_OBSERVE_TX_REG);
    lostPackets = lostPackets & NRF_OBSERVE_TX_PLOS_CNT_MASK;
    return lostPackets >> NRF_OBSERVE_TX_PLOS_CNT_POS;
}

/**
 * @brief
 *
 * @todo The nRF24 have a 3 level FIFO, with this function we only handle
 * 1 level of the FIFO.
 *
 * @param const uint8_t* data:
 * @param size_t size:
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_fillTxFIFO(const uint8_t* data, size_t size)
{
    `$INSTANCE_NAME`_WriteTXPayloadCmd(data, size);
}

/**
 * @brief
 *
 *
 * @param const uint8_t* data:
 * @param size_t size:
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_txTransmit(const uint8_t* data, size_t size)
{
    if (NULL == data) {
        return;
    }

    `$INSTANCE_NAME`_fillTxFIFO(data, size);
    `$INSTANCE_NAME`_transmitPulse();
}

/**
 * @brief
 *
 *
 * @param None.
 *
 * @return bool:
 *
 */
bool `$INSTANCE_NAME`_isDataReady(void)
{
    return NRF_STATUS_DATA_IS_RDY & `$INSTANCE_NAME`_getStatus();
}

/**
 * @brief
 *
 *
 * @param uint8_t* data:
 * @param const size_t size:
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_getRxPayload(uint8_t* data, const size_t size)
{
    `$INSTANCE_NAME`_PRX_ReadRXPayloadCmd(data, size);
}

/**
 * @brief
 *
 *
 * @param const uint8_t* data:
 * @param size_t size:
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_txTransmitWaitNoACK(const uint8_t* data, size_t size)
{
    `$INSTANCE_NAME`_PTX_NoACKPayloadCmd(data, size);
    `$INSTANCE_NAME`_transmitPulse();
}

/**
 * @brief
 *
 *
 * @param const NrfDataPipe pipe:
 * @param const uint8_t* data:
 * @param size_t size:
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_rxWritePayload(const NrfDataPipe pipe, uint8_t* data,
                                     size_t size)
{
    `$INSTANCE_NAME`_PRX_WriteACKPayloadCmd(pipe, data, size);
}

/**
 * @brief
 *
 *
 * @param None.
 *
 * @return uint8_t:
 *
 */
uint8_t `$INSTANCE_NAME`_getDataPipeWithPayload(void)
{
    uint8_t pipe = `$INSTANCE_NAME`_readRegister(NRF_STATUS_REG);
    return (pipe & 0x0Eu) >> 1;
}

/**
 * @brief
 *
 *
 * @param None.
 *
 * @return uint8_t:
 *
 */
uint8_t `$INSTANCE_NAME`_PRX_receivedPowerDetector(void)
{
    return `$INSTANCE_NAME`_readBit(NRF_RPD_REG, NRF_RPD_RPD);
}

/**
 * @brief
 *
 *
 * @param None.
 *
 * @return uint8_t:
 *
 */
uint8_t `$INSTANCE_NAME`_isTXFIFOFull(void)
{
    return `$INSTANCE_NAME`_readBit(NRF_FIFO_STATUS_REG,
                                    NRF_FIFO_STATUS_TX_FULL);
}

/**
 * @brief
 *
 *
 * @param None.
 *
 * @return uint8_t:
 *
 */
uint8_t `$INSTANCE_NAME`_isTXFIFOEmpty(void)
{
    return `$INSTANCE_NAME`_readBit(NRF_FIFO_STATUS_REG,
                                    NRF_FIFO_STATUS_TX_EMPTY);
}

/**
 * @brief
 *
 *
 * @param None.
 *
 * @return uint8_t:
 *
 */
uint8_t `$INSTANCE_NAME`_isRXFIFOFull(void)
{
    return `$INSTANCE_NAME`_readBit(NRF_FIFO_STATUS_REG,
                                    NRF_FIFO_STATUS_RX_FULL);
}

/**
 * @brief
 *
 *
 * @param None.
 *
 * @return uint8_t:
 *
 */
uint8_t `$INSTANCE_NAME`_isRXFIFOEmpty(void)
{
    return `$INSTANCE_NAME`_readBit(NRF_FIFO_STATUS_REG,
                                    NRF_FIFO_STATUS_RX_EMPTY);
}

/**
 * @brief Clear all IRQ flags.
 *
 * @param None.
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_clearAllIRQs(void)
{
    `$INSTANCE_NAME`_writeRegister(NRF_STATUS_REG, 0x70);
}

/**
 * @brief Clears the specific IRQ flag.
 *
 * Clear the flag writing a 1 to the interrupt flag bit.
 *
 * @param NrfIRQ irq_flag:
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_clearIRQFlag(const NrfIRQ irq_flag)
{
    `$INSTANCE_NAME`_writeRegister(NRF_STATUS_REG, (1 << irq_flag));
}

/**
 * @brief Get the IRQ flag that caused the interrupt.
 *
 * @param None.
 *
 * @return NrfIRQ:
 *
 */
NrfIRQ `$INSTANCE_NAME`_getIRQFlag(void)
{
    uint8_t sts = `$INSTANCE_NAME`_getStatus();

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
