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
* @file `$INSTANCE_NAME`_LL_SPI.c
*
* @brief The nRF24 radio is controlled via SPI,
* this file have all the SPI communication between the
* PSoC and the nRF24 radio.
* 
*/

#include <`$CE_PIN`.h>
#include <`$SPI_INTERFACE`.h>
#include <`$SS_PIN`.h>

#if defined(CY_SCB_`$SPI_INTERFACE`_H)
#include <`$SPI_INTERFACE`_SPI_UART.h>
#endif

#include "`$INSTANCE_NAME`_LL_SPI.h"

/**
 * Read a 1 byte size nRF24 register.
 *
 * @param const NrfRegister reg: Register to be read.
 *
 * |    NrfRegister         |   Address |
 * |    :----:              |   :----:  |
 * |    NRF_CONFIG_REG      |   0x00    |
 * |    NRF_EN_AA_REG       |   0x01    |
 * |    NRF_EN_RXADDR_REG   |   0x02    |
 * |    NRF_SETUP_AW_REG    |   0x03    |
 * |    NRF_SETUP_RETR_REG  |   0x04    |
 * |    NRF_RF_CH_REG       |   0x05    |
 * |    NRF_RF_SETUP_REG    |   0x06    |
 * |    NRF_STATUS_REG      |   0x07    |
 * |    NRF_OBSERVE_TX_REG  |   0x08    |
 * |    NRF_RPD_REG         |   0x09    |
 * |    NRF_RX_ADDR_P0_REG  |   0x0A    |
 * |    NRF_RX_ADDR_P1_REG  |   0x0B    |
 * |    NRF_RX_ADDR_P2_REG  |   0x0C    |
 * |    NRF_RX_ADDR_P3_REG  |   0x0D    |
 * |    NRF_RX_ADDR_P4_REG  |   0x0E    |
 * |    NRF_RX_ADDR_P5_REG  |   0x0F    |
 * |    NRF_TX_ADDR_REG     |   0x10    |
 * |    NRF_RX_PW_P0_REG    |   0x11    |
 * |    NRF_RX_PW_P1_REG    |   0x12    |
 * |    NRF_RX_PW_P2_REG    |   0x13    |
 * |    NRF_RX_PW_P3_REG    |   0x14    |
 * |    NRF_RX_PW_P4_REG    |   0x15    |
 * |    NRF_RX_PW_P5_REG    |   0x16    |
 * |    NRF_FIFO_STATUS_REG |   0x17    |
 * |    NRF_DYNPD_REG       |   0x1C    |
 * |    NRF_FEATURE_REG     |   0x1D    |
 *
 * @return uint8_t: Content of the register.
 *
 */
uint8_t `$INSTANCE_NAME`_readRegister(const NrfRegister reg)
{
#if defined(CY_SCB_`$SPI_INTERFACE`_H) // SCB Block

    `$SPI_INTERFACE`_SpiUartClearRxBuffer();
    `$SPI_INTERFACE`_SpiUartClearTxBuffer();

    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_SpiUartWriteTxData(NRF_R_REGISTER_CMD | reg);
    `$SPI_INTERFACE`_SpiUartWriteTxData(NRF_NOP_CMD);

    while (`$SPI_INTERFACE`_SpiUartGetRxBufferSize() != 2) {
    }
    `$SS_PIN`_Write(1);

    // This is the STATUS Register
    (void)`$SPI_INTERFACE`_SpiUartReadRxData();
    // This is the data we want
    return `$SPI_INTERFACE`_SpiUartReadRxData();

#else // UDB Block

    `$SPI_INTERFACE`_ClearRxBuffer();
    `$SPI_INTERFACE`_ClearTxBuffer();

    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_WriteTxData(NRF_R_REGISTER_CMD | reg);
    `$SPI_INTERFACE`_WriteTxData(NRF_NOP_CMD);

    while (!(`$SPI_INTERFACE`_ReadTxStatus() & `$SPI_INTERFACE`_STS_SPI_IDLE)) {
    }
    `$SS_PIN`_Write(1);

    // This is the STATUS Register
    (void)`$SPI_INTERFACE`_ReadRxData();
    // This is the data we want
    return `$SPI_INTERFACE`_ReadRxData();

#endif
}

/**
 * Read a more than 1 byte size nRF24 register.
 *
 * @param const NrfRegister reg: Register to be read.
 * @param uint8_t* data: Pointer to where the content of the register
 * will be stored.
 * @param size_t size: Size of the register, larger register hold 5 bytes of
 * data.
 *
 * @return
 *
 */
void `$INSTANCE_NAME`_readLongRegister(const NrfRegister reg,
                                        uint8_t* data,
                                       const size_t size)
{
#if defined(CY_SCB_`$SPI_INTERFACE`_H) // SCB Block

    #if 0
    `$SPI_INTERFACE`_SpiUartClearRxBuffer();
    `$SPI_INTERFACE`_SpiUartClearTxBuffer();

    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_SpiUartWriteTxData(NRF_R_REGISTER_CMD | reg);

    for (size_t i = 0; i < size; i++) {
        `$SPI_INTERFACE`_SpiUartWriteTxData(NRF_NOP_CMD);
    }

    while (`$SPI_INTERFACE`_SpiUartGetRxBufferSize() != (1 + size)) {
    }
    `$SS_PIN`_Write(1);

    // This is the STATUS Register
    (void)`$SPI_INTERFACE`_SpiUartReadRxData();
    // This is the data we want
    for (size_t j = 0; j < size; j++) {
        data[j] = `$SPI_INTERFACE`_SpiUartReadRxData();
    }
    
    #else
        
    SPI_SpiUartClearRxBuffer();
    SPI_SpiUartClearTxBuffer();
    
    SS_Write(0);
    
    SPI_SpiUartWriteTxData(NRF_R_REGISTER_CMD | NRF_RX_ADDR_P4_REG);
    while (SPI_SpiUartGetRxBufferSize() == 0){}
    
    // Read the status register, just to clear the rx fifo
    SPI_SpiUartReadRxData();
    //SPI_SpiUartClearRxBuffer();
    
    for (size_t i = 0; i < size; i++) {
        SPI_SpiUartWriteTxData(NRF_NOP_CMD);
        while (SPI_SpiUartGetRxBufferSize() == 0){}
        data[i] = SPI_SpiUartReadRxData();
    }

    SS_Write(1);
        
    #endif

#else // UDB Block
    
    #if 0

    `$SPI_INTERFACE`_ClearRxBuffer();
    `$SPI_INTERFACE`_ClearTxBuffer();

    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_WriteTxData(NRF_R_REGISTER_CMD | reg);

    for (size_t i = 0; i < size; i++) {
        `$SPI_INTERFACE`_WriteTxData(NRF_NOP_CMD);
    }

    while (!(`$SPI_INTERFACE`_ReadTxStatus() & `$SPI_INTERFACE`_STS_SPI_IDLE)) {
    }
    `$SS_PIN`_Write(1);

    // This is the STATUS Register
    (void)`$SPI_INTERFACE`_ReadRxData();
    // This is the data we want
    for (size_t j = 0; j < size; j++) {
        data[j] = `$SPI_INTERFACE`_ReadRxData();
    }
    
    #else

    SPI_ClearFIFO();

    SS_Write(0);
    SPI_WriteTxData(NRF_R_REGISTER_CMD | reg);

    // Wait for the byte to be sent
    while (!(SPI_ReadTxStatus() & SPI_STS_BYTE_COMPLETE)) {}

    // Read the status register, just to clear the rx fifo
    SPI_ReadRxData();
    
    for (size_t i = 0; i < size; i++) {
        SPI_WriteTxData(NRF_NOP_CMD);
        while (!(SPI_ReadTxStatus() & SPI_STS_BYTE_COMPLETE)) {}
        data[i] = SPI_ReadRxData();
    }

    SS_Write(1);

    #endif

#endif
}

/**
 * Write 1 byte to a nRF24 Register.
 *
 * @param const NrfRegister reg: Register to be written.
 *
 * |    NrfRegister         |   Address |
 * |    :----:              |   :----:  |
 * |    NRF_CONFIG_REG      |   0x00    |
 * |    NRF_EN_AA_REG       |   0x01    |
 * |    NRF_EN_RXADDR_REG   |   0x02    |
 * |    NRF_SETUP_AW_REG    |   0x03    |
 * |    NRF_SETUP_RETR_REG  |   0x04    |
 * |    NRF_RF_CH_REG       |   0x05    |
 * |    NRF_RF_SETUP_REG    |   0x06    |
 * |    NRF_STATUS_REG      |   0x07    |
 * |    NRF_OBSERVE_TX_REG  |   0x08    |
 * |    NRF_RPD_REG         |   0x09    |
 * |    NRF_RX_ADDR_P0_REG  |   0x0A    |
 * |    NRF_RX_ADDR_P1_REG  |   0x0B    |
 * |    NRF_RX_ADDR_P2_REG  |   0x0C    |
 * |    NRF_RX_ADDR_P3_REG  |   0x0D    |
 * |    NRF_RX_ADDR_P4_REG  |   0x0E    |
 * |    NRF_RX_ADDR_P5_REG  |   0x0F    |
 * |    NRF_TX_ADDR_REG     |   0x10    |
 * |    NRF_RX_PW_P0_REG    |   0x11    |
 * |    NRF_RX_PW_P1_REG    |   0x12    |
 * |    NRF_RX_PW_P2_REG    |   0x13    |
 * |    NRF_RX_PW_P3_REG    |   0x14    |
 * |    NRF_RX_PW_P4_REG    |   0x15    |
 * |    NRF_RX_PW_P5_REG    |   0x16    |
 * |    NRF_FIFO_STATUS_REG |   0x17    |
 * |    NRF_DYNPD_REG       |   0x1C    |
 * |    NRF_FEATURE_REG     |   0x1D    |
 *
 * @param const uint8_t data: Data to be written into the register.
 *
 * @return
 */
void `$INSTANCE_NAME`_writeRegister(const NrfRegister reg, const uint8_t data)
{
#if defined(CY_SCB_`$SPI_INTERFACE`_H) // SCB Block

    `$SPI_INTERFACE`_SpiUartClearRxBuffer();
    `$SPI_INTERFACE`_SpiUartClearTxBuffer();

    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_SpiUartWriteTxData(NRF_W_REGISTER_CMD | reg);
    `$SPI_INTERFACE`_SpiUartWriteTxData(data);

    while (`$SPI_INTERFACE`_SpiUartGetRxBufferSize() != 2) {
    }
    `$SS_PIN`_Write(1);

#else // UDB Block

    `$SPI_INTERFACE`_ClearRxBuffer();
    `$SPI_INTERFACE`_ClearTxBuffer();

    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_WriteTxData(NRF_W_REGISTER_CMD | reg);
    `$SPI_INTERFACE`_WriteTxData(data);

    while (!(`$SPI_INTERFACE`_ReadTxStatus() & `$SPI_INTERFACE`_STS_SPI_IDLE)) {
    }
    `$SS_PIN`_Write(1);

#endif
}

/**
 * Write more than 1 byte to a nRF24 Register, larger register can hold up to
 * 5 bytes.
 *
 * @param const NrfRegister reg:
 * @param const uint8_t* data:
 * @param size_t size: Size (in bytes) of the data to be written.
 *
 * @return
 */
void `$INSTANCE_NAME`_writeLongRegister(const NrfRegister reg,
                                        const uint8_t* data,
                                        const size_t size)
{
#if defined(CY_SCB_`$SPI_INTERFACE`_H) // SCB Block

    `$SPI_INTERFACE`_SpiUartClearRxBuffer();
    `$SPI_INTERFACE`_SpiUartClearTxBuffer();

    `$SS_PIN`_Write(0);

    `$SPI_INTERFACE`_SpiUartWriteTxData(NRF_W_REGISTER_CMD | reg);
    `$SPI_INTERFACE`_SpiUartPutArray(data, size);

    // Wait for the RxBuffer to have dataSize + 1 bytes,
    // dataSize + 1 because we need to count the Command + Register
    // byte at the beginning of the transaction.
    while (`$SPI_INTERFACE`_SpiUartGetRxBufferSize() != (1 + size)) {
    }
    `$SS_PIN`_Write(1);

#else // UDB Block

    `$SPI_INTERFACE`_ClearRxBuffer();
    `$SPI_INTERFACE`_ClearTxBuffer();

    `$SS_PIN`_Write(0);
    `$SPI_INTERFACE`_WriteTxData(NRF_W_REGISTER_CMD | reg);
    `$SPI_INTERFACE`_PutArray(data, size);

    while (!(`$SPI_INTERFACE`_ReadTxStatus() & `$SPI_INTERFACE`_STS_SPI_IDLE)){
    }
    `$SS_PIN`_Write(1);

#endif
}

/**
 * Read a given bit of a given nRF24 register.
 *
 * @param const NrfRegister reg:
 * @param uint8_t bit:
 *
 * @return uint8_t: Return 1 if bit is set (logic 1), return 0 if the bit
 * is clear (logic 0).
 */
uint8_t `$INSTANCE_NAME`_readBit(const NrfRegister reg, const uint8_t bit)
{
    return (`$INSTANCE_NAME`_readRegister(reg) & (1 << bit)) != 0;
}

/**
 * Write a boolean value to a given bit of a given nRF24 register.
 *
 *
 * @param const NrfRegister reg:
 * @param const uint8_t bit:
 * @param const bool value:
 *
 * @return
 */
void `$INSTANCE_NAME`_writeBit(const NrfRegister reg,
                                const uint8_t bit,
                               const bool value)
{
    // Get the @reg content
    uint8_t temp = `$INSTANCE_NAME`_readRegister(reg);

    // Check if @bit is already set
    if ((temp & (1 << bit)) != 0) {
        // Return as we wanted to set it ( @value == 1 ),
        // continue if we wanted to clear it ( @value == 0 )
        if (value)
            return;
    }

    // calculate the new value to write back to @reg, if @value is != 0, then
    // we set the bit, if @value == 0, then we clear the bit
    temp = value ? temp | (1 << bit) : temp & ~(1 << bit);

    // Write back to @reg
    `$INSTANCE_NAME`_writeRegister(reg, temp);
}

/**
 * Set a given bit of a given nRF24 register to logic 0.
 *
 * @param const NrfRegister reg:
 * @param const uint8_t bit:
 *
 * @return
 */
void `$INSTANCE_NAME`_clearBit(const NrfRegister reg, const uint8_t bit)
{
    `$INSTANCE_NAME`_writeBit(reg, bit, 0);
}

/**
 * Set a given bit of a given nRF24 register to logic 1.
 *
 * @param const NrfRegister reg:
 * @param const uint8_t bit:
 *
 * @return
 */
void `$INSTANCE_NAME`_setBit(const NrfRegister reg, const uint8_t bit)
{
    `$INSTANCE_NAME`_writeBit(reg, bit, 1);
}

/* [] END OF FILE */
