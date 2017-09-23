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
* @brief The nRF24 radio is controlled via SPI, this file have all the SPI
* communication between the PSoC and the nRF24 radio.
*/

#include <`$CE_PIN`.h>
#include <`$SPI_INTERFACE`.h>
#include <`$SS_PIN`.h>

#if defined(CY_SCB_`$SPI_INTERFACE`_H)
#include <`$SPI_INTERFACE`_SPI_UART.h>
#endif

#include "`$INSTANCE_NAME`_LL_SPI.h"

/**
 * Read 1byte of an nRF24 register.
 *
 * @param const NrfRegister reg: Register to be read.
 *
 * @return uint8_t: Content of the register.
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
 * Read more than 1byte of a nRF24 register.
 *
 * @param const NrfRegister reg: Register to be read.
 * @param uint8_t* data: Pointer to where the content of the register
 * will be stored.
 * @param size_t size: Size of the register, larger register hold 5 bytes of
 * data.
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
 * Write 1byte to a nRF24 Register.
 *
 * @param const NrfRegister reg: Register to be written.
 * @param const uint8_t data: Data to be written into the register.
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
 * Write more than 1byte to a nRF24 Register, larger register is 5 bytes.
 *
 * @param const NrfRegister reg: Register to be written.
 * @param const uint8_t* data: Data to write into the register.
 * @param size_t size: Size (in bytes) of the data to be written.
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
 * Read the bit state of a NrfRegister.
 *
 * @param const NrfRegister reg: Register to be read.
 * @param uint8_t bit: Bit to be read.
 *
 * @return bool: Return 1 if bit is set (logic 1), return 0 if the bit
 * is clear (logic 0).
 */
bool `$INSTANCE_NAME`_readBit(const NrfRegister reg, const uint8_t bit)
{
    return (`$INSTANCE_NAME`_readRegister(reg) & (1 << bit)) != 0;
}

/**
 * Set (logic 1) or clear (logic 0) a given bit of a given nRF24 register.
 *
 * @param const NrfRegister reg: Register to be written.
 * @param const uint8_t bit: Position of the bit to be written.
 * @param const bool value: Value (Logic 1 or 0) to write into the bit.
 */
void `$INSTANCE_NAME`_writeBit(const NrfRegister reg, const uint8_t bit,
                               const bool value)
{
    uint8_t temp = `$INSTANCE_NAME`_readRegister(reg);

    // Check if the bit of interest is set
    if ((temp & (1 << bit)) != 0) {
        // Return if we wanted to set it, continue if we wanted to clear it.
        if (value) {
            return;
        }
    }

    // Calculate the new value to be written in the register
    temp = value ? temp | (1 << bit) : temp & ~(1 << bit);

    `$INSTANCE_NAME`_writeRegister(reg, temp);
}

/**
 * Clear (logic 0) a given bit of a given nRF24 register.
 *
 * @param const NrfRegister reg: Register to be written.
 * @param const uint8_t bit: Bit to be written.
 */
void `$INSTANCE_NAME`_clearBit(const NrfRegister reg, const uint8_t bit)
{
    `$INSTANCE_NAME`_writeBit(reg, bit, 0);
}

/**
 * Set (logic 1) a given bit of a given nRF24 register.
 *
 * @param const NrfRegister reg: Register to be written.
 * @param const uint8_t bit: Bit to be written.
 */
void `$INSTANCE_NAME`_setBit(const NrfRegister reg, const uint8_t bit)
{
    `$INSTANCE_NAME`_writeBit(reg, bit, 1);
}

/* [] END OF FILE */
