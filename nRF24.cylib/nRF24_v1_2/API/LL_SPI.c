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
 * @brief Read the content of a nRF24 register.
 *
 * @param const NrfRegister reg: Register to be read.
 *
 * @return uint8_t: Content of the read register (reg).
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
 * @brief Read the content of a nRF24 register with more than 1byte of data.
 *
 * @param const NrfRegister reg: Register to be read.
 * @param uint8_t* data: Pointer to the array where the content of the register
 * will be
 * stored.
 * @param size_t size: Size of the register, larger register hold 5 bytes of
 * data.
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_readLongRegister(const NrfRegister reg, uint8_t* data,
                                       const size_t size)
{
    if (NULL == data) {
        return;
    }

    if (5 < size) {
        return;
    }

#if defined(CY_SCB_`$SPI_INTERFACE`_H) // SCB Block

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

#else // UDB Block

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

#endif
}

/**
 * @brief Write data to a nRF24 Register.
 *
 * @param const NrfRegister reg: Register where the data will be written.
 * @param const uint8_t data: Data to be written into the register.
 *
 * @return None.
 *
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
 * @brief
 *
 *
 * @param const NrfRegister reg:
 * @param const uint8_t* data:
 * @param size_t size:
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_writeLongRegister(const NrfRegister reg,
                                        const uint8_t* data, size_t size)
{
    if (NULL == data) {
        return;
    }

    if (5 < size) {
        size = 5;
    }

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

    while (
        !( `$SPI_INTERFACE`_ReadTxStatus() & `$SPI_INTERFACE`_STS_SPI_IDLE)) {
    }
    `$SS_PIN`_Write(1);

#endif
}

/**
 * Return 1 if bit is set (1), return 0 if the bit is clear (0).
 *
 * @param const NrfRegister reg:
 * @param uint8_t bit:
 *
 * @return uint8_t:
 *
 */
uint8_t `$INSTANCE_NAME`_readBit(const NrfRegister reg, uint8_t bit)
{
    return (`$INSTANCE_NAME`_readRegister(reg) & (1 << bit)) != 0;
}

/**
 * @brief
 *
 *
 * @param const NrfRegister reg:
 * @param const uint8_t bit:
 * @param const uint8_t value:
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_writeBit(const NrfRegister reg, const uint8_t bit,
                               const uint8_t value)
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
 * @brief
 *
 *
 * @param const NrfRegister reg:
 * @param const uint8_t bit:
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_clearBit(const NrfRegister reg, const uint8_t bit)
{
    `$INSTANCE_NAME`_writeBit(reg, bit, 0);
}

/**
 * @brief
 *
 *
 * @param const NrfRegister reg:
 * @param const uint8_t bit:
 *
 * @return None.
 *
 */
void `$INSTANCE_NAME`_setBit(const NrfRegister reg, const uint8_t bit)
{
    `$INSTANCE_NAME`_writeBit(reg, bit, 1);
}

/* [] END OF FILE */
