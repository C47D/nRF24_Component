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

#include "`$INSTANCE_NAME`_CONFIG.h"
#include "`$INSTANCE_NAME`_LL_SPI.h"

/**
 * Read the specified nRF24 register (1byte).
 *
 * @param const nrf_register reg: Register to be read.
 *
 * @return uint8_t: Content of the specified register.
 */
uint8_t `$INSTANCE_NAME`_read_register(const nrf_register reg)
{
    uint8_t data = 0;
#if defined (_PSOC6) // PSoC6
    Cy_SCB_ClearRxFifo(`$SPI_MASTER`_HW);
    Cy_SCB_ClearTxFifo(`$SPI_MASTER`_HW);

    Cy_GPIO_Clr(`$SS_PIN`_PORT, `$SS_PIN`_NUM);

    Cy_SCB_WriteArrayBlocking(`$SPI_MASTER`_HW, (uint8_t []){NRF_R_REGISTER_CMD | reg, NRF_NOP_CMD}, 2);

    while (Cy_SCB_IsTxComplete(`$SPI_MASTER`_HW) == false) {
    }
    CyDelayUs(1);

    Cy_GPIO_Set(`$SS_PIN`_PORT, `$SS_PIN`_NUM);

    (void)Cy_SCB_ReadRxFifo(`$SPI_MASTER`_HW);
    data = Cy_SCB_ReadRxFifo(`$SPI_MASTER`_HW);
#elif defined (_PSOC4_SCB) // PSoC4
    `$SPI_MASTER`_SpiUartClearRxBuffer();
    `$SPI_MASTER`_SpiUartClearTxBuffer();

    `$SS_PIN`_Write(0);
    `$SPI_MASTER`_SpiUartWriteTxData(NRF_R_REGISTER_CMD | reg);
    `$SPI_MASTER`_SpiUartWriteTxData(NRF_NOP_CMD);

    while (`$SPI_MASTER`_SpiUartGetRxBufferSize() != 2) {
    }
    `$SS_PIN`_Write(1);

    // This is the STATUS Register
    (void)`$SPI_MASTER`_SpiUartReadRxData();
    // This is the data we want
    data = `$SPI_MASTER`_SpiUartReadRxData();
#else // _PSOC_UDB
    `$SPI_MASTER`_ClearFIFO();

    `$SS_PIN`_Write(0);
    `$SPI_MASTER`_WriteTxData(NRF_R_REGISTER_CMD | reg);
    `$SPI_MASTER`_WriteTxData(NRF_NOP_CMD);

    while (!(`$SPI_MASTER`_ReadTxStatus() & `$SPI_MASTER`_STS_SPI_IDLE)) {
    }
    `$SS_PIN`_Write(1);

    // This is the STATUS Register
    (void)`$SPI_MASTER`_ReadRxData();
    // This is the data we want
    data = `$SPI_MASTER`_ReadRxData();
#endif
    return data;
}

/**
 * Read the specified nRF24 register (bigger than 1 byte).
 *
 * @param const nrf_register reg: Register to be read.
 * @param uint8_t* data: Pointer to where the content of the register
 * will be stored.
 * @param size_t size: Size of the register and data.
 */
void `$INSTANCE_NAME`_read_long_register(const nrf_register reg,
                                           uint8_t* data, const size_t size)
{
#if defined (_PSOC6) // PSoC6
    Cy_SCB_ClearRxFifo(`$SPI_MASTER`_HW);
    Cy_SCB_ClearTxFifo(`$SPI_MASTER`_HW);

    Cy_GPIO_Clr(`$SS_PIN`_PORT, `$SS_PIN`_NUM);
    Cy_SCB_Write(`$SPI_MASTER`_HW, NRF_R_REGISTER_CMD | reg);
    while (Cy_SCB_GetNumInRxFifo(`$SPI_MASTER`_HW) == 0) {
    }

    (void)Cy_SCB_ReadRxFifo(`$SPI_MASTER`_HW);

    for(size_t i = 0; i < size; i++){
        while(Cy_SCB_Write(`$SPI_MASTER`_HW, NRF_NOP_CMD) == 0);
        while (Cy_SCB_GetNumInRxFifo(`$SPI_MASTER`_HW) == 0) {}
        data[1] = Cy_SCB_ReadRxFifo(`$SPI_MASTER`_HW);
    }

    Cy_GPIO_Set(`$SS_PIN`_PORT, `$SS_PIN`_NUM);
#elif defined (_PSOC4_SCB) // PSoC4
    `$SPI_MASTER`_SpiUartClearRxBuffer();
    `$SPI_MASTER`_SpiUartClearTxBuffer();

    `$SS_PIN`_Write(0);
    `$SPI_MASTER`_SpiUartWriteTxData(NRF_R_REGISTER_CMD | reg);
    while (`$SPI_MASTER`_SpiUartGetRxBufferSize() == 0){
    }

    // Read the status register, just to clear the rx fifo
    `$SPI_MASTER`_SpiUartReadRxData();

    for (size_t i = 0; i < size; i++) {
        `$SPI_MASTER`_SpiUartWriteTxData(NRF_NOP_CMD);
        while (`$SPI_MASTER`_SpiUartGetRxBufferSize() == 0){}
        data[i] = `$SPI_MASTER`_SpiUartReadRxData();
    }
    `$SS_PIN`_Write(1);
#else // _PSOC_UDB
    `$SPI_MASTER`_ClearFIFO();

    `$SS_PIN`_Write(0);
    `$SPI_MASTER`_WriteTxData(NRF_R_REGISTER_CMD | reg);
    // Wait for the byte to be sent
    while (!(`$SPI_MASTER`_ReadTxStatus() & `$SPI_MASTER`_STS_BYTE_COMPLETE)) {
    }
    // Read the status register, just to clear the rx fifo
    `$SPI_MASTER`_ReadRxData();

    for (size_t i = 0; i < size; i++) {
        `$SPI_MASTER`_WriteTxData(NRF_NOP_CMD);
        while (!(`$SPI_MASTER`_ReadTxStatus() & `$SPI_MASTER`_STS_BYTE_COMPLETE)) {
        }
        data[i] = `$SPI_MASTER`_ReadRxData();
    }

    `$SS_PIN`_Write(1);
#endif
}

/**
 * Write to the specified nRF24 Register (1byte).
 *
 * @param const nrf_register reg: Register to be written.
 * @param const uint8_t data: Data to be written into the specified register.
 */
void `$INSTANCE_NAME`_write_register(const nrf_register reg, const uint8_t data)
{
#if defined (_PSOC6)
    Cy_SCB_ClearRxFifo(`$SPI_MASTER`_HW);
    Cy_SCB_ClearTxFifo(`$SPI_MASTER`_HW);

    Cy_GPIO_Clr(`$SS_PIN`_PORT, `$SS_PIN`_NUM);

    Cy_SCB_WriteArrayBlocking(`$SPI_MASTER`_HW, (uint8_t []){NRF_W_REGISTER_CMD | reg, data}, 2);

    while (Cy_SCB_IsTxComplete(`$SPI_MASTER`_HW) == false) {
    }
    CyDelayUs(1);

    Cy_GPIO_Set(`$SS_PIN`_PORT, `$SS_PIN`_NUM);
#elif defined (_PSOC4_SCB)
    `$SPI_MASTER`_SpiUartClearRxBuffer();
    `$SPI_MASTER`_SpiUartClearTxBuffer();

    `$SS_PIN`_Write(0);
    `$SPI_MASTER`_SpiUartWriteTxData(NRF_W_REGISTER_CMD | reg);
    `$SPI_MASTER`_SpiUartWriteTxData(data);

    while (`$SPI_MASTER`_SpiUartGetRxBufferSize() != 2) {
    }
    `$SS_PIN`_Write(1);
#else // _PSOC_UDB
    `$SPI_MASTER`_ClearFIFO();

    `$SS_PIN`_Write(0);
    `$SPI_MASTER`_WriteTxData(NRF_W_REGISTER_CMD | reg);
    `$SPI_MASTER`_WriteTxData(data);

    while (!(`$SPI_MASTER`_ReadTxStatus() & `$SPI_MASTER`_STS_SPI_IDLE)) {
    }
    `$SS_PIN`_Write(1);
#endif
}

/**
 * Write one or more bytes to the specified nRF24 Register.
 *
 * @param const nrf_register reg: Register to be written.
 * @param const uint8_t* data: Data to writen into the register.
 * @param size_t size: Bytes of the data to be written in the specified register.
 */
void `$INSTANCE_NAME`_write_long_register(const nrf_register reg,
                                            const uint8_t* data, const size_t size)
{
#if defined (_PSOC6)
    Cy_SCB_ClearRxFifo(`$SPI_MASTER`_HW);
    Cy_SCB_ClearTxFifo(`$SPI_MASTER`_HW);

    Cy_GPIO_Clr(`$SS_PIN`_PORT, `$SS_PIN`_NUM);
    Cy_SCB_SPI_Write(`$SPI_MASTER`_HW, NRF_W_REGISTER_CMD | reg);
    Cy_SCB_SPI_WriteArrayBlocking(`$SPI_MASTER`_HW, (void *) data, size);

    while (Cy_SCB_IsTxComplete(`$SPI_MASTER`_HW) == false) {
    }
    CyDelayUs(1);

    Cy_GPIO_Set(`$SS_PIN`_PORT, `$SS_PIN`_NUM);
#elif defined (_PSOC4_SCB)
    `$SPI_MASTER`_SpiUartClearRxBuffer();
    `$SPI_MASTER`_SpiUartClearTxBuffer();

    `$SS_PIN`_Write(0);

    `$SPI_MASTER`_SpiUartWriteTxData(NRF_W_REGISTER_CMD | reg);
    `$SPI_MASTER`_SpiUartPutArray(data, size);

    // we're not using the embedded SS pin on the SCB component, can't use the
    // SPI_Status function, we have to count the bytes on the RxBuffer to know
    // when the transition is done, size + 1 bytes == W_REGISTER_CMD + data
    while (`$SPI_MASTER`_SpiUartGetRxBufferSize() != (1 + size)) {
    }
    `$SS_PIN`_Write(1);
#else // _PSOC_UDB
    `$SPI_MASTER`_ClearFIFO();

    `$SS_PIN`_Write(0);
    `$SPI_MASTER`_WriteTxData(NRF_W_REGISTER_CMD | reg);
    for (size_t i = 0; i < size; i++) {
        `$SPI_MASTER`_WriteTxData(data[i]);
        while (!(`$SPI_MASTER`_ReadTxStatus() & `$SPI_MASTER`_STS_BYTE_COMPLETE)){
        }
    }

    while (!(`$SPI_MASTER`_ReadTxStatus() & `$SPI_MASTER`_STS_SPI_IDLE)) {
    }
    `$SS_PIN`_Write(1);
#endif
}

/**
 * Read the content of the specified bit of the specified nrf_register.
 *
 * @param const nrf_register reg: Register to be read.
 * @param uint8_t bit: Bit to be read.
 *
 * @return bool: Return the content of the bit.
 */
bool `$INSTANCE_NAME`_read_bit(const nrf_register reg, const uint8_t bit)
{
#if 0
    if (8 < bit) {
        return false; // how to deal with this?
    }
#endif

    return (`$INSTANCE_NAME`_read_register(reg) & (1 << bit)) != 0;
}

/**
 * Set (1) or clear (0) the specified bit of the specified nRF24 register.
 *
 * First we read the specified register and check the content of the specified
 * bit, exit early if the bit already is the value we wanted, otherwise we set
 * the bit to the specified value.
 *
 * @param const nrf_register reg: Register to be written.
 * @param const uint8_t bit: Position of the bit to be written.
 * @param const bool value: Value (1 or 0) to write into the bit.
 */
static void `$INSTANCE_NAME`_write_bit(const nrf_register reg,
                                  const uint8_t bit, const bool value)
{
    uint8_t temp = `$INSTANCE_NAME`_read_register(reg);

    // Check if the bit is 1
    if ((temp & (1 << bit)) != 0) {
        // it is 1, return if we wanted to set it to 1
        if (value) {
            return;
        }
    } else { // the bit is 0
        // it is 0, return if we wanted to set it to 0
        if (!value) {
            return;
        }
    }

    // Calculate the new value to be written into the register
    temp = value ? temp | (1 << bit) : temp & ~(1 << bit);

    `$INSTANCE_NAME`_write_register(reg, temp);
}

/**
 * Set to 0 the specified bit of the specified nRF24 register.
 *
 * @param const nrf_register reg: Register to be written.
 * @param const uint8_t bit: Bit to be written.
 */
void `$INSTANCE_NAME`_clear_bit(const nrf_register reg, const uint8_t bit)
{
    if (8 < bit) {
        return;
    }

    `$INSTANCE_NAME`_write_bit(reg, bit, 0);
}

/**
 * Set to 1 the specified bit of the specified nRF24 register.
 *
 * @param const nrf_register reg: Register to be written.
 * @param const uint8_t bit: Bit to be written.
 */
void `$INSTANCE_NAME`_set_bit(const nrf_register reg, const uint8_t bit)
{
    if (8 < bit) {
        return;
    }

    `$INSTANCE_NAME`_write_bit(reg, bit, 1);
}

/* [] END OF FILE */
