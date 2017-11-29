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
 * Read a (1 byte long) nRF24 register.
 *
 * @param const nrf_register reg: Register to be read.
 *
 * @return uint8_t: Content of the register.
 */
uint8_t `$INSTANCE_NAME`_read_register(const nrf_register reg)
{
    uint8_t data = 0;
#if defined(CY_SCB_`$SPI_MASTER`_H) // SCB Block
#if defined(`$SPI_MASTER`_CY_SCB_SPI_PDL_H) // PSoC6
    Cy_GPIO_Clr(`$SS_PIN`_PORT, `$SS_PIN`_NUM);
    
    Cy_SCB_SPI_Write(`$SPI_MASTER`_HW, NRF_R_REGISTER_CMD | reg);
    Cy_SCB_SPI_Write(`$SPI_MASTER`_HW, NRF_NOP_CMD);
    
    while (Cy_SCB_SPI_GetNumInRxFifo(`$SPI_MASTER`_HW) != 2) {
    }
    
    Cy_GPIO_Set(`$SS_PIN`_PORT, `$SS_PIN`_NUM);
    
    (void)Cy_SCB_SPI_Read(`$SPI_MASTER`_HW);
    data = Cy_SCB_SPI_Read(`$SPI_MASTER`_HW);
#else // PSoC4
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
#endif
#else // UDB Block
    `$SPI_MASTER`_ClearRxBuffer();
    `$SPI_MASTER`_ClearTxBuffer();

    `$SS_PIN`_Write(0);
    `$SPI_MASTER`_WriteTxData(NRF_R_REGISTER_CMD | reg);
    `$SPI_MASTER`_WriteTxData(NRF_NOP_CMD);

    while (!(`$SPI_MASTER`_ReadTxStatus() & `$SPI_MASTER`_STS_BYTE_COMPLETE)) {
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
 * Read a (multi byte long) nRF24 register.
 *
 * @param const nrf_register reg: Register to be read.
 * @param uint8_t* data: Pointer to where the content of the register
 * will be stored.
 * @param size_t size: Size of the register, larger register hold 5 bytes of
 * data.
 */
void `$INSTANCE_NAME`_read_long_register(const nrf_register reg, uint8_t* data,
                                       const size_t size)
{
#if defined(CY_SCB_`$SPI_MASTER`_H) // SCB Block
#if defined(`$SPI_MASTER`_CY_SCB_SPI_PDL_H) // PSoC6
    Cy_GPIO_Clr(`$SS_PIN`_PORT, `$SS_PIN`_NUM);
    Cy_SCB_SPI_Write(`$SPI_MASTER`_HW, NRF_R_REGISTER_CMD | reg);
    while (Cy_SCB_SPI_GetNumInRxFifo(`$SPI_MASTER`_HW) == 0) {
    }
    
    (void)Cy_SCB_SPI_Read(`$SPI_MASTER`_HW);
    for(size_t i = 0; i < size; i++){
        Cy_SCB_SPI_Write(`$SPI_MASTER`_HW, NRF_NOP_CMD);
        while (Cy_SCB_SPI_GetNumInRxFifo(`$SPI_MASTER`_HW) == 0) {}
        data[1] = Cy_SCB_SPI_Read(`$SPI_MASTER`_HW);
    }
    
    Cy_GPIO_Set(`$SS_PIN`_PORT, `$SS_PIN`_NUM);
#else // PSoC4
    `$SPI_MASTER`_SpiUartClearRxBuffer();
    `$SPI_MASTER`_SpiUartClearTxBuffer();

    `$SS_PIN`_Write(0);
    `$SPI_MASTER`_SpiUartWriteTxData(NRF_R_REGISTER_CMD | reg);
    while (`$SPI_MASTER`_SpiUartGetRxBufferSize() == 0){
    }
    
    // Read the status register, just to clear the rx fifo
    `$SPI_MASTER`_SpiUartReadRxData();
    //SPI_SpiUartClearRxBuffer();
    
    for (size_t i = 0; i < size; i++) {
        `$SPI_MASTER`_SpiUartWriteTxData(NRF_NOP_CMD);
        while (`$SPI_MASTER`_SpiUartGetRxBufferSize() == 0){}
        data[i] = `$SPI_MASTER`_SpiUartReadRxData();
    }
    `$SS_PIN`_Write(1);
#endif
#else // UDB Block
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
 * Write one byte to a nRF24 Register.
 *
 * @param const nrf_register reg: Register to be written.
 * @param const uint8_t data: Data to be written into the register.
 */
void `$INSTANCE_NAME`_write_register(const nrf_register reg, const uint8_t data)
{
#if defined(CY_SCB_`$SPI_MASTER`_H) // SCB Block
#if defined(`$SPI_MASTER`_CY_SCB_SPI_PDL_H) // PSoC6
    Cy_GPIO_Clr(`$SS_PIN`_PORT, `$SS_PIN`_NUM);
    Cy_SCB_SPI_Write(`$SPI_MASTER`_HW, NRF_W_REGISTER_CMD | reg);
    Cy_SCB_SPI_Write(`$SPI_MASTER`_HW, data);
    
    while (Cy_SCB_SPI_GetNumInRxFifo(`$SPI_MASTER`_HW) != 2) {
    }
    
    Cy_GPIO_Set(`$SS_PIN`_PORT, `$SS_PIN`_NUM);
#else // PSoC4
    `$SPI_MASTER`_SpiUartClearRxBuffer();
    `$SPI_MASTER`_SpiUartClearTxBuffer();

    `$SS_PIN`_Write(0);
    `$SPI_MASTER`_SpiUartWriteTxData(NRF_W_REGISTER_CMD | reg);
    `$SPI_MASTER`_SpiUartWriteTxData(data);

    while (`$SPI_MASTER`_SpiUartGetRxBufferSize() != 2) {
    }
    `$SS_PIN`_Write(1);
#endif
#else // UDB Block
    `$SPI_MASTER`_ClearRxBuffer();
    `$SPI_MASTER`_ClearTxBuffer();

    `$SS_PIN`_Write(0);
    `$SPI_MASTER`_WriteTxData(NRF_W_REGISTER_CMD | reg);
    `$SPI_MASTER`_WriteTxData(data);

    while (!(`$SPI_MASTER`_ReadTxStatus() & `$SPI_MASTER`_STS_BYTE_COMPLETE)) {
    }
    `$SS_PIN`_Write(1);
#endif
}

/**
 * Write one or more bytes to a nRF24 Register, larger register is 5 bytes.
 *
 * @param const nrf_register reg: Register to be written.
 * @param const uint8_t* data: Data to write into the register.
 * @param size_t size: Size (in bytes) of the data to be written.
 */
void `$INSTANCE_NAME`_write_long_register(const nrf_register reg, const uint8_t* data,
                                        const size_t size)
{
#if defined(CY_SCB_`$SPI_MASTER`_H) // SCB Block
#if defined(`$SPI_MASTER`_CY_SCB_SPI_PDL_H) // PSoC6
    Cy_GPIO_Clr(`$SS_PIN`_PORT, `$SS_PIN`_NUM);
    Cy_SCB_SPI_Write(`$SPI_MASTER`_HW, NRF_W_REGISTER_CMD | reg);
    Cy_SCB_SPI_WriteArray(`$SPI_MASTER`_HW, data, size);
    
    while (Cy_SCB_SPI_GetNumInRxFifo(`$SPI_MASTER`_HW) != (1 + size)) {
    }
    
    Cy_GPIO_Set(`$SS_PIN`_PORT, `$SS_PIN`_NUM);
#else // PSoC4
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
#endif
#else // UDB Block
    `$SPI_MASTER`_ClearRxBuffer();
    `$SPI_MASTER`_ClearTxBuffer();

    `$SS_PIN`_Write(0);
    `$SPI_MASTER`_WriteTxData(NRF_W_REGISTER_CMD | reg);
    `$SPI_MASTER`_PutArray(data, size);

    while (!(`$SPI_MASTER`_ReadTxStatus() & `$SPI_MASTER`_STS_BYTE_COMPLETE)) {
    }
    `$SS_PIN`_Write(1);
#endif
}

/**
 * Read a bit of a nrf_register.
 *
 * @param const nrf_register reg: Register to be read.
 * @param uint8_t bit: Bit to be read.
 *
 * @return bool: Return 1 if bit is set (logic 1), return 0 if the bit
 * is clear (logic 0).
 */
bool `$INSTANCE_NAME`_read_bit(const nrf_register reg, const uint8_t bit)
{
    return (`$INSTANCE_NAME`_read_register(reg) & (1 << bit)) != 0;
}

/**
 * Set (logic 1) or clear (logic 0) a given bit of a given nRF24 register.
 *
 * Before setting the value we want to write to the bit we first check it's value,
 * if the bit have the value we wanted already, we exit the function early.
 *
 * @param const nrf_register reg: Register to be written.
 * @param const uint8_t bit: Position of the bit to be written.
 * @param const bool value: Value (Logic 1 or 0) to write into the bit.
 */
void `$INSTANCE_NAME`_write_bit(const nrf_register reg, const uint8_t bit,
                               const bool value)
{
    uint8_t temp = `$INSTANCE_NAME`_read_register(reg);

    // Check if the bit of interest is set
    if ((temp & (1 << bit)) != 0) {
        // Return if we wanted to set it, continue if we wanted to clear it.
        if (value) {
            return;
        }
    } else { // the bit is clear
        if (!value) {
            return; // return if we wanted to clear the bit
        }
    }

    // Calculate the new value to be written in the register
    temp = value ? temp | (1 << bit) : temp & ~(1 << bit);

    `$INSTANCE_NAME`_write_register(reg, temp);
}

/**
 * Clear (set to 0) a given bit of a given nRF24 register.
 *
 * @param const nrf_register reg: Register to be written.
 * @param const uint8_t bit: Bit to be written.
 */
void `$INSTANCE_NAME`_clear_bit(const nrf_register reg, const uint8_t bit)
{
    `$INSTANCE_NAME`_write_bit(reg, bit, 0);
}

/**
 * Set (set to 1) a given bit of a given nRF24 register.
 *
 * @param const nrf_register reg: Register to be written.
 * @param const uint8_t bit: Bit to be written.
 */
void `$INSTANCE_NAME`_set_bit(const nrf_register reg, const uint8_t bit)
{
    `$INSTANCE_NAME`_write_bit(reg, bit, 1);
}

/* [] END OF FILE */
