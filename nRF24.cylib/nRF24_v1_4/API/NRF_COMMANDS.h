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
* @file `$INSTANCE_NAME`_NRF_COMMANDS.h
*
* @brief The nRF24 radio is controlled via commands, this file implement all
* the available commands.
*/

#ifndef `$INSTANCE_NAME`_NRF_COMMANDS_H
#define `$INSTANCE_NAME`_NRF_COMMANDS_H

#include <cytypes.h>
#include <cyfitter.h>

#include <stdbool.h>
#include <stddef.h>

#include "`$INSTANCE_NAME`_NRF_REGS.h"

void `$INSTANCE_NAME`_sendCommand(const NrfCmd cmd);
void `$INSTANCE_NAME`_PTX_ReuseTxPayloadCmd(void);
void `$INSTANCE_NAME`_PRX_ReadRXPayloadCmd(uint8_t* data, const size_t size);
void `$INSTANCE_NAME`_WriteTXPayloadCmd(const uint8_t* data, const size_t size);
void `$INSTANCE_NAME`_FlushRxCmd(void);
void `$INSTANCE_NAME`_FlushTxCmd(void);
uint8_t `$INSTANCE_NAME`_ReadPayloadWidthCmd(void);
void `$INSTANCE_NAME`_PRX_WriteACKPayloadCmd(const NrfDataPipe pipe,
                                const uint8_t* data, const size_t size);
void `$INSTANCE_NAME`_PTX_NoACKPayloadCmd(const uint8_t* data, const size_t size);
uint8_t `$INSTANCE_NAME`_NOPCmd(void);

#endif /* `$INSTANCE_NAME`_NRF_COMMANDS_H */

/* [] END OF FILE */
