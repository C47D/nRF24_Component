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

#ifndef `$INSTANCE_NAME`_FUNCS_H
#define `$INSTANCE_NAME`_FUNCS_H

#include <cytypes.h>
#include <cyfitter.h>

#include <stdbool.h>
#include <stddef.h>

#include "`$INSTANCE_NAME`_REGS.h"

// PSoC Component Functions
void `$INSTANCE_NAME`_Start(void);
void `$INSTANCE_NAME`_Init(void);
void `$INSTANCE_NAME`_Enable(void);
void `$INSTANCE_NAME`_Stop(void);
void `$INSTANCE_NAME`_Sleep(void);
void `$INSTANCE_NAME`_Wakeup(void);
void `$INSTANCE_NAME`_SaveConfig(void);
void `$INSTANCE_NAME`_RestoreConfig(void);

// nRF24 Functions

// Configuration functions
void `$INSTANCE_NAME`_SetMode(const NrfMode mode);
void `$INSTANCE_NAME`_SetPowerDownMode(void);
void `$INSTANCE_NAME`_SetStandbyIMode(void);
void `$INSTANCE_NAME`_SetStandbyIIMode(void);
void `$INSTANCE_NAME`_SetRxMode(void);
void `$INSTANCE_NAME`_SetTxMode(void);

// CONFIG Reg
// EN_AA_Reg
void `$INSTANCE_NAME`_EnableAutoACK(const NrfDataPipe pipe);
void `$INSTANCE_NAME`_DisableAutoACK(const NrfDataPipe pipe);
// EN_RXADDR Reg
// SETUP_AW Reg
// SETUP_RETR Reg
// RF_CH Reg
void `$INSTANCE_NAME`_SetChannel(uint8_t channel);
// RF_SETUP Reg
// STATUS Reg

// RX ADDR P0 Reg
void `$INSTANCE_NAME`_SetRxAddress(const uint8_t* addr, size_t size);
void `$INSTANCE_NAME`_SetRxPipe0Address(const uint8_t* addr, size_t size);
// RX ADDR P1 Reg
void `$INSTANCE_NAME`_SetRxPipe1Address(const uint8_t* addr, size_t size);
// RX ADDR P2 Reg
void `$INSTANCE_NAME`_SetRxPipe2Address(const uint8_t addr);
// RX ADDR P3 Reg
void `$INSTANCE_NAME`_SetRxPipe3Address(const uint8_t addr);
// RX ADDR P4 Reg
void `$INSTANCE_NAME`_SetRxPipe4Address(const uint8_t addr);
// RX ADDR P5 Reg
void `$INSTANCE_NAME`_SetRxPipe5Address(const uint8_t addr);
// TX_ADDR Reg
void `$INSTANCE_NAME`_SetTxAddress(const uint8_t* addr, size_t size);
// RX_PW_P0 Reg
// RX_PW_P1 Reg
// RX_PW_P2 Reg
// RX_PW_P3 Reg
// RX_PW_P4 Reg
// RX_PW_P5 Reg
void `$INSTANCE_NAME`_SetPayloadSize(const NrfDataPipe pipe , uint8_t size);
uint8_t `$INSTANCE_NAME`_GetPayloadSize(const NrfDataPipe pipe);
// FIFO_STATUS Reg
void `$INSTANCE_NAME`_PTX_ReuseLastTransmittedPayload(void);
// DYNPD Reg
void `$INSTANCE_NAME`_EnableDynamicPayload(const NrfDataPipe pipe);
void `$INSTANCE_NAME`_DisableDynamicPayload(const NrfDataPipe pipe);
// FEATURE Reg
void `$INSTANCE_NAME`_EnableDynamicPayloadLength(void);
void `$INSTANCE_NAME`_EnablePayloadWithACK(void);
void `$INSTANCE_NAME`_EnablePayloadWithNoACKCmd(void);
void `$INSTANCE_NAME`_DisableDynamicPayloadLength(void);
void `$INSTANCE_NAME`_DisablePayloadWithACK(void);
void `$INSTANCE_NAME`_DisablePayloadWithNoACKCmd(void);

// Data transfer functions
// TODO:
// We need to somehow distinguish what functions belong to 
// receiver and transmitter radios.
// `$INSTANCE_NAME`_PRX_x for receiver radio
// `$INSTANCE_NAME`_PTX_x for transmitter radio
// Functions that can be used from both modes will not have that distinction.
// Function naming must be clear enough to make documentation almost redundant,
// anyways the functions will be documented on the component datasheet.

void `$INSTANCE_NAME`_StartListening(void);
void `$INSTANCE_NAME`_StopListening(void);
void `$INSTANCE_NAME`_TransmitPulse(void);
void `$INSTANCE_NAME`_Listen(const bool listen);
uint8_t `$INSTANCE_NAME`_GetStatus(void);
uint8_t `$INSTANCE_NAME`_GetRetransmissionsCount(void);
uint8_t `$INSTANCE_NAME`_GetLostPacketsCount(void);
void `$INSTANCE_NAME`_FillTxFIFO(const uint8_t* data, size_t size);
void `$INSTANCE_NAME`_TxTransmit(const uint8_t* data, size_t size);
bool `$INSTANCE_NAME`_IsDataReady(void);
void `$INSTANCE_NAME`_GetRxPayload(uint8_t* payload, const size_t size);
void `$INSTANCE_NAME`_TxTransmitWaitNoACK(const uint8_t* data, size_t size);
void `$INSTANCE_NAME`_RxWritePayload(const NrfDataPipe pipe, uint8_t* data, size_t size);
uint8_t `$INSTANCE_NAME`_GetDataPipeWithPayload(void);
uint8_t `$INSTANCE_NAME`_PRX_ReceivedPowerDetector(void);
uint8_t `$INSTANCE_NAME`_IsTXFIFOFull(void);
uint8_t `$INSTANCE_NAME`_IsTXFIFOEmpty(void);
uint8_t `$INSTANCE_NAME`_IsRXFIFOFull(void);
uint8_t `$INSTANCE_NAME`_IsRXFIFOEmpty(void);

// IRQ Handle functions
void `$INSTANCE_NAME`_ClearIRQ(void);
void `$INSTANCE_NAME`_ClearIRQFlag(uint8_t flag);
uint8_t `$INSTANCE_NAME`_GetIRQFlag(void);

#endif /* `$INSTANCE_NAME`_FUNCS_H */

/* [] END OF FILE */
