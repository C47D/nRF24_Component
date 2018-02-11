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
* @file `$INSTANCE_NAME`_REGS.h
*
* @brief This file contains all the nRF24 radio registers, commands and
* usefull enum types.
* 
*/

#ifndef `$INSTANCE_NAME`_REGS_H
#define `$INSTANCE_NAME`_REGS_H

// nRF24L01P Registers
typedef enum {
    NRF_CONFIG_REG      = 0x00,
    NRF_EN_AA_REG       = 0x01,
    NRF_EN_RXADDR_REG   = 0x02,
    NRF_SETUP_AW_REG    = 0x03,
    NRF_SETUP_RETR_REG  = 0x04,
    NRF_RF_CH_REG       = 0x05,
    NRF_RF_SETUP_REG    = 0x06,
    NRF_STATUS_REG      = 0x07,
    NRF_OBSERVE_TX_REG  = 0x08,
    NRF_RPD_REG         = 0x09,
    NRF_RX_ADDR_P0_REG  = 0x0A,
    NRF_RX_ADDR_P1_REG  = 0x0B,
    NRF_RX_ADDR_P2_REG  = 0x0C,
    NRF_RX_ADDR_P3_REG  = 0x0D,
    NRF_RX_ADDR_P4_REG  = 0x0E,
    NRF_RX_ADDR_P5_REG  = 0x0F,
    NRF_TX_ADDR_REG     = 0x10,
    NRF_RX_PW_P0_REG    = 0x11,
    NRF_RX_PW_P1_REG    = 0x12,
    NRF_RX_PW_P2_REG    = 0x13,
    NRF_RX_PW_P3_REG    = 0x14,
    NRF_RX_PW_P4_REG    = 0x15,
    NRF_RX_PW_P5_REG    = 0x16,
    NRF_FIFO_STATUS_REG = 0x17,
    NRF_DYNPD_REG       = 0x1C,
    NRF_FEATURE_REG     = 0x1D,
} nrf_register;

// Command Name Mnemonics (Instructions)
typedef enum{
    NRF_R_REGISTER_CMD          = 0x00, // Read command
    NRF_W_REGISTER_CMD          = 0x20, // Write command
    NRF_R_RX_PAYLOAD_CMD        = 0x61, // Read Rx payload
    NRF_W_TX_PAYLOAD_CMD        = 0xA0, // Write Tx payload
    NRF_FLUSH_TX_CMD            = 0xE1, // Flush Tx FIFO
    NRF_FLUSH_RX_CMD            = 0xE2, // Flush Rx FIFO
    NRF_REUSE_TX_PL_CMD         = 0xE3, // Reuse the last transmitted payload
    NRF_R_RX_PL_WID_CMD         = 0x60, // Read Rx payload width for the top R_RX_PAYLOAD
    NRF_W_ACK_PAYLOAD_CMD       = 0xA8, // Write payload to be transmitted together with ACK packet
    NRF_W_TX_PAYLOAD_NOACK_CMD  = 0xB0, // Disable AUTOACK on this specific packet
    NRF_NOP_CMD                 = 0xFF, // No operation, read STATUS register
} nrf_cmd;

// Data pipes
typedef enum{
    NRF_PIPE0   = 0x00,
    NRF_PIPE1   = 0x01,
    NRF_PIPE2   = 0x02,
    NRF_PIPE3   = 0x03,
    NRF_PIPE4   = 0x04,
    NRF_PIPE5   = 0x05,
} nrf_pipe;

// Rx data pipes addresses
typedef enum {
    NRF_PIPE0_ADDR = NRF_RX_ADDR_P0_REG,
    NRF_PIPE1_ADDR = NRF_RX_ADDR_P1_REG,
    NRF_PIPE2_ADDR = NRF_RX_ADDR_P2_REG,
    NRF_PIPE3_ADDR = NRF_RX_ADDR_P3_REG,
    NRF_PIPE4_ADDR = NRF_RX_ADDR_P4_REG,
    NRF_PIPE5_ADDR = NRF_RX_ADDR_P5_REG,
} nrf_rx_pipe_address;

typedef enum {
    NRF_PIPE0_PAYLOAD_SIZE = NRF_RX_PW_P0_REG,
    NRF_PIPE1_PAYLOAD_SIZE = NRF_RX_PW_P1_REG,
    NRF_PIPE2_PAYLOAD_SIZE = NRF_RX_PW_P2_REG,
    NRF_PIPE3_PAYLOAD_SIZE = NRF_RX_PW_P3_REG,
    NRF_PIPE4_PAYLOAD_SIZE = NRF_RX_PW_P4_REG,
    NRF_PIPE5_PAYLOAD_SIZE = NRF_RX_PW_P5_REG,
} nrf_pipe_payload_size;

// nRF24 Modes
typedef enum{
    NRF_MODE_TX     = 0,
    NRF_MODE_RX     = 1,
} nrf_mode;

typedef enum{
    NRF_NONE_IRQ    = 0, // 0x00
    NRF_MAX_RT_IRQ  = 4, // 0x10
    NRF_TX_DS_IRQ   = 5, // 0x20
    NRF_RX_DR_IRQ   = 6, // 0x40
} nrf_irq;

// CONFIG: Configuration Register
enum {
    NRF_CONFIG_PRIM_RX      = 0,
    NRF_CONFIG_PWR_UP       = 1,
    NRF_CONFIG_CRCO         = 2,
    NRF_CONFIG_EN_CRC       = 3,
    NRF_CONFIG_MASK_MAX_RT  = 4,
    NRF_CONFIG_MASK_TX_DS   = 5,
    NRF_CONFIG_MASK_RX_DR   = 6,
};
    
// EN_AA: Enable Enhanced ShockBurst
enum {
    NRF_EN_AA_ENAA_P0   = 0,
    NRF_EN_AA_ENAA_P1   = 1,
    NRF_EN_AA_ENAA_P2   = 2,
    NRF_EN_AA_ENAA_P3   = 3,
    NRF_EN_AA_ENAA_P4   = 4,
    NRF_EN_AA_ENAA_P5   = 5,
};
    
// EN_RXADDR: Enabled RX Addresses
enum {
    NRF_EN_RXADDR_ERX_P0    = 0,
    NRF_EN_RXADDR_ERX_P1    = 1,
    NRF_EN_RXADDR_ERX_P2    = 2,
    NRF_EN_RXADDR_ERX_P3    = 3,
    NRF_EN_RXADDR_ERX_P4    = 4,
    NRF_EN_RXADDR_ERX_P5    = 5,
};

// SETUP_AW: Setup of Address Widths(common for all data pipes)
typedef enum {
    NRF_SETUP_AW_3BYTES = 1,
    NRF_SETUP_AW_4BYTES = 2,
    NRF_SETUP_AW_5BYTES = 3,
} nrf_setup_address_width;

// helper to use set and get pipe addresses
typedef enum {
    NRF_PIPE_ADDR_WIDTH_3BYTES = 3,
    NRF_PIPE_ADDR_WIDTH_4BYTES = 4,
    NRF_PIPE_ADDR_WIDTH_5BYTES = 5,
} nrf_pipe_address_width;

// SETUP_RETR: Setup of Automatic Retransmission
enum {
    NRF_SETUP_RETR_ARC  = 0,
    NRF_SETUP_RETR_ARD  = 4,
};

enum {
    NRF_SETUP_RETR_ARC_0    = 0x00, // 0 retry count, retry disabled
    NRF_SETUP_RETR_ARC_1    = 0x01, // 1 retry count, retry disabled
    NRF_SETUP_RETR_ARC_2    = 0x02, // 2 retry count, retry disabled
    NRF_SETUP_RETR_ARC_3    = 0x03, // 3 retry count, retry disabled
    NRF_SETUP_RETR_ARC_4    = 0x04, // 4 retry count, retry disabled
    NRF_SETUP_RETR_ARC_5    = 0x05, // 5 retry count, retry disabled
    NRF_SETUP_RETR_ARC_6    = 0x06, // 6 retry count, retry disabled
    NRF_SETUP_RETR_ARC_7    = 0x07, // 7 retry count, retry disabled
    NRF_SETUP_RETR_ARC_8    = 0x08, // 8 retry count, retry disabled
    NRF_SETUP_RETR_ARC_9    = 0x09, // 9 retry count, retry disabled
    NRF_SETUP_RETR_ARC_10   = 0x0A, // 10 retry count, retry disabled
    NRF_SETUP_RETR_ARC_11   = 0x0B, // 11 retry count, retry disabled
    NRF_SETUP_RETR_ARC_12   = 0x0C, // 12 retry count, retry disabled
    NRF_SETUP_RETR_ARC_13   = 0x0D, // 13 retry count, retry disabled
    NRF_SETUP_RETR_ARC_14   = 0x0E, // 14 retry count, retry disabled
    NRF_SETUP_RETR_ARC_15   = 0x0F, // 15 retry count, retry disabled
};

enum {
    NRF_SETUP_RETR_ARD_250_US   = 0x00, // 250 us retry delay
    NRF_SETUP_RETR_ARD_500_US   = 0x10, // 500 us retry delay
    NRF_SETUP_RETR_ARD_750_US   = 0x20, // 750 us retry delay
    NRF_SETUP_RETR_ARD_1000_US  = 0x30, // 1000 us retry delay
    NRF_SETUP_RETR_ARD_1250_US  = 0x40, // 1250 us retry delay
    NRF_SETUP_RETR_ARD_1500_US  = 0x50, // 1500 us retry delay
    NRF_SETUP_RETR_ARD_1750_US  = 0x60, // 1750 us retry delay
    NRF_SETUP_RETR_ARD_2000_US  = 0x70, // 2000 us retry delay
    NRF_SETUP_RETR_ARD_2250_US  = 0x80, // 2250 us retry delay
    NRF_SETUP_RETR_ARD_2500_US  = 0x90, // 2500 us retry delay
    NRF_SETUP_RETR_ARD_2750_US  = 0xA0, // 2750 us retry delay
    NRF_SETUP_RETR_ARD_3000_US  = 0xB0, // 3000 us retry delay
    NRF_SETUP_RETR_ARD_3250_US  = 0xC0, // 3250 us retry delay
    NRF_SETUP_RETR_ARD_3500_US  = 0xD0, // 3500 us retry delay
    NRF_SETUP_RETR_ARD_3750_US  = 0xE0, // 3750 us retry delay
    NRF_SETUP_RETR_ARD_4000_US  = 0xF0, // 4400 us retry delay
};

// RF_CH: RF Channel the radio will work on, there are 125 available channels.

// RF_SETUP: RF Setup Register
enum {
    NRF_RF_SETUP_RF_PWR     = 1,
    NRF_RF_SETUP_RF_DR      = 3,
    NRF_RF_SETUP_RF_DR_HIGH = 3,
    NRF_RF_SETUP_PLL_LOCK   = 4,
    NRF_RF_SETUP_RF_DR_LOW  = 5,
    NRF_RF_SETUP_CONT_WAVE  = 7,
};

enum {
    NRF_RF_SETUP_RF_DR_1000 = 0,
    NRF_RF_SETUP_RF_DR_2000 = 8,
    NRF_RF_SETUP_RF_DR_250  = 0x20,
};

enum {
    NRF_RF_SETUP_RF_PWR_18  = 0,
    NRF_RF_SETUP_RF_PWR_12  = 2,
    NRF_RF_SETUP_RF_PWR_6   = 4,
    NRF_RF_SETUP_RF_PWR_0   = 6,
};

// STATUS: Status Register
enum {
    NRF_STATUS_TX_FULL  = 0,
    NRF_STATUS_RX_P_NO  = 1,
    NRF_STATUS_MAX_RT   = 4,
    NRF_STATUS_TX_DS    = 5,
    NRF_STATUS_RX_DR    = 6,
};

enum {
    NRF_STATUS_MAX_RT_MASK  = 0x10,
    NRF_STATUS_TX_DS_MASK   = 0x20,
    NRF_STATUS_RX_DR_MASK   = 0x40,
};

enum {
    NRF_STATUS_TXFIFO_NOTFULL   = 0,
    NRF_STATUS_TXFIFO_FULL      = 1,
};

enum {
    NRF_STATUS_RX_P_NO_0 = 0,
    NRF_STATUS_RX_P_NO_1 = 1,
    NRF_STATUS_RX_P_NO_2 = 2,
    NRF_STATUS_RX_P_NO_3 = 3,
    NRF_STATUS_RX_P_NO_4 = 4,
    NRF_STATUS_RX_P_NO_5 = 5,
    NRF_STATUS_RX_P_NO_UNUSED   = 0x0C,
    NRF_STATUS_RX_P_NO_EMPTY    = 0x0E,
};

// OBSERVE TX: Transmit observe register
enum {
    NRF_OBSERVE_TX_ARC_CNT  = 0,
    NRF_OBSERVE_TX_PLOS_CNT = 4,
};

enum {
    NRF_OBSERVE_TX_ARC_CNT_MASK     = 0x0F,
    NRF_OBSERVE_TX_PLOS_CNT_MASK    = 0xF0,
};

// RPD: Received Power Detector
enum {
    NRF_RPD_RPD = 0,
};

// SETUP_AW: Setup of Address Widths(common for all data pipes)
enum {
    NRF_RX_PW_MASK  = 0x3F,
};

// FIFO_STATUS: FIFO Status Register
enum {
    NRF_FIFO_STATUS_RX_EMPTY    = 0,
    NRF_FIFO_STATUS_RX_FULL     = 1,
    NRF_FIFO_STATUS_TX_EMPTY    = 4,
    NRF_FIFO_STATUS_TX_FULL     = 5,
    NRF_FIFO_STATUS_TX_REUSE    = 6,
};

// DYNPD: Enable dynamic payload length
enum {
    NRF_DYNPD_DPL_P0    = 0,
    NRF_DYNPD_DPL_P1    = 1,
    NRF_DYNPD_DPL_P2    = 2,
    NRF_DYNPD_DPL_P3    = 3,
    NRF_DYNPD_DPL_P4    = 4,
    NRF_DYNPD_DPL_P5    = 5,
};

// FEATURE: Feature Register
enum {
    NRF_FEATURE_EN_DYN_ACK  = 0,
    NRF_FEATURE_EN_ACK_PAY  = 1,
    NRF_FEATURE_EN_DPL      = 2,
};

// return status, based in cyret
enum {
    RET_SUCCESS         = 0x00, // Successful
    RET_BAD_PARAM       = 0x01, // One or more invalid parameters
    RET_INVALID_OBJECT  = 0x02, // Invalid object specified
    RET_MEMORY          = 0x03, // Memory related failure
    RET_LOCKED          = 0x04, // Resource lock failure
    RET_EMPTY           = 0x05, // No more objects available
    RET_BAD_DATA        = 0x06, // Bad data received (CRC or other error check)
    RET_STARTED         = 0x07, // Operation started, but not necessarily completed yet
    RET_FINISHED        = 0x08, // Operation completed
    RET_CANCELED        = 0x09, // Operation canceled
    RET_TIMEOUT         = 0x0A, // Operation timed out
    RET_INVALID_STATE   = 0x0B, // Operation not setup or is in an improper state
    RET_NULL_PTR        = 0x0C, // NULL pointer was used as an parameter.
};

// MISC enums
enum {
    NRF_STATUS_PIPES_SHIFT  = 1,
    NRF_CE_PULSE_WIDTH_US   = 15,
    NRF_MAX_PAYLOAD_SIZE    = 32,
    NRF_POWER_UP_DELAY      = 100,
    NRF_MAX_RF_CHANNEL      = 125,
    NRF_ALL_IRQ_MASK        = 0x70,
    NRF_STATUS_PIPES_MASK   = 0x0E,
};

#endif /* `$INSTANCE_NAME`_REGS_H */

/* [] END OF FILE */
