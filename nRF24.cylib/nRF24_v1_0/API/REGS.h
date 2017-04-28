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

#ifndef `$INSTANCE_NAME`_REGS_H
#define `$INSTANCE_NAME`_REGS_H

// nRF24L01P Registers
typedef enum {
    NRF_CONFIG_REG = 0x00,
    NRF_EN_AA_REG = 0x01,
    NRF_EN_RXADDR_REG = 0x02,
    NRF_SETUP_AW_REG = 0x03,
    NRF_SETUP_RETR_REG = 0x04,
    NRF_RF_CH_REG = 0x05,
    NRF_RF_SETUP_REG = 0x06,
    NRF_STATUS_REG = 0x07,
    NRF_OBSERVE_TX_REG = 0x08,
    NRF_RPD_REG = 0x09,
    NRF_RX_ADDR_P0_REG = 0x0A,
    NRF_RX_ADDR_P1_REG = 0x0B,
    NRF_RX_ADDR_P2_REG = 0x0C,
    NRF_RX_ADDR_P3_REG = 0x0D,
    NRF_RX_ADDR_P4_REG = 0x0E,
    NRF_RX_ADDR_P5_REG = 0x0F,
    NRF_TX_ADDR_REG = 0x10,
    NRF_RX_PW_P0_REG = 0x11,
    NRF_RX_PW_P1_REG = 0x12,
    NRF_RX_PW_P2_REG = 0x13,
    NRF_RX_PW_P3_REG = 0x14,
    NRF_RX_PW_P4_REG = 0x15,
    NRF_RX_PW_P5_REG = 0x16,
    NRF_FIFO_STATUS_REG = 0x17,
    NRF_DYNPD_REG = 0x1C,
    NRF_FEATURE_REG = 0x1D
}NRF_REGISTER_t;

// Command Name Mnemonics (Instructions)
typedef enum{
    NRF_R_REGISTER_CMD = 0x00, // Read command
    NRF_W_REGISTER_CMD = 0x20, // Write command
    NRF_R_RX_PAYLOAD_CMD = 0x61, // Read Rx payload
    NRF_W_TX_PAYLOAD_CMD = 0xA0, // Write Tx payload
    NRF_FLUSH_TX_CMD = 0xE1, // Flush Tx FIFO
    NRF_FLUSH_RX_CMD = 0xE2, // Flush Rx FIFO
    NRF_REUSE_TX_PL_CMD = 0xE3, // Reuse the last transmitted payload
    NRF_R_RX_PL_WID_CMD = 0x60, // Read Rx payload width for the top R_RX_PAYLOAD
    NRF_W_ACK_PAYLOAD_CMD = 0xA8, // Write payload to be transmitted together with ACK packet
    NRF_W_TX_PAYLOAD_NOACK_CMD = 0xB0, // Disable AUTOACK on this specific packet
    NRF_NOP_CMD = 0xFF // No operation, read STATUS register
}NRF_CMD_t;

// Data pipes
typedef enum{
    NRF_DATA_PIPE0 = 0x00,
    NRF_DATA_PIPE1 = 0x01,
    NRF_DATA_PIPE2 = 0x02,
    NRF_DATA_PIPE3 = 0x03,
    NRF_DATA_PIPE4 = 0x04,
    NRF_DATA_PIPE5 = 0x05
}NRF_DATA_PIPE_t;

// nRF24 Modes
typedef enum{
    NRF_MODE_TX = 0,
    NRF_MODE_RX = 1
}NRF_MODE_t;

// Register Bitfields, is it a good approach?

// CONFIG: Configuration Register
#define NRF_CONFIG_PRIM_RX      0
#define NRF_CONFIG_PWR_UP       1
#define NRF_CONFIG_CRCO         2
#define NRF_CONFIG_EN_CRC       3
#define NRF_CONFIG_MASK_MAX_RT  4
#define NRF_CONFIG_MASK_TX_DS   5
#define NRF_CONFIG_MASK_RX_DR   6

// EN_AA: Enable Enhanced ShockBurst
#define NRF_EN_AA_ENAA_P0   0
#define NRF_EN_AA_ENAA_P1   1
#define NRF_EN_AA_ENAA_P2   2
#define NRF_EN_AA_ENAA_P3   3
#define NRF_EN_AA_ENAA_P4   4
#define NRF_EN_AA_ENAA_P5   5
#define NRF_EN_AA_ENAA_ALL  0x3F
#define NRF_EN_AA_ENAA_NONE 0

// EN_RXADDR: Enabled RX Addresses
#define NRF_EN_RXADDR_ERX_P0    0
#define NRF_EN_RXADDR_ERX_P1    1
#define NRF_EN_RXADDR_ERX_P2    2
#define NRF_EN_RXADDR_ERX_P3    3
#define NRF_EN_RXADDR_ERX_P4    4
#define NRF_EN_RXADDR_ERX_P5    5
#define NRF_EN_RXADDR_ERX_ALL   0x3F
#define NRF_EN_RXADDR_ERX_NONE  0

// SETUP_AW: Setup of Address Widths(common for all data pipes)
#define NRF_SETUP_AW_3BYTES 3
#define NRF_SETUP_AW_4BYTES 4
#define NRF_SETUP_AW_5BYTES 1

// SETUP_RETR: Setup of Automatic Retransmission
#define NRF_SETUP_RETR_ARD_SHIFT    4

// i think i can remove this with the custom type on the symbol, need to improve
#define NRF_SETUP_RETR_ARD_4000   0xF0 /* 4400 us retry delay */
#define NRF_SETUP_RETR_ARD_3750   0xE0 /* 3750 us retry delay */
#define NRF_SETUP_RETR_ARD_3500   0xD0 /* 3500 us retry delay */
#define NRF_SETUP_RETR_ARD_3250   0xC0 /* 3250 us retry delay */
#define NRF_SETUP_RETR_ARD_3000   0xB0 /* 3000 us retry delay */
#define NRF_SETUP_RETR_ARD_2750   0xA0 /* 2750 us retry delay */
#define NRF_SETUP_RETR_ARD_2500   0x90 /* 2500 us retry delay */
#define NRF_SETUP_RETR_ARD_2250   0x80 /* 2250 us retry delay */
#define NRF_SETUP_RETR_ARD_2000   0x70 /* 2000 us retry delay */
#define NRF_SETUP_RETR_ARD_1750   0x60 /* 1750 us retry delay */
#define NRF_SETUP_RETR_ARD_1500   0x50 /* 1500 us retry delay */
#define NRF_SETUP_RETR_ARD_1250   0x40 /* 1250 us retry delay */
#define NRF_SETUP_RETR_ARD_1000   0x30 /* 1000 us retry delay */
#define NRF_SETUP_RETR_ARD_750    0x20 /* 750 us retry delay */
#define NRF_SETUP_RETR_ARD_500    0x10 /* 500 us retry delay */
#define NRF_SETUP_RETR_ARD_250    0x00 /* 250 us retry delay */
#define NRF_SETUP_RETR_ARC_15     0x0F /* 15 retry count */
#define NRF_SETUP_RETR_ARC_14     0x0E /* 14 retry count */
#define NRF_SETUP_RETR_ARC_13     0x0D /* 13 retry count */
#define NRF_SETUP_RETR_ARC_12     0x0C /* 12 retry count */
#define NRF_SETUP_RETR_ARC_11     0x0B /* 11 retry count */
#define NRF_SETUP_RETR_ARC_10     0x0A /* 10 retry count */
#define NRF_SETUP_RETR_ARC_9      0x09 /* 9 retry count */
#define NRF_SETUP_RETR_ARC_8      0x08 /* 8 retry count */
#define NRF_SETUP_RETR_ARC_7      0x07 /* 7 retry count */
#define NRF_SETUP_RETR_ARC_6      0x06 /* 6 retry count */
#define NRF_SETUP_RETR_ARC_5      0x05 /* 5 retry count */
#define NRF_SETUP_RETR_ARC_4      0x04 /* 4 retry count */
#define NRF_SETUP_RETR_ARC_3      0x03 /* 3 retry count */
#define NRF_SETUP_RETR_ARC_2      0x02 /* 2 retry count */
#define NRF_SETUP_RETR_ARC_1      0x01 /* 1 retry count */
#define NRF_SETUP_RETR_ARC_0      0x00 /* 0 retry count, retry disabled */

// RF_CH: RF Channel the radio will work on, there are 125 available channels.

// RF_SETUP: RF Setup Register
#define NRF_RF_SETUP_RF_DR      3
#define NRF_RF_SETUP_RF_DR_HIGH 3
#define NRF_RF_SETUP_PLL_LOCK   4
#define NRF_RF_SETUP_RF_DR_LOW  5
#define NRF_RF_SETUP_CONT_WAVE  7
#define NRF_RF_SETUP_RF_DR_250  0x20
#define NRF_RF_SETUP_RF_DR_1000 0
#define NRF_RF_SETUP_RF_DR_2000 8
#define NRF_RF_SETUP_RF_PWR     1
#define NRF_RF_SETUP_RF_PWR_0   6
#define NRF_RF_SETUP_RF_PWR_6   4
#define NRF_RF_SETUP_RF_PWR_12  2
#define NRF_RF_SETUP_RF_PWR_18  0

// STATUS: Status Register
#define NRF_STATUS_RX_DR_MASK   0x40
#define NRF_STATUS_TX_DS_MASK   0x20
#define NRF_STATUS_MAX_RT_MASK  0x10
#define NRF_STATUS_DATA_IS_RDY  0x40

#define NRF_STATUS_TX_FULL          0
#define NRF_STATUS_TX_FIFO_FULL     1
#define NRF_STATUS_TX_FIFO_AV_LOC   0
#define NRF_STATUS_MAX_RT   4
#define NRF_STATUS_TX_DS    5
#define NRF_STATUS_RX_DR    6
#define NRF_STATUS_RX_P_NO_RX_FIFO_NOT_EMPTY  0x0E
#define NRF_STATUS_RX_P_NO_UNUSED  0x0C
#define NRF_STATUS_RX_P_NO_5  0x0A
#define NRF_STATUS_RX_P_NO_4  8
#define NRF_STATUS_RX_P_NO_3  6
#define NRF_STATUS_RX_P_NO_2  4
#define NRF_STATUS_RX_P_NO_1  2
#define NRF_STATUS_RX_P_NO_0  0

// OBSERVE TX: Transmit observe register
#define NRF_OBSERVE_TX_PLOS_CNT_MASK    0xF0
#define NRF_OBSERVE_TX_ARC_CNT_MASK     0x0F

// RPD: Received Power Detector
#define NRF_RPD_RPD 0

// SETUP_AW: Setup of Address Widths(common for all data pipes)
#define NRF_RX_PW_MASK  0x3F

// FIFO_STATUS: FIFO Status Register
#define NRF_FIFO_STATUS_RX_EMPTY    0
#define NRF_FIFO_STATUS_RX_FULL     1
#define NRF_FIFO_STATUS_TX_EMPTY    4
#define NRF_FIFO_STATUS_TX_FULL     5
#define NRF_FIFO_STATUS_TX_REUSE    6

// DYNPD: Enable dynamic payload length
#define NRF_DYNPD_DPL_P0 0
#define NRF_DYNPD_DPL_P1 1
#define NRF_DYNPD_DPL_P2 2
#define NRF_DYNPD_DPL_P3 3
#define NRF_DYNPD_DPL_P4 4
#define NRF_DYNPD_DPL_P5 5

// FEATURE: Feature Register
#define NRF_FEATURE_EN_DYN_ACK 0
#define NRF_FEATURE_EN_ACK_PAY 1
#define NRF_FEATURE_EN_DPL 2

#endif /* `$INSTANCE_NAME`_REGS_H */

/* [] END OF FILE */
