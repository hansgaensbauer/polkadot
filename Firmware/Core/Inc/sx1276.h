/*
 * Header file for SX1276. Includes register definitions, and #defines
 */
#include <stdint.h>
#include "stm32f4xx.h"

#ifndef INC_SX1276_H_
#define INC_SX1276_H_

//status return values
#define SUCCESS	0xFF
#define FAIL 0x00

//register definitions
#define REG_FIFO 0x00
#define REG_OP_MODE 0x01
#define REG_FFR_MSB 0x06
#define REG_FR_MID 0x07
#define REG_FFR_LSB 0x08
#define REG_PA_CONFIG 0x09
#define REG_PA_RAMP 0x0A
#define REG_OCP 0x0B
#define REG_LNA 0x0C
#define REG_FIFO_ADDR_PTR 0x0D
#define REG_FIFO_TX_BASE_ADDR 0x0E
#define REG_FIFO_RX_BASE_ADDR 0x0F
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS_MASK 0x11
#define REG_IRQ_FLAGS 0x12
#define REG_RX_NB_BYTES 0x13
#define REG_RX_HEADER_CNT_VALUE_MSB 0x14
#define REG_RX_HEADER_CNT_VALUE_LSB 0x15
#define REG_RX_PACKET_CNT_VALUE_MSB 0x16
#define REG_RX_PACKET_CNT_VALUE_LSB 0x17
#define REG_MODEM_STAT 0x18
#define REG_PKT_SNR_VALUE 0x19
#define REG_PKT_RSSI_VALUE 0x1A
#define REG_RSSI_VALUE 0x1B
#define REG_HOP_CHANNEL 0x1C
#define REG_MODEM_CONFIG_1 0x1D
#define REG_MODEM_CONFIG_2 0x1E
#define REG_SYMB_TIMEOUT_LSB 0x1F
#define REG_PREAMBLE_MSB 0x20
#define REG_PREAMBLE_LSB 0x21
#define REG_PAYLOAD_LENGTH 0x22
#define REG_MAX_PAYLOAD_LENGTH 0x23
#define REG_HOP_PERIOD 0x24
#define REG_FIFO_RX_BYTE_ADDR 0x25
#define REG_MODEM_CONFIG_3 0x26
#define REG_FEIL_MSB 0x28
#define REG_FEIL_MID 0x29
#define REG_FEIL_LSB 0x2A
#define REG_RSSI_WIDE_BAND 0x2C
#define REG_DETECT_OPTIMIZE 0x31
#define REG_INVERT_IQ 0x33
#define REG_DETECTION_THRESHOLD 0x37
#define REG_SYNC_WORD 0x39
#define REG_DIO_MAPPING_1 0x40
#define REG_DIO_MAPPING_2 0x41
#define REG_VERSION 0x42
#define REG_TCXO 0x4B
#define REG_PA_DAC 0x4D
#define REG_FORMER_TEMP 0x5B
#define REG_AGC_REF 0x61
#define REG_ACG_THRES_1 0x62
#define REG_ACG_THRES_2 0x63
#define REG_ACG_THRES_3 0x64
#define REG_PLL 0x70

//SX1276 modes
#define SLEEP 0
#define STDBY 1
#define FSTX 2
#define TX 3
#define FSRX 4
#define RXCONTINUOUS 5
#define RXSINGLE 6
#define CAD 7

//constants
#define SX127X_FIFO_TX_BASE_ADDR_MAX 0x00
#define SX127X_FIFO_RX_BASE_ADDR_MAX 0x00
#define SYNC_WORD 0x12

//CRC modes
#define CRC_ENABLE 0x04
#define CRC_DISABLE 0x00

//"global" variables
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;
extern volatile unsigned char xdone_flag;
extern volatile unsigned char rx_flag;

//function prototypes
uint8_t SX1276_Write_Register(uint8_t address, uint8_t data);
uint8_t SX1276_Write_Burst(uint8_t start_address, uint8_t * data, uint8_t len);
uint8_t SX1276_Read_Register(uint8_t address);
uint8_t SX1276_Read_Burst(uint8_t start_address, uint8_t data[], uint8_t len);
uint8_t SX1276_Init(uint8_t syncword, uint16_t preamblelength);
uint8_t SX1276_Set_Mode(uint8_t mode);
uint8_t SX1276_Transmit_Blocking(uint8_t data[], uint8_t len);
uint8_t SX1276_Receive_Blocking(uint8_t* data, uint8_t * length);
uint8_t SX1276_Start_Transmit(uint8_t data[], uint8_t len);
uint8_t SX1276_Start_Receive();
uint8_t SX1276_Get_RSSI();
uint8_t SX1276_Configure_IT();
uint8_t SX1276_Set_Sync_Word(uint8_t syncword);
uint8_t SX1276_Set_Premble_Length(uint16_t len);
uint8_t SX1276_Set_Current_Limit(uint8_t limit);
void printarr(uint8_t toprint[], uint8_t len);

void DEBUG_PRINT(char * format, ...);

#endif /* INC_SX1276_H_ */
