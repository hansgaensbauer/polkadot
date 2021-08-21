//Driver for SX1276

#include "sx1276.h"		//SX1276 header file
#include <stdint.h>
#include "main.h"
#include "stm32f4xx.h"
#include <stdarg.h>
#include <stdio.h>

//SPI function mappings
uint8_t SX1276_Write_Register(uint8_t address, uint8_t data){
	HAL_GPIO_WritePin(SX1276_CS_GPIO_Port, SX1276_CS_Pin, GPIO_PIN_RESET);  //activate CS
	uint8_t masked_address_buffer = address | 0x80;							//set write bit
	HAL_SPI_Transmit(&hspi1, &masked_address_buffer, 1, 100);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){}
	HAL_SPI_Transmit(&hspi1, &data, 1, 100);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){}
	HAL_GPIO_WritePin(SX1276_CS_GPIO_Port, SX1276_CS_Pin, GPIO_PIN_SET);  //deactivate CS
	uint8_t newvalue = SX1276_Read_Register(address);
	if(newvalue == data) return SUCCESS;
	//DEBUG_PRINT("Write to register 0x%X Failed. Target = 0x%X, Result = 0x%X\n\r", address, data, newvalue);
	return FAIL;
}

uint8_t SX1276_Write_Burst(uint8_t start_address, uint8_t data[], uint8_t len){
	HAL_GPIO_WritePin(SX1276_CS_GPIO_Port, SX1276_CS_Pin, GPIO_PIN_RESET);  //activate CS
	uint8_t masked_address_buffer = start_address | 0x80;							//set write bit
	HAL_SPI_Transmit(&hspi1, &masked_address_buffer, 1, 100);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){}
	HAL_SPI_Transmit(&hspi1, data, len, 100);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){}
	HAL_GPIO_WritePin(SX1276_CS_GPIO_Port, SX1276_CS_Pin, GPIO_PIN_SET);  //deactivate CS
	return SUCCESS;
}

uint8_t SX1276_Read_Register(uint8_t address){
	uint8_t rxbuf[1];
	HAL_GPIO_WritePin(SX1276_CS_GPIO_Port, SX1276_CS_Pin, GPIO_PIN_RESET);  //activate CS
	HAL_SPI_Transmit(&hspi1, &address, 1, 100);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){}
	HAL_SPI_Receive(&hspi1, rxbuf, 1, 100);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){}
	HAL_GPIO_WritePin(SX1276_CS_GPIO_Port, SX1276_CS_Pin, GPIO_PIN_SET);  //deactivate CS
	return rxbuf[0];
}

uint8_t SX1276_Read_Burst(uint8_t start_address, uint8_t data[], uint8_t len){
	HAL_GPIO_WritePin(SX1276_CS_GPIO_Port, SX1276_CS_Pin, GPIO_PIN_RESET);  //activate CS;
	HAL_SPI_Transmit(&hspi1, &start_address, 1, 100);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){}
	HAL_SPI_Receive(&hspi1, data, len, 100);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){}
	HAL_GPIO_WritePin(SX1276_CS_GPIO_Port, SX1276_CS_Pin, GPIO_PIN_SET);  //deactivate CS
	return SUCCESS;
}

//Initialization function. Changes only registers with incorrect defaults
uint8_t SX1276_Init(uint8_t syncword, uint16_t preamblelength){
	DEBUG_PRINT("\n\nInitializing SX1276!\n\r");
	uint8_t result;
	result = SX1276_Set_Mode(SLEEP); //start from sleep mode
	result &= SX1276_Write_Register(REG_OP_MODE, 0b10001000); //Turn on LoRa
	DEBUG_PRINT("Enabling LoRa Mode\n\r");
	result &= SX1276_Set_Mode(STDBY); //go to standby mode
	result &= SX1276_Write_Register(REG_PA_CONFIG, 0b11001111); //use PA_BOOST pin
	DEBUG_PRINT("Enabling PA Boost\n\r");
	result &= SX1276_Set_Sync_Word(syncword);
	result &= SX1276_Set_Premble_Length(preamblelength);
	return result;
}

//Set the mode register. Modes are defined in SX1276.h
uint8_t SX1276_Set_Mode(uint8_t mode){
	uint8_t current_mode_reg_value = SX1276_Read_Register(REG_OP_MODE);
	DEBUG_PRINT("Setting mode to %d\n\r", mode);
	return SX1276_Write_Register(REG_OP_MODE, (current_mode_reg_value & 0xF8) | mode);
}

//Set the sync word register
uint8_t SX1276_Set_Sync_Word(uint8_t syncword){
	SX1276_Set_Mode(STDBY); //go to standby mode
	DEBUG_PRINT("Setting sync word to 0x%X\n\r", syncword);
	return SX1276_Write_Register(REG_SYNC_WORD, syncword);
}

//Set the length of the preamble. Recommended len > 20 bytes
uint8_t SX1276_Set_Premble_Length(uint16_t len){
	SX1276_Set_Mode(STDBY); //go to standby mode
    if(len < 6) return(FAIL);
	DEBUG_PRINT("Setting preamble length to %d bytes\n\r", len);
	return (SX1276_Write_Register(REG_PREAMBLE_MSB, (uint8_t) (len >> 8)) |
			SX1276_Write_Register(REG_SYNC_WORD, (uint8_t) (len & 0xFF)));
}

//This function is not used at this time
uint8_t SX1276_Set_Current_Limit(uint8_t limit){
	if(!(((limit >= 45) && (limit <= 240)) || (limit == 0))) {
		return(FAIL);
	}
	//SX1276_Set_Mode(STDBY); //go to standby mode
	return SUCCESS;
}

//Waits for transmit to finish before returning
uint8_t SX1276_Transmit_Blocking(uint8_t data[], uint8_t len){
	rx_flag = 0;
	uint8_t result;
	result = SX1276_Set_Mode(STDBY); //go to standby mode
	DEBUG_PRINT("Transmitting %d bytes\n\r",len);
	uint8_t reg_cur = SX1276_Read_Register(REG_DIO_MAPPING_1);
	SX1276_Write_Register(REG_DIO_MAPPING_1, (reg_cur & 0x3F) | 0x40); //set DIO mapping for DIO0 = TXDONE
	SX1276_Write_Register(REG_IRQ_FLAGS, 0xFF); 	//clear interrupt flags, this will "fail" so it's not included in result
	result &= SX1276_Write_Register(REG_PAYLOAD_LENGTH, len); //set packet length
	result &= SX1276_Write_Register(REG_FIFO_TX_BASE_ADDR, SX127X_FIFO_TX_BASE_ADDR_MAX); //set FIFO pointers
	result &= SX1276_Write_Register(REG_FIFO_ADDR_PTR, SX127X_FIFO_TX_BASE_ADDR_MAX);
	result &= SX1276_Write_Burst(REG_FIFO,data, len);	//write packet to FIFO
	//set RF switch --//TODO (maybe SX1276 does this on its own?)
	SX1276_Set_Mode(TX); //start transmitting, this will "fail" so it's not included in result
	//start timer
	HAL_Delay(100);
	//while(!xdone_flag);	//wait for the transmit to finish			//TODO - add timeout
	xdone_flag = 0;
	if(result == FAIL) DEBUG_PRINT("Transmit Failed\n\r");
	return result;
}

//does not wait for transmit to finish before returning
uint8_t SX1276_Start_Transmit(uint8_t data[], uint8_t len){
	rx_flag = 0;
	uint8_t result;
	result = SX1276_Set_Mode(STDBY); //go to standby mode
	DEBUG_PRINT("Transmitting %d bytes\n\r",len);
	uint8_t reg_cur = SX1276_Read_Register(REG_DIO_MAPPING_1);
	SX1276_Write_Register(REG_DIO_MAPPING_1, (reg_cur & 0x3F) | 0x40); //set DIO mapping for DIO0 = TXDONE
	SX1276_Write_Register(REG_IRQ_FLAGS, 0xFF); 	//clear interrupt flags, this will "fail" so it's not included in result
	result &= SX1276_Write_Register(REG_PAYLOAD_LENGTH, len); //set packet length
	result &= SX1276_Write_Register(REG_FIFO_TX_BASE_ADDR, SX127X_FIFO_TX_BASE_ADDR_MAX); //set FIFO pointers
	result &= SX1276_Write_Register(REG_FIFO_ADDR_PTR, SX127X_FIFO_TX_BASE_ADDR_MAX);
	result &= SX1276_Write_Burst(REG_FIFO,data, len);	//write packet to FIFO
	//set RF switch --//TODO (maybe SX1276 does this on its own?)
	SX1276_Set_Mode(TX); //start transmitting, this will "fail" so it's not included in result
	if(result == FAIL) DEBUG_PRINT("Transmit Failed\n\r");
	return result;
}

//Waits for a received packet before returning
uint8_t SX1276_Receive_Blocking(uint8_t* data, uint8_t * length){
	rx_flag = 0;
	DEBUG_PRINT("\n\rSTARTING RECEIVE\n\r");
	uint8_t result = SX1276_Set_Mode(STDBY);
	uint8_t reg_cur = SX1276_Read_Register(REG_DIO_MAPPING_1);
	result &= SX1276_Write_Register(REG_DIO_MAPPING_1, (reg_cur & 0x3F)); //set DIO mapping for for DIO0 = RXDONE
	SX1276_Write_Register(REG_IRQ_FLAGS, 0xFF); 	//clear interrupt flags
	result &= SX1276_Write_Register(REG_FIFO_RX_BASE_ADDR, SX127X_FIFO_RX_BASE_ADDR_MAX); //set FIFO pointers
	result &= SX1276_Write_Register(REG_FIFO_ADDR_PTR, SX127X_FIFO_RX_BASE_ADDR_MAX);
	result &= SX1276_Set_Mode(RXCONTINUOUS); //set mode to receive
	while(!xdone_flag);	//wait for the transmit to finish			//TODO - add timeout
	xdone_flag = 0;
	uint8_t len = SX1276_Read_Register(REG_RX_NB_BYTES); //get packet length
	DEBUG_PRINT("received %d bytes\n\r", len);
	*length = len;
	//TODO- CRC checking
	result &= SX1276_Read_Burst(REG_FIFO, data, len);
	result &= SX1276_Set_Mode(STDBY);
	return result;
}

//starts receive, interrupt is triggered on receive
uint8_t SX1276_Start_Receive(){
	rx_flag = 1;
	DEBUG_PRINT("\n\rSTARTING RECEIVE\n\r");
	uint8_t result = SX1276_Set_Mode(STDBY);
	uint8_t reg_cur = SX1276_Read_Register(REG_DIO_MAPPING_1);
	result &= SX1276_Write_Register(REG_DIO_MAPPING_1, (reg_cur & 0x3F)); //set DIO mapping for for DIO0 = RXDONE
	SX1276_Write_Register(REG_IRQ_FLAGS, 0xFF); 	//clear interrupt flags
	result &= SX1276_Write_Register(REG_FIFO_RX_BASE_ADDR, SX127X_FIFO_RX_BASE_ADDR_MAX); //set FIFO pointers
	result &= SX1276_Write_Register(REG_FIFO_ADDR_PTR, SX127X_FIFO_RX_BASE_ADDR_MAX);
	result &= SX1276_Set_Mode(RXCONTINUOUS); //set mode to receive
	return result;
}

//read RSSI register
uint8_t SX1276_Get_RSSI(){
	return SX1276_Read_Register(REG_PKT_RSSI_VALUE);
}

//Turn on/off CRC. Modes are defined in sx1276.h
uint8_t SX1276_Set_CRC(uint8_t mode){
	uint8_t lastregvalue = SX1276_Read_Register(REG_MODEM_CONFIG_2);
	return SX1276_Write_Register(REG_MODEM_CONFIG_2, (lastregvalue & 0xFB) | mode);
}

//This function is not implemented
uint8_t SX1276_Configure_IT(){
	return SUCCESS;
}

//Debug functions  -- disable this here if you don't want uart coming out of the GPIO
void DEBUG_PRINT(char * format, ...){
	char buffer[256];
	va_list args;
	va_start (args, format);
	uint8_t stringsize = vsnprintf(buffer, 256, format, args);
	HAL_UART_Transmit(&huart1,(uint8_t *)buffer,stringsize, 100);
	va_end (args);
}

//Prints an array with proper formatting
void printarr(uint8_t toprint[], uint8_t len){
	DEBUG_PRINT("[");
	for(uint8_t i = 0; i<len-1; i++){
		DEBUG_PRINT("%d, ",toprint[i]);
	}
	DEBUG_PRINT("%d]", toprint[len-1]);
}
