#include "main.h"

void MY1690_WriteCommand(uint8_t command){
	uint8_t data[5] = {0x7E, 0x03, command, 0x03^command, 0xEF};
	if(0x11<=command && command<=0x1E){
		HAL_UART_Transmit(&huart5, data, 5, 0xffff);
	}
}

void MY1690_Read(uint8_t command, uint8_t* pData){
	uint8_t data[5] = {0x7E, 0x03, command, 0x03^command, 0xEF};
	if(0x20<=command && command<=0x2F){
		HAL_UART_Transmit(&huart5, data, 5, 0xffff);
		HAL_UART_Receive(&huart5, pData, 9, 0xffff);
	}
}

void MY1690_SetCommand(uint8_t command, uint8_t num){
	uint8_t data[6] = {0x7E, 0x04, command, num, 0x04^command^num, 0xEF};
	if(0x20<=command && command<=0x2F){
		HAL_UART_Transmit(&huart5, data, 6, 0xffff);
	}
}

void MY1690_LongCommand(uint8_t command, uint8_t num1, uint8_t num2){
	uint8_t data[7] = {0x7E, 0x05, command, num1, num2, 0x05^command^num1^num2, 0xEF};
	if(0x41<=command && command<=0x44){
		HAL_UART_Transmit(&huart5, data, 7, 0xffff);
	}
}

