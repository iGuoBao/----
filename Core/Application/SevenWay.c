#include "SevenWay.h"
#include "imu901.h"
#define flag 0
uint8_t seven_datas[3];
uint8_t Byte[2] = {0x57, 0x01};
static int error[7] = {3, 2, 1, 0, -1, -2, -3};

int power(uint16_t x)
{
	int t = 0;
	for (int i = 0; i < 16; i++)
	{
		if (x & (1 << i))
			t = t + (1 << i);
	}
	return t;
}
int SevenWay_Read(void)
{
#if KT
	uint8_t data;
	uint16_t seven_data = 0x00;
	char k1[20] = "";
	HAL_UART_Transmit(&huart2, Byte, 2, 0xffff);
	HAL_UART_Receive_IT(&huart2, seven_datas, 20);
	if (seven_datas[0] != 0x75)
		return 0xff;
	seven_ff = seven_datas[1];
	check_data = seven_datas[2];
	seven_data = seven_datas[3];
	seven_data = seven_data << 8 | seven_datas[4];
	if (!seven_datas[2] & 0x20)
		return (power(seven_data));
	if ((seven_datas[2] & 0x20))
		return -1 * power(seven_data);
#else
	uint8_t data;
	int Receive_data = 0;
	HAL_UART_Transmit_IT(&huart2, Byte, 2);
	HAL_UART_Receive_IT(&huart2, seven_datas, 3);
	if (seven_datas[0] != 0x75)
		return 0xff;
	seven_ff = seven_datas[1];
	// check_data=seven_datas[2];
	// data=~seven_datas[1]&0x7f;
	//		if     ((data&28)==28)	    Receive_data = 0;   //0011100
	//		else if((data&56)==56)	    Receive_data = 10;  //0111000
	//		else if((data&14)==14)	    Receive_data = -10; //0001110
	//		else if((data&122)==122) 	Receive_data = 20;  //1110000
	//		else if((data&7)==7)	    Receive_data = -20; //0000111
	//
	//		else if((data&24)==24)		Receive_data = 5;   //0011000
	//		else if((data&12)==12)		Receive_data = -5;  //0001100
	//		else if((data&48)==48)		Receive_data = 15;  //0110000
	//		else if((data&6)==6)	  	Receive_data = -15; //0000110
	//		else if((data&96)==96)		Receive_data = 25;  //1100000
	//		else if((data&3)==3)	  	Receive_data = -25; //0000011
	//
	//		else if((data&8)==8)	 	Receive_data = 0;   //0001000
	//		else if((data&16)==16)		Receive_data = 10;  //0010000
	//		else if((data&4)==4)	  	Receive_data = -10; //0000100
	//		else if((data&32)==32)		Receive_data = 20;  //0100000
	//		else if((data&2)==2)	  	Receive_data = -20; //0000010
	//		else if((data&64)==64)		Receive_data = 30;  //1000000
	//		else if((data&1)==1)	  	Receive_data = -30; //0000001
	//		else	                    Receive_data = 0;   //0000000
	//
	Receive_data = 0;
	float cumulate = 0;
	int js = 0;
	for (int i = 6; i >= 0; i--)
	{
		if (data & 1 << i)
		{
			cumulate += error[i];
			js++;
		}
	}
	Receive_data = cumulate / js * 10;
	return -Receive_data;

#endif
}

void sevenway_init()
{
	RingBuffer_Init(&sevenway_buffer);
	HAL_UART_Transmit_IT(&huart2, Byte, 2);
	HAL_UART_Receive_IT(&huart2, seven_datas, 3);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart2)
	{
		uint8_t data;
		int Receive_data = 0;
		HAL_UART_Transmit_IT(&huart2, Byte, 2);
		HAL_UART_Receive_IT(&huart2, seven_datas, 3);
		if (seven_datas[0] != 0x75)
		{
			return;
		}
		seven_ff = seven_datas[1];
	}

	// imu901 接收回调
	if (huart == &huart3)
	{
		imu901_rx_cplt_callback();
	}
}
static int prav_data = 0;

void error_calculate()
{
	float cumulate = 0;
	int js = 0;
	uint8_t data = ~seven_ff;
	for (int i = 6; i >= 0; i--)
	{
		if (data & 1 << i)
		{
			cumulate += error[i];
			js++;
		}
	}
	if ((seven_ff & 0XFF) != 0x7F)
		sevenway_data = -cumulate / js * 10;
	else
		sevenway_data = prav_data;
	prav_data = sevenway_data;
}
