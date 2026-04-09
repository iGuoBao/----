#include "main.h"
#include "Servos.h"
static char k[16]="";
void delay(int time)
{
	int t=0;
	while(1){
		if(receive_flag){
			receive_flag=0;
			t++;
		}
		if(t>time)break;
	}
}

void delay_20ms(int time)
{
	int t=0;
	while(1){
		if(receive_flag){
			receive_flag=0;
			t++;
		}
		if(t>time)break;
	}
}


void Servos_Init(void){
#if defined(NEW_SERVO_1)
	HAL_UART_Transmit(&huart1, (uint8_t *)"#001PMOD1!\r\n", 10, 0xffff);
#else
	HAL_UART_Transmit(&huart1, (uint8_t *)"#001PMOD1\r\n", 9, 0xffff);
#endif

#if defined(NEW_SERVO_0)
	Delay_ms(100);
	HAL_UART_Transmit(&huart1, (uint8_t *)"#000PMOD8!\r\n", 10, 0xffff);
}
#else
	Delay_ms(100);
	HAL_UART_Transmit(&huart1, (uint8_t *)"#000PMOD8\r\n", 9, 0xffff);
}
#endif

void Servos_Lift(int lift){
	switch(lift){
		case NONE:
			HAL_UART_Transmit(&huart1, (uint8_t *)"#2P1500T100\r\n", 13, 0xffff);
			break;
		case UP:
			HAL_UART_Transmit(&huart1, (uint8_t *)"#2P1100T100\r\n", 13, 0xffff);
			break;
		case DOWN:
			HAL_UART_Transmit(&huart1, (uint8_t *)"#2P1900T100\r\n", 13, 0xffff);
			break;
	}
}

void Servos_Retract(int retract){
	switch(retract){
		case NONE:
			break;
		case OPEN:
			HAL_UART_Transmit(&huart1, (uint8_t *)"#1P1000T100!\r\n", 13, 0xffff);//#1P1100T100
			break;
		case CLOSE:
			HAL_UART_Transmit(&huart1, (uint8_t *)"#1P1400T100!\r\n", 13, 0xffff);
			break;
	}
}


void Servos_down(int position) {
#if defined(NEW_SERVO_0)
	sprintf(k,"#000P1054T000!%d\r\n",position);
	HAL_UART_Transmit_IT(&huart1,(uint8_t *)k ,19);
#else
	sprintf(k,"#000P0954T000%d\r\n",position);
	HAL_UART_Transmit_IT(&huart1,(uint8_t *)k ,18);
#endif
	 
	 delay_20ms(150);
}


void Servos_up(int position) {
#if defined(NEW_SERVO_0)
	sprintf(k,"#000P2229T000!%d\r\n",position);
	HAL_UART_Transmit_IT(&huart1,(uint8_t *)k ,19);
#else
	sprintf(k,"#000P2229T000%d\r\n",position);
	HAL_UART_Transmit_IT(&huart1,(uint8_t *)k ,18);
#endif
	delay_20ms(150);
} 


void Servos_open(int position) {	
#if defined(NEW_SERVO_1)
	sprintf(k,"#001P%4dT0300!\r\n",position);
	HAL_UART_Transmit_IT(&huart1,(uint8_t *)k ,17);
#else
	sprintf(k,"#001P%4dT0300\r\n",position);
	HAL_UART_Transmit_IT(&huart1,(uint8_t *)k ,16);
#endif
	delay_20ms(25);
}

void Servos_close(int position) {
#if defined(NEW_SERVO_1)
    sprintf(k,"#001P%4dT0300!\r\n",position);
	HAL_UART_Transmit_IT(&huart1,(uint8_t *)k ,17);
#else
	sprintf(k,"#001P%4dT0300\r\n",position);
	HAL_UART_Transmit_IT(&huart1,(uint8_t *)k ,16);
#endif
	delay_20ms(25);
}

