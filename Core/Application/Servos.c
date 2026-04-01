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

	HAL_UART_Transmit(&huart1, (uint8_t *)"#001PMOD1\r\n", 9, 0xffff);
	Delay_ms(100);
	HAL_UART_Transmit(&huart1, (uint8_t *)"#000PMOD6\r\n", 9, 0xffff);
}

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
			HAL_UART_Transmit(&huart1, (uint8_t *)"#1P1000T100\r\n", 13, 0xffff);//#1P1100T100
			break;
		case CLOSE:
			HAL_UART_Transmit(&huart1, (uint8_t *)"#1P1400T100\r\n", 13, 0xffff);
			break;
	}
}

//��ץ�½�
void Servos_down(int position) {
	 sprintf(k,"#000P1154T000%d\r\n",position);
	 HAL_UART_Transmit_IT(&huart1,(uint8_t *)k ,16);
	 delay_20ms(20);
}

//��ץ����
void Servos_up(int position) {
	sprintf(k,"#000P2229T000%d\r\n",position);
	HAL_UART_Transmit_IT(&huart1,(uint8_t *)k ,16);
	delay_20ms(10);
} 

//��ץ�ſ�
void Servos_open(int position) {	
	sprintf(k,"#001P%dT0500\r\n",position);
	HAL_UART_Transmit_IT(&huart1,(uint8_t *)k ,16);
	delay_20ms(10);
}
//��ץ�պ�
void Servos_close(int position) {
    sprintf(k,"#001P%dT0500\r\n",position);
	HAL_UART_Transmit_IT(&huart1,(uint8_t *)k ,16);
	delay_20ms(10);
}

