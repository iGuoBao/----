#include "main.h"
#include "Servos.h"
static char k[22]="";
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


void Servos_down(int position) {
#if defined(NEW_SERVO_0)
	int _pos = SERVO_RETRACT_PWM;
	sprintf(k,"#000%04dT000%d!\r\n", _pos, position);
	HAL_UART_Transmit_IT(&huart1,(uint8_t *)k ,strlen(k));
#else
	int _pos = SERVO_RETRACT_PWM;
	sprintf(k,"#000P%04dT000%d\r\n", _pos, position);
	HAL_UART_Transmit_IT(&huart1,(uint8_t *)k ,strlen(k));
#endif
	 
	 delay_20ms(50);
}

void Servos_up(int position) {
#if defined(NEW_SERVO_0)
	int _pos = SERVO_LIFT_PWM;
	sprintf(k,"#000%04dT000%d!\r\n", _pos, position);
	HAL_UART_Transmit_IT(&huart1,(uint8_t *)k ,strlen(k));
#else
	int _pos = SERVO_LIFT_PWM;
	sprintf(k,"#000P%04dT000%d\r\n", _pos, position);
	HAL_UART_Transmit_IT(&huart1,(uint8_t *)k ,strlen(k));
#endif
	delay_20ms(50);
} 

void Servos_open(int position) {	
#if defined(NEW_SERVO_1)
	sprintf(k,"#001P%4dT0300!\r\n",position);
	HAL_UART_Transmit_IT(&huart1,(uint8_t *)k ,strlen(k));
#else
	sprintf(k,"#001P%4dT0300\r\n",position);
	HAL_UART_Transmit_IT(&huart1,(uint8_t *)k ,strlen(k));
#endif
	delay_20ms(5);
}

void Servos_close(int position) {
#if defined(NEW_SERVO_1)
    sprintf(k,"#001P%4dT0300!\r\n",position);
	HAL_UART_Transmit_IT(&huart1,(uint8_t *)k ,strlen(k));
#else
	sprintf(k,"#001P%4dT0300\r\n",position);
	HAL_UART_Transmit_IT(&huart1,(uint8_t *)k ,strlen(k));
#endif
	delay_20ms(5);
}

