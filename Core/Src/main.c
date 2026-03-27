// /* USER CODE BEGIN Header */
// /**
//   ******************************************************************************
//   * @file           : main.c
//   * @brief          : Main program body
//   ******************************************************************************
//   * @attention
//   *
//   * Copyright (c) 2024 STMicroelectronics.
//   * All rights reserved.
//   *
//   * This software is licensed under terms that can be found in the LICENSE file
//   * in the root directory of this software component.
//   * If no LICENSE file comes with this software, it is provided AS-IS.
//   *
//   ******************************************************************************
//   */
// /* USER CODE END Header */
// /* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "button.h"

// /* Private includes ----------------------------------------------------------*/
// /* USER CODE BEGIN Includes */
// /* USER CODE END Includes */
// /* Private typedef -----------------------------------------------------------*/
// /* USER CODE BEGIN PTD */
// /* USER CODE END PTD */
// /* Private define ------------------------------------------------------------*/
// /* USER CODE BEGIN PD */
void uint8_to_binary(uint8_t num, char *buffer)
{
    for (int i = 7; i >= 0; i--)
    {
        buffer[7 - i] = (num >> i) & 1 ? '1' : '0'; // 从最高位到最低位依次提取
    }
    buffer[8] = '\0'; // 字符串结束符
}
// /* USER CODE END PD */
// /* Private macro -------------------------------------------------------------*/
// /* USER CODE BEGIN PM */
// /* USER CODE END PM */
// /* Private variables ---------------------------------------------------------*/
// /* USER CODE BEGIN PV */
// /* USER CODE END PV */
// /* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
// /* USER CODE BEGIN PFP */
// /* USER CODE END PFP */
// /* Private user code ---------------------------------------------------------*/
// /* USER CODE BEGIN 0 */
short x, y, z;
uint8_t res1 = 8;

/* USER CODE END 0 */
/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */
    //   float pitch, roll, yaw;

    //    short	aacx, aacy, aacz;
    //    char k[20] = "";
    //    char k1[20] = "";
    //    char s1[10] = "";
    //    uint8_t s[1] = "n";
    //   uint8_t seven_commend[4] = {0x4c, 0x01, 0x01, 0x00};
    uint8_t seven_commend[4] = {0x4c, 0x01, 0x05, 0x06};
    //    uint8_t tx_data[6];
    encoder_total_l = 0;
    encoder_total_r = 0;
    char k[20];
    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_SPI2_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_UART5_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    MX_TIM1_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    MX_TIM8_Init();
    MX_TIM6_Init();
    MX_I2C1_Init();
    /* USER CODE BEGIN 2 */
    // Delay_ms(1000);
    // Encoder_Init();
    //  HAL_UART_Transmit(&huart2,seven_commend,4,0xffff);

    OLED_Init();

    HAL_Delay(500);
    uint8_t res = mpu_dmp_init();
    HAL_Delay(200);
    OLED_Update();
    Encoder_Init();
    sevenway_init();
    Servos_Init();

    motor_pid_init();
    seven_line_pid_init();
    mpu6050_pid_init();
    mpu6050_sevenway_init();
    OLED_Clear();
    motor_speed_set(0, 0);

    // static char te1[100] = {'f','S'};
    // static char te1[100] ={
    //     // manfen
    //     'O','2','K','L','2','R','2','L','d','f','t',
    //     // 2方块
    //     'O','d','b','t','L','1','L','7','w',
    //     // 1 黄圈
    //     'b','w','L','L','w','1','L','1','K','d','L','f','t',
    //     // 1方块
    //     'O','d','b','w','t','L','3','K','L','L','w','2','L','1','O',
    //     // 2 方块
    //     'b','w','L','L','w','1','L','2','L','1','K','L','L','w','1','R','2','R','1','O',
    // 'S','\0'}; //第一轮重启

    static char test_s[100] = {'t', 'w', 'd', 'T', 'w', 'D', 'S'}; // 测试用
    // static char test_s[100] = {'d','t','D','T'}; //测试用
    static char te1[100] = {
        // manfen
        'O', '2', 'K', 'L', '2', 'R', 'w', '2', 'w', 'L', 'T', 'f', 'd',
        // 2方块
        'O', 't', 'b', 'D', 'R', '1', 'K', 'L', '1', 'O', 'L', 'L', '1', 'L', '1', 'K', 'L', 'L', '1', 'R', '1', 'O', 'w',
        // 2方块
        'L', 'L', 'w', '8', 'w', 'f', 'w', 'b', 'w',
        // 1 圆环
        'L', 'L', '1', 'R', '1', 'K', 'T', 'R', 'f', 'd', 'O', 'b',
        // 2 方块
        'b', 'w', 'L', 'L', 'w', '1', 'L', '2', 'L', '1', 'K', 'L', 'L', 'w', '1', 'R', '2', 'R', '1', 'O',
        'S', '\0'}; // 第一轮重启

    static char test_9[100] = {
        'O', 'K', 'T', '1', 'L', 'f', 'd', 'd', 'S'

    };

    static char test1[100] = {'T', 'w', 'w', 'T', 'w', 'w', 'D', 'w', 'w', 'D', 'w', 'w', 'S'};                                                                                                                                                                                                                                                                        /*·测试代码*/
    /*满分环*/ static char test2[100] = {'O', '2', 'K', 'L', '1', 'R', '2', 'L', '1', 'T', 'w', 'w', 'w', 'T', 'w', 'w', 'f', 'O', 'B', 'D', 'w', 'w', 'D', 'w', 'w', 'R', '1', 'R', '7', 'S'};                                                                                                                                                                        // 满分环990
    /*物块八分*/ static char test3[100] = {'O', '1', 'R', '1', 'L', '2', 'L', '1', 'R', '2', 'K', 'R', 'O', '3', 'A', 'w', '8', 'A', '1', 'R', '2', 'K', 'A', '2', 'L', '1', 'O', 'B', 'A', '2', 'L', '2', 'L', '2', 'K', 'A', '2', 'R', '2', 'R', '2', 'O', 'A', '3', 'R', '1', 'K', 'A', '1', 'R', '5', 'O', 'A', '1', 'L', '2', 'K', 'A', '2', 'R', '1', 'O', 'S'}; // 方块八分960
                                                                                                                                                                                                                                                                                                                                                                       // static char test4[100] = {'D','w','w','D','R','1','R','7','S'};
    /*完胜*/ static char test5[100] = {'O', '1', 'L', '2', 'R', '1', 'K', '2', 'L', 'T', 'w', 'w', 'w', 'T', 'w', 'w', 'f', 'O', 'B', 'D', 'w', 'w', 'w', 'D', 'w', 'w', 'A', 'T', 'w', 'w', 'w', 'T', 'w', 'w', '1', 'K', 'A', '1', 'f', 'O', 'B', 'D', 'w', 'w', 'w', 'D', 'w', 'w', 'A', '1', 'L', '1', 'R', '6', 'S'};

    // static char test1[100] = {'2','R','1','R','1','L','3','d','O', 'L', '3', 'b', 'A', '1', 'R', '1','K', 'b','A','1','t','1','f','d','O','b','A','d',
    // 														'3','K','1','t','1','f','d','O','b','A','d','4','L','3','K','L','2','T','L','f','f','f','O','b','A','1',
    // 														'D','L','1','L','4','L','1','K','A','1','R','4','L','t','1','f','d','O','b','A','1','R',
    // 														'4','L','d','2','K','1','L','4','R','t','1','f','d','O','b','A','d','1','S','\0'};

    // 	static char test2[100] = {'O','2','H','L','1','R','1','L','2','R','T','f','f','f','O','b','A','D',
    // 														'2','K','R','1','t','R','4','L','1','f','d','O','b','A','d',
    // 														'1','R','4','L','2','K','1','L','t','4','R','1','f','d','O','b','A','d','1','S','\0'};//第一轮重启

    // 		////第二轮，前车，爪车1,去优先3分区，一个三分一个两分
    // 	static char test3[100]={'3','R','O','4','H','T','R','f','f','f','O','b','A','D',
    // 													'1','L','2','L','O','2','K','b','A','2','t','1','f','d','S','\0'};
    while (1)
    {
        // remote_data[1] = 0x00;
        // if(NRF24L01_RxPacket(remote_data))
        // {
        //     NRF24L01_RX_Mode();
        // }
        if (Button_IsPressed(BUTTON_PC0))
        {
            route(te1);
            // route(test_s);
            // route(test_9);
        }
    }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
