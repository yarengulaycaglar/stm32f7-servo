#include "main.h"
#include <stdio.h>

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
void Error_Handler(void);

FDCAN_HandleTypeDef hfdcan1;
int main(void)
{
	/* Sistem başlatma */
	printf("Main start...\n");
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_FDCAN1_Init();

	uint8_t controller_id = 1;

	enter_motor_control_mode(controller_id, &hfdcan1);

	while (1)
	{
		/* RPM komutunu FDCAN üzerinden gönder */
		pack_cmd(controller_id,p_des, v_des, kp, kd, t_ff, &hfdcan1);
		HAL_Delay(100);
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

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 24;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
			|RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief FDCAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_FDCAN1_Init(void)
{
	hfdcan1.Instance = FDCAN1;
	hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
	hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
	hfdcan1.Init.AutoRetransmission = DISABLE;
	hfdcan1.Init.TransmitPause = DISABLE;
	hfdcan1.Init.ProtocolException = DISABLE;
	hfdcan1.Init.NominalPrescaler = 16;
	hfdcan1.Init.NominalSyncJumpWidth = 1;
	hfdcan1.Init.NominalTimeSeg1 = 2;
	hfdcan1.Init.NominalTimeSeg2 = 2;
	hfdcan1.Init.DataPrescaler = 1;
	hfdcan1.Init.DataSyncJumpWidth = 1;
	hfdcan1.Init.DataTimeSeg1 = 1;
	hfdcan1.Init.DataTimeSeg2 = 1;
	hfdcan1.Init.MessageRAMOffset = 0;
	hfdcan1.Init.StdFiltersNbr = 0;
	hfdcan1.Init.ExtFiltersNbr = 0;
	hfdcan1.Init.RxFifo0ElmtsNbr = 0;
	hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
	hfdcan1.Init.RxFifo1ElmtsNbr = 0;
	hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
	hfdcan1.Init.RxBuffersNbr = 0;
	hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
	hfdcan1.Init.TxEventsNbr = 0;
	hfdcan1.Init.TxBuffersNbr = 0;
	hfdcan1.Init.TxFifoQueueElmtsNbr = 0;
	hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
	hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
	if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
	{
		Error_Handler();
	}

	/* FDCAN RX interrupt'ını etkinleştir */
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

	// Enable interrupts
	HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

	/* FDCAN'ı başlat */
	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
	{
		Error_Handler(); // FDCAN başlatma hatası
	}
}

void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* FDCAN1 pinlerini (PD0: RX, PD1: TX) yapılandır */
	__HAL_RCC_GPIOD_CLK_ENABLE();  // GPIOD için saat aktif et
	__HAL_RCC_FDCAN_CLK_ENABLE();

	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;  // FDCAN1 pin alternatifi
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}


/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
}

// HAL callback fonksiyonu
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	FDCAN_RxHeaderTypeDef rx_msg;

	if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_msg, msg) == HAL_OK)
	{
		// Mesaj başarıyla alındı
		unpack_reply();
	}
}


#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
