/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */

ISM330DHCX_Object_t myISM;
ISM330DHCX_IO_t io_ctx;
ISM330DHCX_Axes_t accelData, gyroData;

uint8_t ledState = GPIO_PIN_SET;

float pitch = 0.0, roll = 0.0;
float pitchAccel = 0.0, rollAccel = 0.0;
float kpitch = 0.98;
float kroll = 0.98;

CAN_TxHeaderTypeDef   TxHeader;
uint8_t               TxData[8];
uint32_t              TxMailbox;

CAN_TxHeaderTypeDef   TxHeader1;
uint8_t               TxData1[8];
uint32_t              TxMailbox1;

CAN_TxHeaderTypeDef   TxHeader2;
uint8_t               TxData2[8];
uint32_t              TxMailbox2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN2_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.StdId = 0x360;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 6;

  TxHeader1.IDE = CAN_ID_STD;
  TxHeader1.StdId = 0x361;
  TxHeader1.RTR = CAN_RTR_DATA;
  TxHeader1.DLC = 6;

  TxHeader2.IDE = CAN_ID_STD;
  TxHeader2.StdId = 0x362;
  TxHeader2.RTR = CAN_RTR_DATA;
  TxHeader2.DLC = 6;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN2_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);


  io_ctx.BusType  = ISM330DHCX_I2C_BUS;
  io_ctx.Address  = (0x6B << 1);
  io_ctx.Init     = MX_I2C1_Init;
//  io_ctx.DeInit   = MX_I2C1_DeInit;
  io_ctx.ReadReg  = BSP_I2C1_ReadReg;
  io_ctx.WriteReg = BSP_I2C1_WriteReg;
  io_ctx.GetTick  = HAL_GetTick;
  ISM330DHCX_RegisterBusIO(&myISM, &io_ctx);


  if (ISM330DHCX_Init(&myISM) != ISM330DHCX_OK) { // ISM330HDCX OK is equal to 0
	  	  printf("lmao your ism didnt work lil bro \n");
          Error_Handler();
      }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  if (device_reset(&myISM) != 0) {
	  printf("ism failed to reset \n");
	  Error_Handler();
  }

  config_ism(&myISM);
  //implement CAN here

  HAL_Delay(500);

  uint32_t prevTime = HAL_GetTick();
  float dt;
  int xAccel;
  int yAccel;
  int zAccel;
  int xGyro;
  int yGyro;
  int zGyro;
  while (1)
  {
	  // Note that setting it to 0 or GPIO_PIN_RESET will turn on the LED because of board tomfoolery
//	  HAL_Delay(1000);
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
//	  HAL_Delay(1000);
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
//	  HAL_Delay(1000);
//	  char msg[100];
//	  sprintf(msg, "burh this be cringlee");
//	  CDC_Transmit_FS((uint8_t*) msg, strlen(msg));
//	  HAL_Delay(100);
	  printf("cringlee");

	  if (ISM330DHCX_ACC_GetAxes(&myISM, &accelData) == ISM330DHCX_OK && ISM330DHCX_GYRO_GetAxes(&myISM, &gyroData) == ISM330DHCX_OK) {
		  dt = ((float) HAL_GetTick() - (float) prevTime) / 1000.0;
		  prevTime = HAL_GetTick();

		  xAccel = (int) accelData.x;
		  yAccel = (int) accelData.y;
		  zAccel = (int) accelData.z;

		  //write to CAN
		  TxData[0] = (xAccel & 0xFF000000) >> 24;
		  TxData[1] = (xAccel & 0x00FF0000) >> 16;
		  TxData[2] = (xAccel & 0x0000FF00) >> 8;
		  TxData[3] = xAccel & 0xFF;
		  TxData[4] = (yAccel & 0xFF000000) >> 24;
		  TxData[5] = (yAccel & 0x00FF0000) >> 16;
		  TxData[6] = (yAccel & 0x0000FF00) >> 8;
		  TxData[7] = yAccel & 0xFF;
		  if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK) { // this only throws error if it stores the message in mailbox incorrectly (realistically)
//		     Error_Handler();
			  HAL_Delay(50);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
			  HAL_Delay(50);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
			  HAL_Delay(50);
		  }

		  // Fill half of second can message
		  TxData1[0] = (zAccel & 0xFF000000) >> 24;
		  TxData1[1] = (zAccel & 0x00FF0000) >> 16;
		  TxData1[2] = (zAccel & 0x0000FF00) >> 8;
 		  TxData1[3] = zAccel & 0xFF;

		  xGyro = (int) gyroData.x;
		  yGyro = (int) gyroData.y;
		  zGyro = (int) gyroData.z;

		  TxData[4] = (xGyro * 0xFF000000) >> 24;
		  TxData1[5] = (xGyro & 0x00FF0000) >> 16;
		  TxData1[6] = (xGyro & 0x0000FF00) >> 8;
  		  TxData1[7] = xGyro & 0xFF;

		  if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader1, TxData1, &TxMailbox1) != HAL_OK) {
//			 Error_Handler();
			  HAL_Delay(50);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
			  HAL_Delay(50);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
			  HAL_Delay(50);
		  }

		  //write third CAN message
		  TxData2[0] = (yGyro & 0xFF000000) >> 24;
		  TxData2[1] = (yGyro & 0x00FF0000) >> 16;
		  TxData2[2] = (yGyro & 0x0000FF00) >> 8;
		  TxData2[3] = yGyro & 0xFF;
		  TxData2[4] = (zGyro & 0xFF000000) >> 24;
		  TxData2[5] = (zGyro & 0x00FF0000) >> 16;
		  TxData2[6] = (zGyro & 0x0000FF00) >> 8;
		  TxData2[7] = zGyro & 0xFF;
		  if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader2, TxData2, &TxMailbox2) != HAL_OK) {
//			 Error_Handler();
			  HAL_Delay(50);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
			  HAL_Delay(50);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
			  HAL_Delay(50);
		  }

//		  pitchAccel = atan2(-yAccel, sqrt(pow(xAccel, 2) + pow(zAccel, 2))) * (180000.0f / M_PI);
//		  rollAccel = atan2(xAccel, sqrt(pow(yAccel, 2) + pow(zAccel, 2))) * (180000.0f / M_PI);
//		  pitch = kpitch * (pitch + ((float)xGyro * dt)) + (1.0f - kpitch) * pitchAccel;
//
//		  printf("%f\t%f\n", pitchAccel * 180000.0/3.1415, pitch);

	  } else {
		  Error_Handler();
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  static uint32_t timeout = 0;
	  if (HAL_GetTick() - timeout > 1000) {
		  ledState = (ledState == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, ledState);
		  timeout = HAL_GetTick();
	  }

	  HAL_Delay(5);
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
	hcan2.Instance = CAN2;
	hcan2.Init.Prescaler = 2;
	hcan2.Init.Mode = CAN_MODE_NORMAL;
	hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan2.Init.TimeSeg1 = CAN_BS1_16TQ;
	hcan2.Init.TimeSeg2 = CAN_BS2_4TQ;
	hcan2.Init.TimeTriggeredMode = DISABLE;
	hcan2.Init.AutoBusOff = ENABLE;
	hcan2.Init.AutoWakeUp = DISABLE;
	hcan2.Init.AutoRetransmission = DISABLE;
	hcan2.Init.ReceiveFifoLocked = DISABLE;
	hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
  if (HAL_CAN_Start(&hcan2) != HAL_OK)
    {
  	  Error_Handler();
    }
  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int device_reset(ISM330DHCX_Object_t *ism) {
	ism330dhcx_ctrl3_c_t ctrl3_c;
	int32_t ret;

	ret = ism330dhcx_read_reg(&ism->Ctx, ISM330DHCX_CTRL3_C, (uint8_t *)&ctrl3_c, 1);
	if (ret != 0) {
		return ret;
	}

	ctrl3_c.sw_reset = 1; // turns on reset
	ret = ism330dhcx_write_reg(&ism->Ctx, ISM330DHCX_CTRL3_C, (uint8_t *)&ctrl3_c, 1);
	if (ret != 0) {
		return ret;
	}

	do { //check to make sure register is updated
		HAL_Delay(10);
		ret = ism330dhcx_read_reg(&ism->Ctx, ISM330DHCX_CTRL3_C, (uint8_t *)&ctrl3_c, 1);
		if (ret != 0) {
			return ret;
		}
	} while (ctrl3_c.sw_reset != 0);

	return 0;
}

void config_ism(ISM330DHCX_Object_t *ism) {
    // set acc data rate to 208hz
    if (ISM330DHCX_ACC_SetOutputDataRate(ism, 208.0f) != ISM330DHCX_OK) {
        Error_Handler();
    }
    // set acc data to scale to +- 2 G's
    if (ISM330DHCX_ACC_SetFullScale(ism, 2) != ISM330DHCX_OK) {
        Error_Handler();
    }

    // set gyro data rate to 208hz
    if (ISM330DHCX_GYRO_SetOutputDataRate(ism, 208.0f) != ISM330DHCX_OK) {
        Error_Handler();
    }
    //set gyro scaling to 500 dps
    if (ISM330DHCX_GYRO_SetFullScale(ism, 500) != ISM330DHCX_OK) {
        Error_Handler();
    }

    // enable lpf2 filter
    if (ism330dhcx_xl_filter_lp2_set(&(ism->Ctx),1) != ISM330DHCX_OK) {
        Error_Handler();
    }

    // some filter
    if (ISM330DHCX_Write_Reg(ism, ISM330DHCX_CTRL1_XL, 0x00) != ISM330DHCX_OK) {
        Error_Handler();
    }

//    // gyro lpf1 filter
//    if (ISM330DHCX_GYRO_EnableLPF1(ism) != ISM330DHCX_OK) {
//        Error_Handler();
//    }
//
//    // josh cooked here? soemthing to do with setting the bandwidth to agressive
//    if (ISM330DHCX_Write_Reg(ism, ISM330DHCX_CTRL4_C, 0x00) != ISM330DHCX_OK) {
//        Error_Handler();
//    }
    if (ISM330DHCX_ACC_Enable(&myISM) != ISM330DHCX_OK || ISM330DHCX_GYRO_Enable(&myISM) != ISM330DHCX_OK) {
        Error_Handler();
     }
}

static int32_t BSP_I2C1_ReadReg(uint8_t Address, uint8_t Reg, uint8_t *pData, uint16_t Length) {
  if (HAL_I2C_Mem_Read(&hi2c1, Address, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length, HAL_MAX_DELAY) == HAL_OK) {
    return 0;
  }
  return -1;
}

static int32_t BSP_I2C1_WriteReg(uint8_t Address, uint8_t Reg, uint8_t *pData, uint16_t Length) {
  if (HAL_I2C_Mem_Write(&hi2c1, Address, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length, HAL_MAX_DELAY) == HAL_OK) {
    return 0;
  }
  return -1;
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
//  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
  while (1)
  {
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	  HAL_Delay(100);
  }
  /* USER CODE END Error_Handler_Debug */
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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
