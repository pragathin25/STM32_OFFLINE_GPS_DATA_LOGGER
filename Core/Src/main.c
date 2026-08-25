

/* USER CODE BEGIN Header */

/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "nand.h"
#include "lfs.h"
#include "lfs_port.h"
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

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
#define RX_BUFFER_SIZE 300
uint8_t rx_char;   //stores one received UART character.
char rx_buffer[RX_BUFFER_SIZE];
volatile uint16_t rx_index = 0;//stores the current position inside rx_buffer

#define LIS2DH12_ADDR     (0x19 << 1)//Defines the I2C address
#define CTRL_REG1         0x20  //Accelerometer xyz enable
#define OUT_X_L           0x28

uint32_t last_output_time = 0;

char gps_date[16] = "NA";
char gps_time_ist[16] = "NA";
double latitude_dd = 0.0;
double longitude_dd = 0.0;         //Stores date time lon lat after gps parsing

uint8_t nand_id[3];                //Stores flsh id


lfs_file_t file;  //Creates a LittleFS file object.,,currently opened file
char current_file_name[32];

uint8_t gps_valid = 0;

uint8_t file_opened = 0;
char last_file_name[32] = "";

uint8_t command_mode = 0;     // 0 = logging, 1 = command mode
uint8_t cmd_char;


#define MAX_FILES 10

char file_list[MAX_FILES][32];   // store file names
uint8_t file_count = 0;

uint8_t read_mode = 0;   // 0 = normal, 1 = waiting for a/b/c


uint8_t delete_mode = 0;   // Controls file deletion selection.

uint8_t download_mode = 0;

volatile uint8_t delete_request = 0;
volatile uint8_t delete_index = 0;//Stores which file should be deleted.
volatile uint8_t list_request = 0;//Indicates that a file-list operation has been requested.

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART5_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void LIS2DH12_Init(void);
void LIS2DH12_Read(int16_t *x, int16_t *y, int16_t *z);

double NMEA_To_Decimal(char *coord);
void Parse_GNRMC(char *sentence);
void Increment_Date(char *date);

void Download_File_By_Index(uint8_t index);
void List_Files(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart5, (uint8_t*)ptr, len, HAL_MAX_DELAY);          // Redirects printf() to UART5.
    return len;

}
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART5_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart3, (uint8_t*)&rx_char, 1);
  HAL_UART_Receive_IT(&huart5, &cmd_char, 1);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);    // Makes WP pin HIGH.
  // Start GPS UART

  // NAND INIT
  NAND_Init();
  printf("NAND INIT DONE\r\n");

  uint8_t status = NAND_ReadStatus();
  printf("STATUS AFTER INIT = 0x%02X\r\n", status);
  //  READ ID
  NAND_ReadID(nand_id);

  // PRINT ID
  printf("NAND ID: %02X %02X %02X\r\n",
         nand_id[0], nand_id[1], nand_id[2]); // EF AA 21

  HAL_Delay(100);
  if (LittleFS_Init() != 0)
  {
      printf("LittleFS Init Failed\r\n");

      printf("Formatting...\r\n");

      if (LittleFS_Format() != 0)
      {
          printf("Format Failed\r\n");
          Error_Handler();
      }

      printf("Format Done\r\n");

      // ⭐ ADD THIS BLOCK RIGHT HERE
      if (LittleFS_Init() != 0)
      {
          printf("Mount failed after format\r\n");
          Error_Handler();
      }

      printf("Mounted after format\r\n");
  }
  else
  {
      printf("LittleFS Mounted Successfully\r\n");
  }

  // Accelerometer init
  LIS2DH12_Init();

  HAL_Delay(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    if (delete_request)
	    {
	        delete_request = 0;//don't want to delete the file repeatedly.

	        Delete_File_By_Index(delete_index);//which file to delete.
	    }

	    if (list_request)
	    {
	        list_request = 0;

	        List_Files();
	    }

	  if (!command_mode && (HAL_GetTick() - last_output_time >= 5000))
	  {
	         last_output_time = HAL_GetTick();

	         int16_t x, y, z;
	         LIS2DH12_Read(&x, &y, &z);

	         char log_line[200];

	         if(gps_valid)
	         {
	             sprintf(log_line,
	                     "DATE=%s TIME=%s LAT=%.6f LON=%.6f X=%d Y=%d Z=%d\r\n",
	                     gps_date, gps_time_ist,latitude_dd,longitude_dd, x, y, z);

	             printf("%s", log_line);
	         }
	         else
	         {
	             printf("Waiting for GPS fix...\r\n");
	             continue;
	         }


	         if(gps_valid && strlen(gps_date) > 5)   //GPS date string must contain more than 5 characters
	         {
	        	 char dd[3];
	        	 char mm[3];
	        	 char yyyy[5];

	        	 sscanf(gps_date, "%2s/%2s/%4s", dd,mm,yyyy);

	        	 sprintf(current_file_name,"/%s%s%s.txt",yyyy,mm, dd);//This creates your NAND filename.

	        	 //printf("FILENAME = %s\r\n", current_file_name);
	        	 if (!file_opened || strcmp(last_file_name, current_file_name) != 0)
	        	 {
	        	     if (file_opened)//If there is already an open file, close it.
	        	     {
	        	         lfs_file_close(&lfs, &file);
	        	       //  printf("OLD FILE CLOSED\r\n");
	        	         file_opened = 0;
	        	     }

	        	    // printf("Opening file: %s\r\n", current_file_name);
	        	    // printf("Before open\r\n");

	        	     //Open this file so I can append new data to it
	        	     int res_open = lfs_file_open( &lfs, &file,current_file_name, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_APPEND);

	        	    // printf("After open\r\n");
	        	    // printf("OPEN RES = %d\r\n", res_open);

	        	     if (res_open < 0)
	        	     {
	        	        // printf("FILE OPEN FAILED\r\n");
	        	         continue;
	        	     }

	        	     file_opened = 1;//A file is currently open.
	        	     strcpy(last_file_name, current_file_name); //copies cfn to lfn
	        	 }

	        	 // ✅ WRITE
	        	// printf("WRITE START\r\n");

	        	 int res_write = lfs_file_write(&lfs, &file, log_line, strlen(log_line));//writes your log data into the LittleFS file
	        	 //printf("WRITE RES = %d\r\n", res_write);

	        	 // ✅ SYNC
	        	 //printf("SYNC START\r\n");
	        	 int res_sync = lfs_file_sync(&lfs, &file);//Make sure the pending file changes are synchronized to the storage.

	        	 HAL_Delay(5);   // ⭐ IMPORTANT for NAND
	        	 //printf("SYNC RES = %d\r\n", res_sync);
	         }

	    }
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x00000E14;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  SPI_AutonomousModeConfTypeDef HAL_SPI_AutonomousMode_Cfg_Struct = {0};

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x7;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi1.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi1.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerState = SPI_AUTO_MODE_DISABLE;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerSelection = SPI_GRP1_GPDMA_CH0_TCF_TRG;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerPolarity = SPI_TRIG_POLARITY_RISING;
  if (HAL_SPIEx_SetConfigAutonomousMode(&hspi1, &HAL_SPI_AutonomousMode_Cfg_Struct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 38400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, nand_hold_Pin|nand_cs_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

  /*Configure GPIO pins : nand_hold_Pin nand_cs_Pin PA8 */
  GPIO_InitStruct.Pin = nand_hold_Pin|nand_cs_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void LIS2DH12_Init(void)
{
    uint8_t data;

    // 100Hz, XYZ enable
    data = 0x57;
    HAL_I2C_Mem_Write(&hi2c1,
                      LIS2DH12_ADDR,
                      CTRL_REG1,
                      I2C_MEMADD_SIZE_8BIT,
                      &data,
                      1,
                      HAL_MAX_DELAY);

    // High resolution, BDU enable, ±2g
    data = 0x88;
    HAL_I2C_Mem_Write(&hi2c1,
                      LIS2DH12_ADDR,
                      0x23,
                      I2C_MEMADD_SIZE_8BIT,
                      &data,
                      1,
                      HAL_MAX_DELAY);
}


void LIS2DH12_Read(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t rawData[6];
    int16_t raw_x, raw_y, raw_z;

    HAL_I2C_Mem_Read(&hi2c1,
                     LIS2DH12_ADDR,
                     OUT_X_L | 0x80,// auto incr
                     I2C_MEMADD_SIZE_8BIT,
                     rawData,
                     6,
                     HAL_MAX_DELAY);

    raw_x = (int16_t)((rawData[1] << 8) | rawData[0]);//ombines two 8-bit values into one 16-bit value
    raw_y = (int16_t)((rawData[3] << 8) | rawData[2]);
    raw_z = (int16_t)((rawData[5] << 8) | rawData[4]);

    // 12-bit data in high-resolution mode
    raw_x = raw_x / 16;//left-aligned 12-bit format within the 16-bit register pair.
    raw_y = raw_y / 16;
    raw_z = raw_z / 16;

    *x = raw_x;
    *y = raw_y;
    *z = raw_z;
}
double NMEA_To_Decimal(char *coord)
{
    double value;
    int degrees;
    double minutes;

    value = atof(coord);//ASCII to floating-point.
    degrees = (int)(value / 100);
    minutes = value - (degrees * 100);
    return degrees + (minutes / 60.0);
}

void Increment_Date(char *date)//moves a date forward by exactly one day
{
    int dd, mm, yyyy;
    int days_in_month;

    sscanf(date, "%2d/%2d/%4d", &dd, &mm, &yyyy);

    // Check number of days in current month
    switch (mm)//which month you're currently in
    {
        case 1:
        case 3:
        case 5:
        case 7:
        case 8:
        case 10:
        case 12:
            days_in_month = 31;
            break;

        case 4:
        case 6:
        case 9:
        case 11:
            days_in_month = 30;
            break;

        case 2:
            // Leap year check
            if ((yyyy % 400 == 0) ||
                ((yyyy % 4 == 0) && (yyyy % 100 != 0)))
            {
                days_in_month = 29;
            }
            else
            {
                days_in_month = 28;
            }
            break;

        default:
            return;
    }

    dd++;

    if (dd > days_in_month)
    {
        dd = 1;
        mm++;

        if (mm > 12)
        {
            mm = 1;
            yyyy++;
        }
    }

    sprintf(date, "%02d/%02d/%04d", dd, mm, yyyy);
}


void Parse_GNRMC (char *sentence)
{
	if(strlen(sentence) < 20) return;

    char temp[150];
    strcpy(temp, sentence);
    char *token;
    uint8_t field = 0;//which comma-separated field it is currently processing.
    char raw_time[20] = {0};//stores the GPS UTC time
    char raw_lat[20] = {0};//stores latitude in NMEA format
    char lat_dir = 'N';
    char raw_lon[20] = {0};
    char lon_dir = 'E';
    char status = 'V';
    char *ptr = temp;
    while((token = strsep(&ptr, ",")) != NULL)//separates a string using a delimiter.(,)
    {
        switch(field)
        {
            case 1:
                strcpy(raw_time, token);
                break;

            case 2:
                 status = token[0];   // ⭐ ADD THIS (VERY IMPORTANT)
                 break;

            case 3:
                strcpy(raw_lat, token);
                break;

            case 4:
                lat_dir = token[0];
                break;

            case 5:
                strcpy(raw_lon, token);
                break;

            case 6:
                lon_dir = token[0];
                break;

            case 9:
                if(strlen(token) == 6 && status == 'A')//25. Check date length and GPS status
                {
                    char dd[3], mm[3], yy[3];

                    strncpy(dd, token, 2);
                    dd[2] = '\0';

                    strncpy(mm, token + 2, 2);
                    mm[2] = '\0';

                    strncpy(yy, token + 4, 2);
                    yy[2] = '\0';

                    sprintf(gps_date, "%s/%s/20%s", dd, mm, yy);
                }
                break;
        }
        field++;
    }

    // Convert UTC to IST
    // ⭐ CHECK GPS VALIDITY
    if(status == 'A')
    {
        gps_valid = 1;
    }
    else
    {
        gps_valid = 0;
        return;   // 🚨 STOP updating invalid data
    }
    int hh, mm, ss;

    sscanf(raw_time, "%2d%2d%2d", &hh, &mm, &ss);

    /* UTC -> IST */
    hh += 5;
    mm += 30;

    if (mm >= 60)
    {
        mm -= 60;
        hh++;
    }


    if (hh >= 24)
    {
        hh -= 24;//midnight crossing

        // Move date to next day
        Increment_Date(gps_date);
    }

    sprintf(gps_time_ist, "%02d:%02d:%02d", hh, mm, ss);
    // Convert latitude & longitude
    if(strlen(raw_lat) > 0 && status == 'A')
    {
        latitude_dd = NMEA_To_Decimal(raw_lat);

        if(lat_dir == 'S')
        {
            latitude_dd = -latitude_dd;
        }
    }
    if(strlen(raw_lon) > 0 && status == 'A')
    {
        longitude_dd = NMEA_To_Decimal(raw_lon);

        if(lon_dir == 'W')
        {
            longitude_dd = -longitude_dd;
        }
    }
}
void List_Files(void)
{
    lfs_dir_t dir;//dir represents the folder being examined
    struct lfs_info info;//This structure is filled by LittleFS with information about each directory entry.

    file_count = 0;

    printf("\r\n--- FILE LIST START ---\r\n");

    if (lfs_dir_open(&lfs, &dir, "/") < 0)
    {
        printf("Failed to open directory\r\n");
        return;
    }

    char label = 'a';

    while (1)
    {
        int res = lfs_dir_read(&lfs, &dir, &info);//Give me the next item in this directory

        if (res <= 0)
            break;

        if (strcmp(info.name, ".") == 0 ||//. Current directory.
            strcmp(info.name, "..") == 0)//..  Parent directory.
        {
            continue;
        }

        if (info.type == LFS_TYPE_REG)
        {
            if (file_count < MAX_FILES)
            {
                strcpy(file_list[file_count], info.name);

                printf("%c. %s\r\n",
                       label,
                       file_list[file_count]);

                file_count++;
                label++;
            }
        }
    }

    lfs_dir_close(&lfs, &dir);

    printf("--- FILE LIST END ---\r\n");
    printf("Total files: %d\r\n", file_count);
}

void Read_File_By_Index(uint8_t index)//Take a file index, open that file from NAND, read its contents, and send the contents through UART.
{
    if (index >= file_count)
    {
        printf("Invalid file selection\r\n");
        return;
    }

    lfs_file_t rfile;

    if (lfs_file_open(&lfs, &rfile, file_list[index], LFS_O_RDONLY) < 0)//Open the file selected by index in read-only mode.
    {
        printf("File open failed\r\n");
        return;
    }

    printf("\r\n--- FILE DATA (%s) ---\r\n", file_list[index]);

    char buffer[64];
    int bytes;

    while ((bytes = lfs_file_read(&lfs, &rfile, buffer, sizeof(buffer)-1)) > 0)
    {
        buffer[bytes] = '\0';
        printf("%s", buffer);
    }

    lfs_file_close(&lfs, &rfile);

    printf("\r\n--- END OF FILE ---\r\n");
}

void Download_File_By_Index(uint8_t index)
{
    if(index >= file_count)
    {
        printf("Invalid file\r\n");
        return;
    }

    lfs_file_t dfile;//creates a LittleFS file object

    if(lfs_file_open(&lfs,
                     &dfile,
                     file_list[index],
                     LFS_O_RDONLY) < 0)
    {
        printf("Open failed\r\n");
        return;
    }

    printf("<START_FILE>\r\n");
    printf("FILENAME=%s\r\n", file_list[index]);

    char buffer[64];
    int bytes;

    while((bytes = lfs_file_read(&lfs,
                                 &dfile,
                                 buffer,
                                 sizeof(buffer))) > 0)//ittleFS will try to read up to 64 bytes at a time.
    {
        HAL_UART_Transmit(&huart5,
                          (uint8_t *)buffer,
                          bytes,
                          HAL_MAX_DELAY);
    }

    printf("\r\n<END_FILE>\r\n");

    lfs_file_close(&lfs,&dfile);
}


void Delete_File_By_Index(uint8_t index) //Delete a selected file from the LittleFS filesystem on the SPI NAND.
{
    if (index >= file_count)
    {
        printf("Invalid file selection\r\n");
        return;
    }

    char full_path[40];
    sprintf(full_path, "/%s", file_list[index]);

    // 🚨 If currently open → close it
    if (file_opened && strcmp(last_file_name, full_path) == 0)
    {
        lfs_file_close(&lfs, &file);
        file_opened = 0;
        last_file_name[0] = '\0';
    }

    int res = lfs_remove(&lfs, full_path);

    if (res < 0)
    {
        printf("Delete FAILED (%d): %s\r\n", res, full_path);
    }
    else
    {
        printf("Deleted: %s\r\n", full_path);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        if (rx_char == '$')// begin of nmea
        {
            rx_index = 0;//$ = 0
            rx_buffer[rx_index++] = rx_char;
        }
        else if (rx_char == '\n' || rx_char == '\r')//end
        {
            rx_buffer[rx_index] = '\0';

            //printf("LINE: %s\r\n", rx_buffer);

            if (strstr(rx_buffer, "$GNRMC") || strstr(rx_buffer, "$GPRMC"))//asks Does rx_buffer contain $GNRMC
            {
            	Parse_GNRMC(rx_buffer);
            }

            rx_index = 0;
        }
        else
        {
            if (rx_index < RX_BUFFER_SIZE - 1)//prevents the program from writing beyond the end of the buffer
            {
                rx_buffer[rx_index++] = rx_char;
            }
        }

        HAL_UART_Receive_IT(&huart3, (uint8_t*)&rx_char, 1);
    }
    if (huart->Instance == UART5)
    {
    	printf("RX CMD: %c\r\n", cmd_char);

    	if (cmd_char == '\r' || cmd_char == '\n')
    	{
    	    HAL_UART_Receive_IT(&huart5, &cmd_char, 1);
    	    return;
    	}


    	if (cmd_char == 'T')
    	{
    	    command_mode = 1;
    	    read_mode = 0; //sTM32 is not currently waiting for a file selection for reading.

    	    printf("\r\n--- COMMAND MODE ---\r\n");
    	    printf("1. List\r\n");
    	    printf("2. Read\r\n");
    	    printf("3. Delete\r\n");
    	    printf("4. Resume\r\n");
    	    printf("5. Download\r\n");
    	}
    	else if (command_mode)
    	{
    		if(read_mode || delete_mode || download_mode)
    		{
    		    if (cmd_char >= 'a' && cmd_char < ('a' + file_count))//character corresponds to one of the files currently listed
    		    {
    		        uint8_t index = cmd_char - 'a';//converts the user's letter into a numerical array index

    		        if(read_mode)
    		        {
    		            Read_File_By_Index(index);

    		            read_mode = 1;//waiting for another letter
    		        }
    		        else if(delete_mode)
    		        {
    		            delete_index = index;
    		            delete_request = 1;

    		            delete_mode = 1;
    		        }
    		        else if(download_mode)
    		        {
    		            Download_File_By_Index(index);

    		            download_mode = 1;
    		        }
    		    }
    		    else
    		    {
    		        printf("Invalid selection\r\n");
    		    }
    		}
    	    else
    	    {
    	        switch(cmd_char)//NOT waiting for a/b/c
    	        {
    	            case '1':
    	                list_request = 1;
    	                break;

    	            case '2':
    	                printf("\r\n--- SELECT FILE TO READ ---\r\n");
    	                List_Files();

    	                printf("Total files: %d\r\n", file_count);

    	                read_mode = 1;
    	                break;

    	            case '3':
    	                printf("\r\n--- SELECT FILE TO DELETE ---\r\n");
    	                List_Files();

    	                printf("Enter option (a/b/c...)\r\n");

    	                delete_mode = 1;
    	                break;


    	            case '4':
    	                printf("Resuming logging...\r\n");
    	                command_mode = 0;
    	                read_mode = 0;
    	                delete_mode = 0;
    	                download_mode = 0;
    	                last_output_time = HAL_GetTick();
    	                break;

    	            case '5':

    	                printf("\r\n--- SELECT FILE TO DOWNLOAD ---\r\n");

    	                List_Files();

    	                printf("Enter option (a/b/c...)\r\n");

    	                download_mode = 1;

    	                break;

    	            default:
    	                printf("Invalid option\r\n");
    	                break;
    	        }
    	    }
    	}

        HAL_UART_Receive_IT(&huart5, &cmd_char, 1);
    }
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



