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
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CMD_RDID  0x9F
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//#define LCD_BASE    ((uint32_t *) 0x60000000)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

OSPI_HandleTypeDef hospi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
uint16_t rowPins[] = {KBD_ROW_A_Pin, KBD_ROW_B_Pin, KBD_ROW_C_Pin, KBD_ROW_D_Pin, KBD_ROW_E_Pin, KBD_ROW_F_Pin, KBD_ROW_G_Pin, KBD_ROW_H_Pin, KBD_ROW_I_Pin};
GPIO_TypeDef *rowPorts[] = {KBD_ROW_A_GPIO_Port, KBD_ROW_B_GPIO_Port, KBD_ROW_C_GPIO_Port, KBD_ROW_D_GPIO_Port, KBD_ROW_E_GPIO_Port, KBD_ROW_F_GPIO_Port, KBD_ROW_G_GPIO_Port, KBD_ROW_H_GPIO_Port, KBD_ROW_I_GPIO_Port};

uint16_t colPins[] = {KBD_COL_1_Pin, KBD_COL_2_Pin, KBD_COL_3_Pin, KBD_COL_4_Pin, KBD_COL_5_Pin, KBD_COL_6_Pin};
GPIO_TypeDef *colPorts[] = {KBD_COL_1_GPIO_Port, KBD_COL_2_GPIO_Port, KBD_COL_3_GPIO_Port, KBD_COL_4_GPIO_Port, KBD_COL_5_GPIO_Port, KBD_COL_6_GPIO_Port};

Color color = {255, 0, 0};

const static int FSMCMemoryBank = 1;
const static int FSMCDataCommandAddressBit = 16;

const static uint32_t FSMCBaseAddress = 0x60000000;
const static uint32_t FSMCBankAddress = FSMCBaseAddress + (FSMCMemoryBank-1)*0x04000000;

static volatile uint16_t * const CommandAddress = (uint16_t *)(FSMCBankAddress);
static volatile uint16_t * const DataAddress = (uint16_t *)(FSMCBankAddress | (1<<(FSMCDataCommandAddressBit+1)));

void (*timeoutCallback)() = NULL;

typedef enum {
  Nop = 0x00,
  Reset = 0x01,
  ReadDisplayID = 0x04,
  SleepIn = 0x10,
  SleepOut = 0x11,
  DisplayInversionOff = 0x20,
  DisplayInversionOn = 0x21,
  DisplayOff = 0x28,
  DisplayOn = 0x29,
  ColumnAddressSet = 0x2A,
  PageAddressSet = 0x2B,
  MemoryWrite = 0x2C,
  MemoryRead = 0x2E,
  TearingEffectLineOn = 0x35,
  MemoryAccessControl = 0x36,
  PixelFormatSet = 0x3A,
  FrameRateControl = 0xC6,
  PositiveVoltageGammaControl = 0xE0,
  NegativeVoltageGammaControl = 0xE1
} Command;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_FMC_Init(void);
static void MX_TIM2_Init(void);
static void MX_OCTOSPI1_Init(void);
/* USER CODE BEGIN PFP */
void UART_SendChar(char c);
void UART_SendString(char *str);
void setLedColor(Color color);
int keydown(KeyboardPos pos);
void sendMessage(char *message);
void LCD_WriteCommand(Command cmd);
void LCD_WriteData(uint8_t data);
uint32_t panelIdentifier();
void LCD_Init(void);
void LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void LCD_SetBrightness(int brightness);
uint16_t RGB565(uint8_t r, uint8_t g, uint8_t b);
uint8_t ReadFlashID(void);
void OctoSPI_SendCommand(uint8_t instruction, uint8_t *data, uint16_t dataSize);
void EnableQuadSPI();

char *uint32_to_str(uint32_t value) {
    char *buffer = (char *)malloc(11 * sizeof(char));
    if (buffer == NULL) return NULL;

    snprintf(buffer, 11, "%lu", value);
    return buffer;
}

char *concat_strings(int num, ...) {
    if (num <= 0) return NULL;

    va_list args;
    va_start(args, num);

    // Calculer la taille totale nécessaire
    size_t total_length = 0;
    for (int i = 0; i < num; i++) {
        const char *str = va_arg(args, const char*);
        if (!str) str = ""; // Gérer les NULL comme des chaînes vides
        total_length += strlen(str);
    }

    va_end(args);

    // Allouer la mémoire pour la chaîne finale
    char *result = (char *)malloc(total_length + 1); // +1 pour '\0'
    if (!result) return NULL;

    result[0] = '\0'; // Initialiser à une chaîne vide

    // Concaténer toutes les chaînes
    va_start(args, num);
    for (int i = 0; i < num; i++) {
        const char *str = va_arg(args, const char*);
        if (!str) str = "";
        strcat(result, str);
    }
    va_end(args);

    return result; // L'utilisateur devra libérer la mémoire
}
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  MX_FMC_Init();
  MX_TIM2_Init();
  MX_OCTOSPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  /*setLedColor((Color){255, 255, 0});
  HAL_Delay(250);
  setLedColor((Color){0, 0, 0});
  HAL_Delay(250);
  setLedColor((Color){255, 255, 0});
  HAL_Delay(250);
  setLedColor((Color){0, 0, 0});
  HAL_Delay(250);

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, 1);
  HAL_Delay(10000);
  sendMessage(uint32_to_str_dynamic(panelIdentifier()));*/
  EnableQuadSPI();

  setLedColor((Color){0, 255, 0});
  HAL_Delay(250);
  setLedColor((Color){0, 0, 0});
  HAL_Delay(250);
  setLedColor((Color){0, 255, 0});
  HAL_Delay(250);
  setLedColor((Color){0, 0, 0});
  HAL_Delay(250);

  //Color color = {255, 0, 0};
  //setLedColor(color);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //LCD_Init();
  //LCD_SetBrightness(1);
  //int b = 0;
  Color color = {255, 0, 0};
  setLedColor(color);
  while (1)
  {
	HAL_Delay(1000);
	sendMessage(uint32_to_str(ReadFlashID()));
	//if(keydown(KEY_SHIFT)) LCD_DrawPixel(10, 10, RGB565(255, 0, 0));
	//if(keydown(KEY_ALPHA)) LCD_DrawPixel(10, 10, RGB565(0, 255, 0));
	//HAL_Delay(100);
	//LCD_WriteCommand(MemoryWrite);
	//LCD_WriteData(RGB565(255, 0, 0));
	//LCD_DrawPixel(10, 10, RGB565(255, 0, 0));
	//LCD_SetBrightness(b + 1);
	//HAL_Delay(500);
	//b++;
	//if(b >= 13) b=13;
	/*if(keydown((KeyboardPos){8, 0})) {

	}*/
	/*if(keydown(KEY_LEFT)) sendMessage("LEFT\r\n");
	if(keydown(KEY_UP)) sendMessage("UP\r\n");
	if(keydown(KEY_DOWN)) sendMessage("DOWN\r\n");
	if(keydown(KEY_RIGHT)) sendMessage("RIGHT\r\n");
	if(keydown(KEY_OK)) sendMessage("OK\r\n");
	if(keydown(KEY_BACK)) sendMessage("BACK\r\n");
	if(keydown(KEY_HOME)) sendMessage("HOME\r\n");
	if(keydown(KEY_ONOFF)) sendMessage("ONOFF\r\n");
	if(keydown(KEY_SHIFT)) sendMessage("SHIFT\r\n");
	if(keydown(KEY_ALPHA)) sendMessage("ALPHA\r\n");
	if(keydown(KEY_XNT)) sendMessage("XNT\r\n");
	if(keydown(KEY_VAR)) sendMessage("VAR\r\n");
	if(keydown(KEY_TOOLBOX)) sendMessage("TOOLBOX\r\n");
	if(keydown(KEY_BACKSPACE)) sendMessage("BACKSPACE\r\n");
	if(keydown(KEY_EXP)) sendMessage("EXP\r\n");
	if(keydown(KEY_LN)) sendMessage("LN\r\n");
	if(keydown(KEY_LOG)) sendMessage("LOG\r\n");
	if(keydown(KEY_IMAGINARY)) sendMessage("IMAGINARY\r\n");
	if(keydown(KEY_COMMA)) sendMessage("COMMA\r\n");
	if(keydown(KEY_POWER)) sendMessage("POWER\r\n");
	if(keydown(KEY_SINE)) sendMessage("SINE\r\n");
	if(keydown(KEY_COSINE)) sendMessage("COSINE\r\n");
	if(keydown(KEY_TANGENT)) sendMessage("TANGENT\r\n");
	if(keydown(KEY_PI)) sendMessage("PI\r\n");
	if(keydown(KEY_SQRT)) sendMessage("SQRT\r\n");
	if(keydown(KEY_SQUARE)) sendMessage("SQUARE\r\n");
	if(keydown(KEY_SEVEN)) sendMessage("SEVEN\r\n");
	if(keydown(KEY_EIGHT)) sendMessage("EIGHT\r\n");
	if(keydown(KEY_NINE)) sendMessage("NINE\r\n");
	if(keydown(KEY_LEFTPARENTHESIS)) sendMessage("LEFTPARENTHESIS\r\n");
	if(keydown(KEY_RIGHTPARENTHESIS)) sendMessage("RIGHTPARENTHESIS\r\n");
	if(keydown(KEY_FOUR)) sendMessage("FOUR\r\n");
	if(keydown(KEY_FIVE)) sendMessage("FIVE\r\n");
	if(keydown(KEY_SIX)) sendMessage("SIX\r\n");
	if(keydown(KEY_MULTIPLICATION)) sendMessage("MULTIPLICATION\r\n");
	if(keydown(KEY_DIVISION)) sendMessage("DIVISION\r\n");
	if(keydown(KEY_ONE)) sendMessage("ONE\r\n");
	if(keydown(KEY_TWO)) sendMessage("TWO\r\n");
	if(keydown(KEY_THREE)) sendMessage("THREE\r\n");
	if(keydown(KEY_PLUS)) sendMessage("PLUS\r\n");
	if(keydown(KEY_MINUS)) sendMessage("MINUS\r\n");
	if(keydown(KEY_ZERO)) sendMessage("ZERO\r\n");
	if(keydown(KEY_DOT)) sendMessage("DOT\r\n");
	if(keydown(KEY_EE)) sendMessage("EE\r\n");
	if(keydown(KEY_ANS)) sendMessage("ANS\r\n");
	if(keydown(KEY_EXE)) sendMessage("EXE\r\n");*/
	//if(keydown(KEY_ONE)) sendMessage("ONE\r\n");
	//HAL_Delay(250);
	/*for (int i=0; i<255; i++) {
		color.Green += 1;
		setLedColor(color);
		HAL_Delay(2);
	}
	for (int i=0; i<255; i++) {
		color.Red -= 1;
		setLedColor(color);
		HAL_Delay(2);
	}
	for (int i=0; i<255; i++) {
		color.Blue += 1;
		setLedColor(color);
		HAL_Delay(2);
	}
	for (int i=0; i<255; i++) {
		color.Green -= 1;
		setLedColor(color);
		HAL_Delay(2);
	}
	for (int i=0; i<255; i++) {
		color.Red += 1;
		setLedColor(color);
		HAL_Delay(2);
	}
	for (int i=0; i<255; i++) {
		color.Blue -= 1;
		setLedColor(color);
		HAL_Delay(2);
	}*/
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
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
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the SYSCFG APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
  */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_LSE;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,32768);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
}

/**
  * @brief OCTOSPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI1_Init(void)
{

  /* USER CODE BEGIN OCTOSPI1_Init 0 */

  /* USER CODE END OCTOSPI1_Init 0 */

  OSPIM_CfgTypeDef sOspiManagerCfg = {0};

  /* USER CODE BEGIN OCTOSPI1_Init 1 */

  /* USER CODE END OCTOSPI1_Init 1 */
  /* OCTOSPI1 parameter configuration*/
  hospi1.Instance = OCTOSPI1;
  hospi1.Init.FifoThreshold = 4;
  hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_MACRONIX_RAM;
  hospi1.Init.DeviceSize = 23;
  hospi1.Init.ChipSelectHighTime = 2;
  hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi1.Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
  hospi1.Init.ClockPrescaler = 4;
  hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
  hospi1.Init.ChipSelectBoundary = 0;
  hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
  hospi1.Init.MaxTran = 0;
  hospi1.Init.Refresh = 0;
  if (HAL_OSPI_Init(&hospi1) != HAL_OK)
  {
    Error_Handler();
  }
  sOspiManagerCfg.ClkPort = 1;
  sOspiManagerCfg.NCSPort = 1;
  sOspiManagerCfg.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
  if (HAL_OSPIM_Config(&hospi1, &sOspiManagerCfg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI1_Init 2 */

  /* USER CODE END OCTOSPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 9999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FMC_NORSRAM_DEVICE;
  hsram1.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;
  hsram1.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  hsram1.Init.WriteFifo = FMC_WRITE_FIFO_ENABLE;
  hsram1.Init.PageSize = FMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, KBD_COL_1_Pin|KBD_COL_4_Pin|KBD_COL_5_Pin|KBD_COL_6_Pin
                          |KBD_COL_2_Pin|KBD_COL_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BACKLIGHT_GPIO_Port, BACKLIGHT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LCD_RESET_Pin */
  GPIO_InitStruct.Pin = LCD_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KBD_COL_1_Pin KBD_COL_4_Pin KBD_COL_5_Pin */
  GPIO_InitStruct.Pin = KBD_COL_1_Pin|KBD_COL_4_Pin|KBD_COL_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : KBD_ROW_B_Pin KBD_ROW_A_Pin KBD_ROW_C_Pin KBD_ROW_D_Pin
                           KBD_ROW_E_Pin KBD_ROW_F_Pin KBD_ROW_G_Pin KBD_ROW_H_Pin
                           KBD_ROW_I_Pin */
  GPIO_InitStruct.Pin = KBD_ROW_B_Pin|KBD_ROW_A_Pin|KBD_ROW_C_Pin|KBD_ROW_D_Pin
                          |KBD_ROW_E_Pin|KBD_ROW_F_Pin|KBD_ROW_G_Pin|KBD_ROW_H_Pin
                          |KBD_ROW_I_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : KBD_COL_6_Pin KBD_COL_2_Pin KBD_COL_3_Pin */
  GPIO_InitStruct.Pin = KBD_COL_6_Pin|KBD_COL_2_Pin|KBD_COL_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BACKLIGHT_Pin */
  GPIO_InitStruct.Pin = BACKLIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BACKLIGHT_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void setLedColor(Color c) {
  color = c;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, color.Red*1000/255);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, color.Green*1000/255);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, color.Blue*1000/255);
}

int keydown(KeyboardPos pos) {
	uint16_t rowPin = rowPins[pos.row];
	GPIO_TypeDef* rowPort = rowPorts[pos.row];
	uint16_t colPin = colPins[pos.col];
	GPIO_TypeDef* colPort = colPorts[pos.col];
	HAL_GPIO_WritePin(colPort, colPin, GPIO_PIN_SET);
	GPIO_PinState down = HAL_GPIO_ReadPin(rowPort, rowPin);
	HAL_GPIO_WritePin(colPort, colPin, GPIO_PIN_RESET);
	return down;
}

uint32_t panelIdentifier() {
  LCD_WriteCommand(ReadDisplayID);
  *DataAddress; // Dummy read, per datasheet
  uint8_t id1 = *DataAddress;
  uint8_t id2 = *DataAddress;
  uint8_t id3 = *DataAddress;

  return (id1 << 16) | (id2 << 8) | id3;
}

void setTimeout(void (*callback)(), uint32_t delay_ms) {
    timeoutCallback = callback;  // Stocke la fonction callback
    __HAL_TIM_SET_AUTORELOAD(&htim2, delay_ms - 1);  // Définit la durée
    __HAL_TIM_SET_COUNTER(&htim2, 0);  // Réinitialise le compteur
    HAL_TIM_Base_Start_IT(&htim2);  // Démarre le Timer avec interruption
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim2) {  // Vérifie si c'est bien TIM2
        HAL_TIM_Base_Stop_IT(&htim2);  // Stoppe le Timer
        if (timeoutCallback) timeoutCallback();  // Exécute la fonction
    }
}

void resetLed() {
  setLedColor((Color){0, 0, 0});
}

void sendMessage(char *message) {
  uint16_t len = strlen(message);
  CDC_Transmit_HS((uint8_t *)message, len);
  timeoutCallback = NULL;
  setLedColor((Color){255, 255, 0});
  setTimeout(resetLed, 300);
}

void LCD_WriteCommand(Command cmd)
{
  //GPIOB->ODR &= ~GPIO_PIN_11;  // RS = 0 pour commande
  *(volatile uint16_t *)(CommandAddress) = cmd;
}

void LCD_WriteData(uint8_t data)
{
    //GPIOB->ODR |= GPIO_PIN_11;   // RS = 1 pour données
    *(volatile uint16_t *)(DataAddress) = data;
}

void LCD_Init(void)
{
    // Reset de l'écran
    /*HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);  // RESET = 0
    HAL_Delay(50);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);    // RESET = 1
    HAL_Delay(50);

    // Commande de sortie du mode Sleep
    LCD_WriteCommand(0x11);  // Sleep out
    HAL_Delay(120);

    // Configuration de la fenêtre d'affichage
    LCD_WriteCommand(0x36);  // Memory Access Control
    LCD_WriteData(0x00);     // Orientation (de gauche à droite, de haut en bas)

    LCD_WriteCommand(0x3A);  // Pixel format set
    LCD_WriteData(0x05);     // 16 bits par pixel

    // Configuration du contrôle de contraste (gamma)
    LCD_WriteCommand(0xC0);  // Power control 1
    LCD_WriteData(0x10);     // Valeur spécifique pour le ST7789V
    LCD_WriteData(0x3B);     // Valeur spécifique pour le ST7789V

    LCD_WriteCommand(0x29);  // Display ON
    HAL_Delay(100);*/
}

void LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
	sendMessage(concat_strings(7,"SET_PIXEL(",uint32_to_str(x),",",uint32_to_str(y),",",uint32_to_str(color),")"));
    /*LCD_WriteCommand(0x2A); // Commande pour définir les colonnes
    LCD_WriteData(x >> 8);  // Adresse haute de x
    LCD_WriteData(x & 0xFF); // Adresse basse de x

    LCD_WriteCommand(0x2B); // Commande pour définir les lignes
    LCD_WriteData(y >> 8);  // Adresse haute de y
    LCD_WriteData(y & 0xFF); // Adresse basse de y

    LCD_WriteCommand(0x2C); // Commande pour écrire les données du pixel
    LCD_WriteData(color >> 8); // Couleur (partie haute)
    LCD_WriteData(color & 0xFF); // Couleur (partie basse)*/
}

void LCD_FillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color) {
	sendMessage(concat_strings(11,"FILL_RECT(",uint32_to_str(x),",",uint32_to_str(y),",",uint32_to_str(width),",",uint32_to_str(height),",",uint32_to_str(color),")"));
}

void LCD_DrawString(uint16_t x, uint16_t y, char* text, uint16_t color1, uint16_t color2) {
	sendMessage(concat_strings(11,"DRAW_STRING(",uint32_to_str(x),",",uint32_to_str(y),",",text,",",uint32_to_str(color1),",",uint32_to_str(color2),")"));
}

uint16_t RGB565(uint8_t r, uint8_t g, uint8_t b)
{
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

void LCD_SetBrightness(int brightness) {
  for (int i=0; i<brightness; i++) {
	//HAL_GPIO_WritePin(BACKLIGHT_GPIO_Port, BACKLIGHT_Pin, 0);
	HAL_Delay(0.01);
	//HAL_GPIO_WritePin(BACKLIGHT_GPIO_Port, BACKLIGHT_Pin, 1);
	HAL_Delay(0.01);
  }
}

void OctoSPI_SendCommand(uint8_t instruction, uint8_t *data, uint16_t dataSize) {
    OSPI_RegularCmdTypeDef sCommand = {0};

    sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
    sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
    sCommand.Instruction        = instruction;
    sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE; // 1 ligne pour l'instruction
    sCommand.AddressMode        = HAL_OSPI_ADDRESS_NONE;       // Pas d'adresse pour lire l'ID
    sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode           = HAL_OSPI_DATA_1_LINE;        // 1 ligne pour les données
    sCommand.NbData             = dataSize;                    // Nombre de bytes à recevoir
    sCommand.DummyCycles        = 0;
    sCommand.DQSMode            = HAL_OSPI_DQS_DISABLE;
    sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

    if (HAL_OSPI_Command(&hospi1, &sCommand, HAL_MAX_DELAY) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_OSPI_Receive(&hospi1, data, HAL_MAX_DELAY) != HAL_OK) {
    	Error_Handler();
    }
}

uint8_t ReadFlashID(void) {
	uint8_t id[3] = {0};
	OctoSPI_SendCommand(CMD_RDID, id, 3); // 0x9F = Commande Read ID
	return id[0];
}

void EnableQuadSPI()
{
    OSPI_RegularCmdTypeDef sCommand;
    uint8_t reg;

    // Lire le registre de statut
    memset(&sCommand, 0, sizeof(sCommand));
    sCommand.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
    sCommand.FlashId = HAL_OSPI_FLASH_ID_1;
    sCommand.Instruction = 0x05; // Read Status Register
    sCommand.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
    sCommand.DataMode = HAL_OSPI_DATA_1_LINE;
    sCommand.NbData = 1;

    HAL_OSPI_Command(&hospi1, &sCommand, HAL_MAX_DELAY);
    HAL_OSPI_Receive(&hospi1, &reg, HAL_MAX_DELAY);

    // Activer Quad Enable Bit (QE = bit 6)
    reg |= (1 << 6);

    // Écrire le registre de statut
    sCommand.Instruction = 0x01; // Write Status Register
    HAL_OSPI_Command(&hospi1, &sCommand, HAL_MAX_DELAY);
    HAL_OSPI_Transmit(&hospi1, &reg, HAL_MAX_DELAY);

    sendMessage("Quad SPI activé !\n");
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
  //Color color = {255, 0, 0};
  //setLedColor(color);
  while(1) {
	for (int i=0; i<255; i++) {
	  color.Red -= 1;
	  setLedColor(color);
	  HAL_Delay(2);
	}
	for (int i=0; i<255; i++) {
	  color.Red += 1;
	  setLedColor(color);
	  HAL_Delay(2);
	}
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
