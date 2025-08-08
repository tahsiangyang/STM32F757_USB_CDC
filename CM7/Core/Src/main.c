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
#include "usbd_cdc.h"
#include "arm_math.h"
#include "arm_const_structs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEST_LENGTH_SAMPLES 2048

#define PI              3.14159265358979f
#define SAMPLE_RATE     2048      // 取樣頻率 1kHz
#define FREQ            50.0f     // 正弦波頻率 50Hz
/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;

DSI_HandleTypeDef hdsi;

LTDC_HandleTypeDef hltdc;

SAI_HandleTypeDef hsai_BlockB1;
SAI_HandleTypeDef hsai_BlockA1;

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;
extern float32_t testInput_f32_10khz[TEST_LENGTH_SAMPLES];
static float32_t testOutput[TEST_LENGTH_SAMPLES / 2];
static float32_t testInput_sine[SAMPLE_RATE];
uint32_t fftSize = 1024;
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;
uint32_t refIndex = 213, testIndex = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DSIHOST_DSI_Init(void);
static void MX_LTDC_Init(void);
static void MX_SAI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */
	uint8_t UserTxBuffer[] =
			"WaveShare Open7XXI-C Board STM32 Virtual COM Port Driver \r\n";
	arm_status status;
	float32_t maxValue;
	uint8_t raw_bytes[4];

	status = ARM_MATH_SUCCESS;
	/* USER CODE END 1 */
	/* USER CODE BEGIN Boot_Mode_Sequence_0 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
	int32_t timeout;
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
	/* USER CODE END Boot_Mode_Sequence_0 */

	/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
	/* Wait until CPU2 boots and enters in stop mode or timeout*/
	timeout = 0xFFFF;
	//while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
	//if ( timeout < 0 )
	//{
	//Error_Handler();
	//}
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
	/* USER CODE END Boot_Mode_Sequence_1 */
	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();
	/* USER CODE BEGIN Boot_Mode_Sequence_2 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
	/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
	 HSEM notification */
	/*HW semaphore Clock enable*/
	__HAL_RCC_HSEM_CLK_ENABLE();
	/*Take HSEM */
	HAL_HSEM_FastTake(HSEM_ID_0);
	/*Release HSEM in order to notify the CPU2(CM4)*/
	HAL_HSEM_Release(HSEM_ID_0, 0);
	/* wait until CPU2 wakes up from stop mode */
	timeout = 0xFFFF;
	//while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
	//if ( timeout < 0 )
	//{
	//Error_Handler();
	//}
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
	/* USER CODE END Boot_Mode_Sequence_2 */

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	//MX_GPIO_Init();
	//MX_DSIHOST_DSI_Init();
	//MX_LTDC_Init();
	//MX_SAI1_Init();
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		HAL_Delay(1000);

		for (int i = 0; i < SAMPLE_RATE; i++) {
			float32_t angle = 2.0f * PI * FREQ
					* ((float32_t) i / (float32_t) SAMPLE_RATE);
			float32_t radians = (PI / 180.0f) * angle;
			testInput_sine[i] = arm_sin_f32(radians);
		}

		for (int i = 0; i < SAMPLE_RATE; i++) {
			memcpy(raw_bytes, &testInput_sine[i], sizeof(float32_t));
			USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t*) &raw_bytes,
					sizeof(raw_bytes));
			USBD_CDC_TransmitPacket(&hUsbDeviceFS);
			HAL_Delay(100);
		}

		HAL_Delay(1000);

		for (int i = 0; i < TEST_LENGTH_SAMPLES / 2; i++) {
			testOutput[i] = 0.0f;
		}

		/* Process the data through the CFFT/CIFFT module */
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, testInput_sine, ifftFlag,
				doBitReverse);
		/* Process the data through the Complex Magnitude Module for
		 calculating the magnitude at each bin */
		arm_cmplx_mag_f32(testInput_sine, testOutput, fftSize);
		/* Calculates maxValue and returns corresponding BIN value */
		arm_max_f32(testOutput, fftSize, &maxValue, &testIndex);

		//USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t*)&UserTxBuffer, sizeof(UserTxBuffer));
		//USBD_CDC_TransmitPacket(&hUsbDeviceFS);
		for (int i = 0; i < SAMPLE_RATE/2; i++) {
			memcpy(raw_bytes, &testOutput[i], sizeof(float32_t));
			USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t*) &raw_bytes,
					sizeof(raw_bytes));
			USBD_CDC_TransmitPacket(&hUsbDeviceFS);
			HAL_Delay(100);
		}

		while (1)
			;

		for (int i = 0; i < TEST_LENGTH_SAMPLES / 2; i++) {
			testOutput[i] = 0.0f;
		}

		/* Process the data through the CFFT/CIFFT module */
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, testInput_f32_10khz, ifftFlag,
				doBitReverse);
		/* Process the data through the Complex Magnitude Module for
		 calculating the magnitude at each bin */
		arm_cmplx_mag_f32(testInput_f32_10khz, testOutput, fftSize);
		/* Calculates maxValue and returns corresponding BIN value */
		arm_max_f32(testOutput, fftSize, &maxValue, &testIndex);

		//USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t*)&UserTxBuffer, sizeof(UserTxBuffer));
		//USBD_CDC_TransmitPacket(&hUsbDeviceFS);
		for (int i = 0; i < TEST_LENGTH_SAMPLES / 2; i++) {
			memcpy(raw_bytes, &testOutput[i], sizeof(float32_t));
			USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t*) &raw_bytes,
					sizeof(raw_bytes));
			USBD_CDC_TransmitPacket(&hUsbDeviceFS);
			HAL_Delay(100);
		}

		while (1)
			;

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48
			| RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 2;
	RCC_OscInitStruct.PLL.PLLN = 12;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLQ = 1;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1
			| RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
	HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/**
 * @brief DFSDM1 Initialization Function
 * @param None
 * @retval None
 */
void MX_DFSDM1_Init(void) {

	/* USER CODE BEGIN DFSDM1_Init 0 */

	/* USER CODE END DFSDM1_Init 0 */

	/* USER CODE BEGIN DFSDM1_Init 1 */

	/* USER CODE END DFSDM1_Init 1 */
	hdfsdm1_channel1.Instance = DFSDM1_Channel1;
	hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
	hdfsdm1_channel1.Init.OutputClock.Selection =
			DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
	hdfsdm1_channel1.Init.OutputClock.Divider = 2;
	hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
	hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
	hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
	hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
	hdfsdm1_channel1.Init.SerialInterface.SpiClock =
			DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
	hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
	hdfsdm1_channel1.Init.Awd.Oversampling = 1;
	hdfsdm1_channel1.Init.Offset = 0;
	hdfsdm1_channel1.Init.RightBitShift = 0x00;
	if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN DFSDM1_Init 2 */

	/* USER CODE END DFSDM1_Init 2 */

}

/**
 * @brief DSIHOST Initialization Function
 * @param None
 * @retval None
 */
static void MX_DSIHOST_DSI_Init(void) {

	/* USER CODE BEGIN DSIHOST_Init 0 */

	/* USER CODE END DSIHOST_Init 0 */

	DSI_PLLInitTypeDef PLLInit = { 0 };
	DSI_HOST_TimeoutTypeDef HostTimeouts = { 0 };
	DSI_PHY_TimerTypeDef PhyTimings = { 0 };
	DSI_LPCmdTypeDef LPCmd = { 0 };
	DSI_CmdCfgTypeDef CmdCfg = { 0 };

	/* USER CODE BEGIN DSIHOST_Init 1 */

	/* USER CODE END DSIHOST_Init 1 */
	hdsi.Instance = DSI;
	hdsi.Init.AutomaticClockLaneControl = DSI_AUTO_CLK_LANE_CTRL_DISABLE;
	hdsi.Init.TXEscapeCkdiv = 2;
	hdsi.Init.NumberOfLanes = DSI_ONE_DATA_LANE;
	PLLInit.PLLNDIV = 20;
	PLLInit.PLLIDF = DSI_PLL_IN_DIV1;
	PLLInit.PLLODF = DSI_PLL_OUT_DIV2;
	if (HAL_DSI_Init(&hdsi, &PLLInit) != HAL_OK) {
		Error_Handler();
	}
	HostTimeouts.TimeoutCkdiv = 1;
	HostTimeouts.HighSpeedTransmissionTimeout = 0;
	HostTimeouts.LowPowerReceptionTimeout = 0;
	HostTimeouts.HighSpeedReadTimeout = 0;
	HostTimeouts.LowPowerReadTimeout = 0;
	HostTimeouts.HighSpeedWriteTimeout = 0;
	HostTimeouts.HighSpeedWritePrespMode = DSI_HS_PM_DISABLE;
	HostTimeouts.LowPowerWriteTimeout = 0;
	HostTimeouts.BTATimeout = 0;
	if (HAL_DSI_ConfigHostTimeouts(&hdsi, &HostTimeouts) != HAL_OK) {
		Error_Handler();
	}
	PhyTimings.ClockLaneHS2LPTime = 20;
	PhyTimings.ClockLaneLP2HSTime = 18;
	PhyTimings.DataLaneHS2LPTime = 10;
	PhyTimings.DataLaneLP2HSTime = 13;
	PhyTimings.DataLaneMaxReadTime = 0;
	PhyTimings.StopWaitTime = 0;
	if (HAL_DSI_ConfigPhyTimer(&hdsi, &PhyTimings) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_DSI_ConfigFlowControl(&hdsi, DSI_FLOW_CONTROL_BTA) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_DSI_SetLowPowerRXFilter(&hdsi, 10000) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_DSI_ConfigErrorMonitor(&hdsi, HAL_DSI_ERROR_NONE) != HAL_OK) {
		Error_Handler();
	}
	LPCmd.LPGenShortWriteNoP = DSI_LP_GSW0P_DISABLE;
	LPCmd.LPGenShortWriteOneP = DSI_LP_GSW1P_DISABLE;
	LPCmd.LPGenShortWriteTwoP = DSI_LP_GSW2P_DISABLE;
	LPCmd.LPGenShortReadNoP = DSI_LP_GSR0P_DISABLE;
	LPCmd.LPGenShortReadOneP = DSI_LP_GSR1P_DISABLE;
	LPCmd.LPGenShortReadTwoP = DSI_LP_GSR2P_DISABLE;
	LPCmd.LPGenLongWrite = DSI_LP_GLW_DISABLE;
	LPCmd.LPDcsShortWriteNoP = DSI_LP_DSW0P_DISABLE;
	LPCmd.LPDcsShortWriteOneP = DSI_LP_DSW1P_DISABLE;
	LPCmd.LPDcsShortReadNoP = DSI_LP_DSR0P_DISABLE;
	LPCmd.LPDcsLongWrite = DSI_LP_DLW_DISABLE;
	LPCmd.LPMaxReadPacket = DSI_LP_MRDP_DISABLE;
	LPCmd.AcknowledgeRequest = DSI_ACKNOWLEDGE_DISABLE;
	if (HAL_DSI_ConfigCommand(&hdsi, &LPCmd) != HAL_OK) {
		Error_Handler();
	}
	CmdCfg.VirtualChannelID = 0;
	CmdCfg.ColorCoding = DSI_RGB888;
	CmdCfg.CommandSize = 640;
	CmdCfg.TearingEffectSource = DSI_TE_EXTERNAL;
	CmdCfg.TearingEffectPolarity = DSI_TE_RISING_EDGE;
	CmdCfg.HSPolarity = DSI_HSYNC_ACTIVE_LOW;
	CmdCfg.VSPolarity = DSI_VSYNC_ACTIVE_LOW;
	CmdCfg.DEPolarity = DSI_DATA_ENABLE_ACTIVE_HIGH;
	CmdCfg.VSyncPol = DSI_VSYNC_FALLING;
	CmdCfg.AutomaticRefresh = DSI_AR_ENABLE;
	CmdCfg.TEAcknowledgeRequest = DSI_TE_ACKNOWLEDGE_DISABLE;
	if (HAL_DSI_ConfigAdaptedCommandMode(&hdsi, &CmdCfg) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_DSI_SetGenericVCID(&hdsi, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN DSIHOST_Init 2 */

	/* USER CODE END DSIHOST_Init 2 */

}

/**
 * @brief LTDC Initialization Function
 * @param None
 * @retval None
 */
static void MX_LTDC_Init(void) {

	/* USER CODE BEGIN LTDC_Init 0 */

	/* USER CODE END LTDC_Init 0 */

	LTDC_LayerCfgTypeDef pLayerCfg = { 0 };
	LTDC_LayerCfgTypeDef pLayerCfg1 = { 0 };

	/* USER CODE BEGIN LTDC_Init 1 */

	/* USER CODE END LTDC_Init 1 */
	hltdc.Instance = LTDC;
	hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
	hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
	hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
	hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
	hltdc.Init.HorizontalSync = 7;
	hltdc.Init.VerticalSync = 3;
	hltdc.Init.AccumulatedHBP = 14;
	hltdc.Init.AccumulatedVBP = 5;
	hltdc.Init.AccumulatedActiveW = 654;
	hltdc.Init.AccumulatedActiveH = 485;
	hltdc.Init.TotalWidth = 660;
	hltdc.Init.TotalHeigh = 487;
	hltdc.Init.Backcolor.Blue = 0;
	hltdc.Init.Backcolor.Green = 0;
	hltdc.Init.Backcolor.Red = 0;
	if (HAL_LTDC_Init(&hltdc) != HAL_OK) {
		Error_Handler();
	}
	pLayerCfg.WindowX0 = 0;
	pLayerCfg.WindowX1 = 0;
	pLayerCfg.WindowY0 = 0;
	pLayerCfg.WindowY1 = 0;
	pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
	pLayerCfg.Alpha = 0;
	pLayerCfg.Alpha0 = 0;
	pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
	pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
	pLayerCfg.FBStartAdress = 0;
	pLayerCfg.ImageWidth = 0;
	pLayerCfg.ImageHeight = 0;
	pLayerCfg.Backcolor.Blue = 0;
	pLayerCfg.Backcolor.Green = 0;
	pLayerCfg.Backcolor.Red = 0;
	if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK) {
		Error_Handler();
	}
	pLayerCfg1.WindowX0 = 0;
	pLayerCfg1.WindowX1 = 0;
	pLayerCfg1.WindowY0 = 0;
	pLayerCfg1.WindowY1 = 0;
	pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
	pLayerCfg1.Alpha = 0;
	pLayerCfg1.Alpha0 = 0;
	pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
	pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
	pLayerCfg1.FBStartAdress = 0;
	pLayerCfg1.ImageWidth = 0;
	pLayerCfg1.ImageHeight = 0;
	pLayerCfg1.Backcolor.Blue = 0;
	pLayerCfg1.Backcolor.Green = 0;
	pLayerCfg1.Backcolor.Red = 0;
	if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN LTDC_Init 2 */

	/* USER CODE END LTDC_Init 2 */

}

/**
 * @brief SAI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SAI1_Init(void) {

	/* USER CODE BEGIN SAI1_Init 0 */

	/* USER CODE END SAI1_Init 0 */

	/* USER CODE BEGIN SAI1_Init 1 */

	/* USER CODE END SAI1_Init 1 */
	hsai_BlockB1.Instance = SAI1_Block_B;
	hsai_BlockB1.Init.Protocol = SAI_FREE_PROTOCOL;
	hsai_BlockB1.Init.AudioMode = SAI_MODESLAVE_RX;
	hsai_BlockB1.Init.DataSize = SAI_DATASIZE_8;
	hsai_BlockB1.Init.FirstBit = SAI_FIRSTBIT_MSB;
	hsai_BlockB1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
	hsai_BlockB1.Init.Synchro = SAI_SYNCHRONOUS;
	hsai_BlockB1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
	hsai_BlockB1.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
	hsai_BlockB1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
	hsai_BlockB1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
	hsai_BlockB1.Init.MonoStereoMode = SAI_STEREOMODE;
	hsai_BlockB1.Init.CompandingMode = SAI_NOCOMPANDING;
	hsai_BlockB1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
	hsai_BlockB1.Init.PdmInit.Activation = DISABLE;
	hsai_BlockB1.Init.PdmInit.MicPairsNbr = 1;
	hsai_BlockB1.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;
	hsai_BlockB1.FrameInit.FrameLength = 8;
	hsai_BlockB1.FrameInit.ActiveFrameLength = 1;
	hsai_BlockB1.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
	hsai_BlockB1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
	hsai_BlockB1.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
	hsai_BlockB1.SlotInit.FirstBitOffset = 0;
	hsai_BlockB1.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
	hsai_BlockB1.SlotInit.SlotNumber = 1;
	hsai_BlockB1.SlotInit.SlotActive = 0x00000000;
	if (HAL_SAI_Init(&hsai_BlockB1) != HAL_OK) {
		Error_Handler();
	}
	hsai_BlockA1.Instance = SAI1_Block_A;
	hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_TX;
	hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
	hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
	hsai_BlockA1.Init.NoDivider = SAI_MCK_OVERSAMPLING_DISABLE;
	hsai_BlockA1.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
	hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
	hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_192K;
	hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
	hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
	hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
	hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
	if (HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD,
			SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SAI1_Init 2 */

	/* USER CODE END SAI1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOI_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();

	/*Configure GPIO pins : PB5 PB10 PB11 PB1
	 PB12 PB0 PB13 */
	GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_1
			| GPIO_PIN_12 | GPIO_PIN_0 | GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG2_HS;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PG11 PG12 PG13 */
	GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pin : PE2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF10_SAI4;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : MCO1_Pin */
	GPIO_InitStruct.Pin = MCO1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
	HAL_GPIO_Init(MCO1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PI11 */
	GPIO_InitStruct.Pin = GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG2_HS;
	HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

	/*Configure GPIO pin : PC0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG2_HS;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PC1 PC4 PC5 */
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA2 PA1 PA7 */
	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PH4 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG2_HS;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	/*Configure GPIO pins : PA5 PA3 */
	GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG2_HS;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	//while (1)
	//{
	//}
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
