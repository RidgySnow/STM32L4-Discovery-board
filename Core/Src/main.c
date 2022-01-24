/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * This program sets up codec on STM32L4 - Discovery board in beep mode.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Define all codac register addresses
#define CODAC_ADRESS_WRITE 0x94
#define CODAC_ADRESS_READ 0x95
#define TIMEOUT 10
#define POWER_UP_REG 0x02
#define BEEP_REG 0x1E
#define BEEP_FREQ_ONTIME_REG 0x1C
#define BEEP_VOLUME_OFFTIME_REG 0x1D
#define MASTER_VOLUME_REG 0x20
#define HPA_VOLUME_REG 0x22
#define HPB_VOLUME_REG 0x23
#define CLOCK_CONTROL_REG 0x05
#define POWER_CR2_REG 0x04
#define PLAYBACKC_2_REG 0x0F // playback control 2
#define PASSAVOL_REG 0x14 // Passthrough A Volume
#define PASSBVOL_REG 0x15 // Passthrough B Volume
#define MISCELLANEOUS_CONTROLS_REG 0x0E //MISCELLANEOUS CONTROLS register
#define INTERFACE_CONTROL_1_REG 0x06 //inerface control 1 register
#define INTERFACE_CONTROL_2_REG 0x07 //interface control 2 register
#define PCMA_REG 0x1A
#define PCMB_REG 0x1B
#define MSTA_REG 0x20
#define MSTB_REG 0x21
#define SPKA_REG 0x24
#define SPKB_REG 0x25
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;

I2C_HandleTypeDef hi2c1;

SAI_HandleTypeDef hsai_BlockA1;

TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
// Configure all desired messages to the codec by adding 
// needed value of the register to the array
uint8_t POWER_UP[] = {(uint8_t)POWER_UP_REG,(uint8_t)0x9E}; //PU sequence 
uint8_t CLOCK_AUTO[] = {(uint8_t)CLOCK_CONTROL_REG,(uint8_t)0x80};
uint8_t PBCR_2[] = {(uint8_t)PLAYBACKC_2_REG, (uint8_t)0x08};
uint8_t PSAVOL[] = {(uint8_t)PASSAVOL_REG, (uint8_t)0x0F}; //volume +12dB channel A
uint8_t PSBVOL[] = {(uint8_t)PASSBVOL_REG, (uint8_t)0x0F}; //volume +12dB channel B 
// BEEP GENERATOR BUFFERS
//------------------------------------------------------
uint8_t MASTER_VOLUME[] = {(uint8_t)MASTER_VOLUME_REG,(uint8_t)0x18};//volume 12 db
uint8_t BEEPVOL_OFFTIME[] = {(uint8_t)BEEP_VOLUME_OFFTIME_REG,(uint8_t)0x06};//volume 12 db, off time 1.23s (for 96 kHz)
uint8_t BEEPFREQ_ONTIME[] = {(uint8_t)BEEP_FREQ_ONTIME_REG,(uint8_t)0x77};//beep freq 1000 Hz, on time 2.50s
uint8_t MISCEL_CONTROLS[] = {(uint8_t)MISCELLANEOUS_CONTROLS_REG, (uint8_t)0x60};//passthrough analog A to HP line enabled, B disabled and muted
uint8_t POWER_CR2[] = {(uint8_t)POWER_CR2_REG,(uint8_t)0xAF};//HP line always on, speaker power always off
uint8_t BEEP[] = {(uint8_t)BEEP_REG,(uint8_t)0x81};//multiple beeping, mix with SAI disabled, treble corner freq 5kHz, bass corner freq 50Hz, tone control enabled; change last!!!
//------------------------------------------------------
//----------------------BUFFERS-------------------------
uint8_t TX_BUFFER = 0;
uint8_t RX_BUFFER =  0;
uint8_t POWER_UP_BUFFER =  0x02; //address of power reg
uint8_t MODE_BUFFER =  0x02;
uint8_t CODAC_ID = 0x01;
uint8_t CODAC_MVC = 0x20;
uint8_t CODAC_BEEP = 0x1E; //BEEP address
uint8_t CODAC_BEEPING = 0x03; //continious beeping
uint8_t CODAC_BPVOL = 0x1D; //volume address
uint8_t CODAC_12DB = 0x06;
uint8_t CODAC_PU = 0x02;//power reg adr
uint8_t CODAC_PU_EN = 0x9E;//power reg status
uint8_t CODAC_IC_ADDR = 0x06;
uint8_t INTRFC_CR1 = 0x06;//Interface Control 1 reg address
uint8_t SAI_TRANSFER[] = {0xFF, 0xFF,0xFF, 0xFF,0xFF, 0xFF};//Message for SAI to send
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SAI1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void USER_TIM6_init(void)//TIMER 6 init
{
        RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;            

        TIM6->PSC = 0x0003;                            
        TIM6->ARR = 0xFF;       

        TIM6->CR2 |= TIM_CR2_MMS_1;    
        TIM6->CR1 |= TIM_CR1_CEN;                        
}  

void SAI_PASSTHROUGH_INIT (void)// Function initialises passthrough for PCM (SAI) to the HP
{
	//messages to send
	uint8_t pwr_up[] = {(uint8_t)POWER_UP_REG, (uint8_t)0x01};//power up sequense
	uint8_t pwr_2[] = {(uint8_t)POWER_CR2_REG, (uint8_t)0xAF};//HP always on, speaker muted
	uint8_t clk[] = {(uint8_t)CLOCK_CONTROL_REG, (uint8_t)0x80};//auto detection of clk signal
	uint8_t intfcntrl_1[] = {(uint8_t)INTERFACE_CONTROL_1_REG, (uint8_t)0x10};//slave with enabled DSP
	uint8_t miscel[] = {(uint8_t)MISCELLANEOUS_CONTROLS_REG, (uint8_t)0x30};//passthrough muted
	uint8_t pbck_ctrl_2[] = {(uint8_t)PLAYBACKC_2_REG, (uint8_t)0x38};//speaker muted, headphones not muted
	uint8_t pcma[] = {(uint8_t)PCMA_REG, (uint8_t)0x18};//PCM data from SAI volume +12dB on channel A to DSP
	uint8_t pcmb[] = {(uint8_t)PCMB_REG, (uint8_t)0x18};//PCM data from SAI volume +12dB on channel B to DSP
	uint8_t beep[] = {(uint8_t)BEEP_REG, (uint8_t)0x00};//BEEP generator is off
	uint8_t msta[] = {(uint8_t)MSTA_REG, (uint8_t)0x18};//master volume +12dB on channel A
	uint8_t mstb[] = {(uint8_t)MSTB_REG, (uint8_t)0x18};//master volume +12dB on channel B
	uint8_t hpa[] = {(uint8_t)HPA_VOLUME_REG, (uint8_t)0x00};//hp volume not muted (0dB)
	uint8_t hpb[] = {(uint8_t)HPB_VOLUME_REG, (uint8_t)0x00};//hp volume not muted (0dB)
	uint8_t spka[] = {(uint8_t)SPKA_REG,(uint8_t)0x01};//speaker muted
	uint8_t spkb[] = {(uint8_t)SPKB_REG,(uint8_t)0x01};//speaker muted
	
	//using HAL functions to set up the codac
	HAL_I2C_Master_Transmit(&hi2c1,CODAC_ADRESS_WRITE, pwr_up, 2, 100);
	HAL_I2C_Master_Transmit(&hi2c1,CODAC_ADRESS_WRITE, pwr_2, 2, 100);
	HAL_I2C_Master_Transmit(&hi2c1,CODAC_ADRESS_WRITE, clk, 2, 100);
	HAL_I2C_Master_Transmit(&hi2c1,CODAC_ADRESS_WRITE, intfcntrl_1, 2, 100);
	HAL_I2C_Master_Transmit(&hi2c1,CODAC_ADRESS_WRITE, miscel, 2, 100);
	HAL_I2C_Master_Transmit(&hi2c1,CODAC_ADRESS_WRITE, pbck_ctrl_2, 2, 100);
	HAL_I2C_Master_Transmit(&hi2c1,CODAC_ADRESS_WRITE, pcma, 2, 100);
	HAL_I2C_Master_Transmit(&hi2c1,CODAC_ADRESS_WRITE, pcmb, 2, 100);
	HAL_I2C_Master_Transmit(&hi2c1,CODAC_ADRESS_WRITE, beep, 2, 100);
	HAL_I2C_Master_Transmit(&hi2c1,CODAC_ADRESS_WRITE, msta, 2, 100);
	HAL_I2C_Master_Transmit(&hi2c1,CODAC_ADRESS_WRITE, mstb, 2, 100);
	HAL_I2C_Master_Transmit(&hi2c1,CODAC_ADRESS_WRITE, hpa, 2, 100);
	HAL_I2C_Master_Transmit(&hi2c1,CODAC_ADRESS_WRITE, hpb, 2, 100);
	HAL_I2C_Master_Transmit(&hi2c1,CODAC_ADRESS_WRITE, spka, 2, 100);
	HAL_I2C_Master_Transmit(&hi2c1,CODAC_ADRESS_WRITE, spkb, 2, 100);
	
	pwr_up[1] = (uint8_t)0x9E;//sequence for starting (look up datasheet for power up procedure)
	HAL_I2C_Master_Transmit(&hi2c1,CODAC_ADRESS_WRITE, pwr_up, 2, 100);//power up routine 
	
	__HAL_SAI_ENABLE(&hsai_BlockA1);
	
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
  MX_DAC1_Init();
  MX_I2C1_Init();
  MX_SAI1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	USER_TIM6_init();
	HAL_NVIC_EnableIRQ(TIM6_IRQn);
	HAL_NVIC_EnableIRQ(SAI1_IRQn);
	
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);//Reset pin on codac is HIGH (see datasheet for details)
	//Codac setup via I2C1 begin
	// Reading from registers to make sure that it works and checking 
	// if register Power Ctl 1 is set to 0x01
  HAL_I2C_Master_Transmit(&hi2c1,CODAC_ADRESS_WRITE,&CODAC_ID,1,TIMEOUT);
	HAL_I2C_Master_Receive(&hi2c1,CODAC_ADRESS_READ,&RX_BUFFER,1,TIMEOUT);
	HAL_I2C_Master_Transmit(&hi2c1,CODAC_ADRESS_WRITE,&POWER_UP_BUFFER,1,TIMEOUT);
	HAL_I2C_Master_Receive(&hi2c1,CODAC_ADRESS_READ,&RX_BUFFER,1,TIMEOUT);
	HAL_I2C_Master_Transmit(&hi2c1,CODAC_ADRESS_WRITE,&INTRFC_CR1,1,TIMEOUT);
	HAL_I2C_Master_Receive(&hi2c1,CODAC_ADRESS_READ,&RX_BUFFER,1,TIMEOUT);
	// End
	//To enable the beep generator, comment SAI_PASSTHROUGH_INIT() and uncomment sequence below
	SAI_PASSTHROUGH_INIT();//Custom function to setup SAI transmit
	/*------------------BEEP GENERATOR INIT BEGIN------------------------------------
	HAL_I2C_Master_Transmit(&hi2c1,CODAC_ADRESS_WRITE,CLOCK_AUTO,2,TIMEOUT);
	//  BEEP GENERATOR SETUP FOR A MULTIPLE PULSES
	//---------------------------------------------------------------------------
	HAL_I2C_Master_Transmit(&hi2c1,CODAC_ADRESS_WRITE,POWER_CR2,2,TIMEOUT);
	HAL_I2C_Master_Receive(&hi2c1,CODAC_ADRESS_READ,&RX_BUFFER,1,TIMEOUT);	
	
	HAL_I2C_Master_Transmit(&hi2c1,CODAC_ADRESS_WRITE,MASTER_VOLUME,2,TIMEOUT);
	HAL_I2C_Master_Receive(&hi2c1,CODAC_ADRESS_READ,&RX_BUFFER,1,TIMEOUT);	

	HAL_I2C_Master_Transmit(&hi2c1,CODAC_ADRESS_WRITE,BEEPVOL_OFFTIME,2,TIMEOUT);
	HAL_I2C_Master_Receive(&hi2c1,CODAC_ADRESS_READ,&RX_BUFFER,1,TIMEOUT);
	
	HAL_I2C_Master_Transmit(&hi2c1,CODAC_ADRESS_WRITE,BEEPFREQ_ONTIME,2,TIMEOUT);
	HAL_I2C_Master_Receive(&hi2c1,CODAC_ADRESS_READ,&RX_BUFFER,1,TIMEOUT);	
	
	HAL_I2C_Master_Transmit(&hi2c1,CODAC_ADRESS_WRITE,MISCEL_CONTROLS,2,TIMEOUT);
	HAL_I2C_Master_Receive(&hi2c1,CODAC_ADRESS_READ,&RX_BUFFER,1,TIMEOUT);
	

	HAL_I2C_Master_Transmit(&hi2c1,CODAC_ADRESS_WRITE,BEEP,2,TIMEOUT);
	HAL_I2C_Master_Receive(&hi2c1,CODAC_ADRESS_READ,&RX_BUFFER,1,TIMEOUT);
	
	HAL_I2C_Master_Transmit(&hi2c1,CODAC_ADRESS_WRITE,POWER_UP,2,TIMEOUT);
	HAL_I2C_Master_Receive(&hi2c1,CODAC_ADRESS_READ,&RX_BUFFER,1,TIMEOUT);
	
	__HAL_SAI_ENABLE(&hsai_BlockA1);
	
	HAL_SAI_Transmit_IT(&hsai_BlockA1, SAI_TRANSFER, 2);//Transfer data to pump up the codac
	------------------BEEP GENERATOR INIT END--------------------------------------*/

  //Codac setup via I2C1 end
	/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

		//HAL_SAI_Transmit use in to transmit data in blocking mode
		//or use HAL_SAI_Transmit_IT to send data in nonblocking mode
		
		HAL_SAI_Transmit(&hsai_BlockA1, SAI_TRANSFER, 6, 10);
		HAL_Delay(10);//delay for transfer time(?) without it there won't be time for volume amplification
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */
  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Triangle wave generation on DAC OUT1
  */
  if (HAL_DACEx_TriangleWaveGenerate(&hdac1, DAC_CHANNEL_1, DAC_TRIANGLEAMPLITUDE_31) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Noise wave generation on DAC OUT2
  */
  if (HAL_DACEx_NoiseWaveGenerate(&hdac1, DAC_CHANNEL_2, DAC_LFSRUNMASK_BIT0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */
		RCC->APB1ENR1 |= RCC_APB1ENR1_DAC1EN;
	
    DAC->CR |= DAC_CR_TEN1; // trigger enable
    DAC->CR |= DAC_CR_MAMP1; // MAx amplitude
    DAC->CR |= DAC_CR_WAVE1_0; // Noise generator activated ch1

		DAC->CR |= DAC_CR_TEN2; // trigger enable
    DAC->CR |= DAC_CR_MAMP2; // MAx amplitude
    DAC->CR |= DAC_CR_WAVE2_1; // Noise generator activated ch2

		DAC->CR |= DAC_CR_EN1; // Enable ch1
    DAC->CR |= DAC_CR_EN2; // Enable ch2 
	
		HAL_DAC_Start(&hdac1,DAC1_CHANNEL_1);
		HAL_DAC_Start(&hdac1,DAC1_CHANNEL_2);	
  /* USER CODE END DAC1_Init 2 */

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
  hi2c1.Init.Timing = 0x20303E5D;
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
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */
  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA1.Init.DataSize = SAI_DATASIZE_8;
  hsai_BlockA1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_96K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockA1.FrameInit.FrameLength = 8;
  hsai_BlockA1.FrameInit.ActiveFrameLength = 1;
  hsai_BlockA1.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockA1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockA1.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockA1.SlotInit.FirstBitOffset = 0;
  hsai_BlockA1.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockA1.SlotInit.SlotNumber = 1;
  hsai_BlockA1.SlotInit.SlotActive = 0x00000000;
  if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
