/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * 
  * Ho va ten: Bui Van An  (vanan92)
  * Oscilloscope: 2CH  (1MSPS/CH)
  * Version: 1.2.2
  * Bluepill - Stm32f103c8t6 80MHz overclock
  * Feel free to use this source code!
  * Really thanks to: Wilko Lunenburg - https://www.youtube.com/watch?v=m5hbsKATEz8&t=0s
  *                   Radiopench1 - https://www.youtube.com/watch?v=SwaZkHX7DKU&t=0s
  *                   JYEtech - https://github.com/JYEtech/DSO-Shell-open-source-version-
  *                       
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "st7789.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define NUM_SAMPLES 		  2048
#define TFT_WIDTH         ((uint16_t)240)
#define TFT_HEIGHT			  ((uint16_t)240)
#define GRID_WIDTH			  ((uint16_t)240)
#define GRID_HEIGHT			  ((uint16_t)200)
#define A1 					      0
#define A2 					      1

#define vRangeA_High    GPIOC->BSRR |= (1<<14)      //HIGH
#define vRangeA_Low     GPIOC->BSRR |= (1<<14)<<16         //LOW
#define vRangeB_High    GPIOC->BSRR |= (1<<15)      //HIGH
#define vRangeB_Low     GPIOC->BSRR |= (1<<15)<<16         //LOW




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

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

 typedef struct Stats {
	int duty;
	int freq;
	float cycle;
	float Vmaxf;
	float Vminf;
} t_Stats;
t_Stats Stats;
int NUM_ADC=NUM_SAMPLES,
    NUM_ADC_new=NUM_SAMPLES,
	NUM_ADC_old=NUM_SAMPLES;

uint8_t  hOffset = (TFT_WIDTH - GRID_WIDTH)/2,
         Hold_old =0,
         trigger_pos_drawing=0,
         prepare_mode_change=0,
         old_zeropos_A1=120,old_zeropos_A2=120;
uint16_t ch1Capture[NUM_SAMPLES], ch2Capture[NUM_SAMPLES],
         zeroVoltageA1 = 2048, zeroVoltageA2 = 2048, //0V
         ADC_2_GRID_A=2048, ADC_2_GRID_B=2048, 
         ADC_2_GRID_OLD_A=2048, ADC_2_GRID_OLD_B=2048,
         ch1Capture_zoom[250],ch2Capture_zoom[250],
         trigger_color=0,
         trigger_point,
         Stats_color=0,
         trigger_extra_point=0,
         i=0;
int16_t  ch1Old[GRID_WIDTH+5] = {0}, ch2Old[GRID_WIDTH+5] = {0}, 
				 old_trigger_pos_X=0, old_trigger_pos_Y=0,
				 val1, val2,
         transposedPt1, transposedPt2;
volatile int16_t trigger_pos=0,
                 trigger_pos_X=120,
                 yCursors[2] = {-100,-100};
volatile uint8_t trigger_pos_Y=120,
	               Stats_source =1,
                 Hold =0, 
                 Bt_pressed=1,
                 trigger_source = 1,
                 vOffset = (TFT_HEIGHT - GRID_HEIGHT)/2,
                 show_mode=1,
                 position_wave=0,
                 conversion_ready=0,data_flag=0,
                 XY_MODE=0, //0-1
                 Legh_data=2,// 1:256pt:     2:512pt;     3:768pt;    4:1024pt  5:1280pt  6:1536pt  7:1792pt  8:2048pt
                 MENU=1; //1-2-3
uint32_t adc_buffer[NUM_SAMPLES];
int32_t  trigger_level;	
const  uint16_t vRangeValue[] = {310, 620, 1552, 3103, 750, 1875, 3750}; //Fullscale 200pixel= 310(raw_ADC)/0.1V; 620/0.2V; 1552/0.5V; ... 3750/10V
const  float hRangeValue[]    = { 0.2, 0.1, 0.05, 0.02, 0.01, 0.005, 0.002, 0.001, 0.5e-3, 0.2e-3, 0.1e-3, 0.5e-4, 0.2e-4, 0.2e-4, 0.2e-4};
volatile int hRange=10, vRangeA=3, vRangeB=3;  /*   V-range            H-range
                                                    0:0.1V             0:200ms,      7:1ms,
                                                    1:0.2V             1:100ms,      8:500us, 
                                                    2:0.5V             2:50ms,       9:200us,
                                                    3:  1V             3:20ms,       10:100us,
                                                    4:  2V             4:10ms,       11:50s,
                                                    5:  5V             5:5ms,        12:20us,
                                                    6: 10V             6:2ms,           */

static int16_t yCursorsSnap[2],yCursorsOld[2];
static bool wavesSnap[2],wavesOld[2] = {false,false};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void plotLineSegment(int16_t transposedPt1, int16_t transposedPt2,  int index, uint16_t color);
void MARSK_SCREEN(void);
void DRAW_TRIGGER_ICON(void);
void SEARCH_TRIGGER_POINT(void);
void WAVE_INFO_CALCULATOR (void);
void DRAW_GRIDS(void);
void DRAW_HOLD_MASK(void);
void CLEAR_HOLD_MASK(void);
void DRAW_STATS_WAVE(void);
void START_ADC_AGAIN(void);
void DRAW_WAVES(void);
void ZOOM_DATA(void);
void DRAW_WAVE_POS_ICON(void);
void DRAW_WAVES_HOLD_MODE(void);
void XY_DRAW(void);


int sum3(int k) {       // Sum of before and after and own value
	int m=0;
	if (Stats_source==1) {m = ch1Capture[k - 1] + ch1Capture[k] + ch1Capture[k + 1];}
	else {m = ch2Capture[k - 1] + ch2Capture[k] + ch2Capture[k + 1];}
 	return m;
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

	//uint16_t previous_trigger_point;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /*
	TIMER 2: Dung de set time base, tranh khi timebase lon bi tre.
	TIMER 3: ADC Strigger. Important!
	*/
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	
	LL_ADC_Enable(ADC1);
	LL_ADC_StartCalibration(ADC1);
	while (LL_ADC_IsCalibrationOnGoing(ADC1));
	LL_mDelay(10);

	LL_ADC_Enable(ADC2);
	LL_ADC_StartCalibration(ADC2);
	while (LL_ADC_IsCalibrationOnGoing(ADC2));
	LL_mDelay(10);
	
	ST7789_Init();
	ST7789_FillScreen(BLACK);
	conversion_ready = 1;
	trigger_point = 1;		
	
	//DMA_ADC first start:
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t) &ADC1->DR);
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t) &adc_buffer);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, NUM_SAMPLES);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);								
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

	//TIM3(trigger) -> ADC:
	LL_ADC_REG_StartConversionExtTrig(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
	LL_ADC_REG_StartConversionExtTrig(ADC2, LL_ADC_REG_TRIG_EXT_RISING);
	LL_TIM_EnableCounter(TIM3);


	ST7789_WriteString(24, 102, "OSCILLOSCOPE", Font_16x26, WHITE, BLACK);
	ST7789_WriteString(50, 130, "Sampling Rate 1MSa/s", Font_7x10, WHITE, BLACK);
	LL_mDelay(2000);
	ST7789_WriteString(46, 130, "       Vanan92       ", Font_7x10, WHITE, BLACK);
	LL_mDelay(2000);
	ST7789_WriteString(50, 130, "       V1.2.2       ", Font_7x10, WHITE, BLACK);
	LL_mDelay(1500);
	ST7789_FillScreen(BLACK);
	LL_mDelay(200);
	


	//LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);  //PWM 1kHz
	GPIOC->BSRR |= (1<<13);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		uint8_t bt_a = 0;
		if(Bt_pressed==1) {Bt_pressed=0; bt_a=1;}
		if (XY_MODE==1)
		{
			if(prepare_mode_change==0) //prepare from normal mode to XY mode: clear lcd,...
			{
				if(hRange>12) hRange=12;
				if(MENU>2) MENU=7;
				val1 =0; val2 =0;
				for(int i=0; i < GRID_WIDTH; i++)	//run from 0 -> 239+1
				{
					// erase old line segment 
					plotLineSegment(ch1Old[i], ch1Old[i + 1], i, BLACK);
					plotLineSegment(ch2Old[i], ch2Old[i + 1], i, BLACK);
				}
				CLEAR_HOLD_MASK();
				ST7789_FillRectangle(110, 0, 70, 18, BLACK);
				ST7789_DrawFastVLine(0,old_zeropos_A1-2,old_zeropos_A1+2, BLACK);
				ST7789_DrawFastVLine(1,old_zeropos_A1-2,old_zeropos_A1+2, BLACK);
				ST7789_DrawFastVLine(2,old_zeropos_A1-1,old_zeropos_A1+1, BLACK);
				ST7789_DrawPixel(3, old_zeropos_A1, BLACK);
				ST7789_DrawFastVLine(239,old_zeropos_A2-2,old_zeropos_A2+2, BLACK);
				ST7789_DrawFastVLine(238,old_zeropos_A2-2,old_zeropos_A2+2, BLACK);
				ST7789_DrawFastVLine(237,old_zeropos_A2-1,old_zeropos_A2+1, BLACK);
				ST7789_DrawPixel(236, old_zeropos_A2, BLACK);
				yCursors[0] = -100;yCursors[1] = -100;trigger_pos_X=120;trigger_pos_Y=120;show_mode=1;trigger_pos=120;
				MARSK_SCREEN();
				DRAW_GRIDS();
				prepare_mode_change=1;
			}
			if (conversion_ready==1)
			{
				if(Hold==0)
				{
					CLEAR_HOLD_MASK();
					//SEARCH_TRIGGER_POINT();
					WAVE_INFO_CALCULATOR();
					MARSK_SCREEN();
					DRAW_GRIDS();
					XY_DRAW();
					DRAW_STATS_WAVE();
					START_ADC_AGAIN();
					NUM_ADC=NUM_ADC_new;
				}
				else
				{
					DRAW_HOLD_MASK();
					if (bt_a==1)
					{
						WAVE_INFO_CALCULATOR();
						SEARCH_TRIGGER_POINT();
						MARSK_SCREEN();
						DRAW_GRIDS();
						XY_DRAW();
						DRAW_STATS_WAVE();
						//START_ADC_AGAIN();
						//NUM_ADC=NUM_ADC_new;
					}
				}
			}
			else if (bt_a==1)
			{
				if(Hold==0)
				{
					CLEAR_HOLD_MASK();
				}
				else
				{
					DRAW_HOLD_MASK();
				}
				WAVE_INFO_CALCULATOR();
				SEARCH_TRIGGER_POINT();
				MARSK_SCREEN();
				DRAW_GRIDS();
				XY_DRAW();
				DRAW_STATS_WAVE();
				//START_ADC_AGAIN();
				//NUM_ADC=NUM_ADC_new;
			}
		}
		else  //prepare from XY mode to normal mode: clear lcd,...
		{
			if(MENU==7) MENU=1;
			if(prepare_mode_change==1)
			{
				for(int i = 0; i < NUM_ADC_old; i++) //410 is more clear
				{
					ST7789_DrawPixel(ch1Capture[i], ch2Capture[i], BLACK);			
				}
				CLEAR_HOLD_MASK();
				ST7789_FillRectangle(80, 0, 70, 18, BLACK);
				MARSK_SCREEN();
				DRAW_GRIDS();
				prepare_mode_change=0;
			}
		}
		if      (show_mode==1) {wavesSnap[A1]=true; wavesSnap[A2]=true;}
		else if (show_mode==2) {wavesSnap[A1]=true; wavesSnap[A2]=false;}
		else                   {wavesSnap[A1]=false;wavesSnap[A2]=true;}
		if(Hold==1 && XY_MODE==0) //Hold = ON;
		{
			Hold_old=1;
			if (bt_a==1)
			{
				SEARCH_TRIGGER_POINT();
				MARSK_SCREEN();
				DRAW_TRIGGER_ICON();
				if (MENU==4) //SHOW
					DRAW_WAVES_HOLD_MODE();
				else
				{
					DRAW_GRIDS();
					DRAW_WAVES();
				}
				DRAW_HOLD_MASK();
				DRAW_WAVE_POS_ICON();
				DRAW_STATS_WAVE();
			}
		}
		else if(Hold_old==1 && XY_MODE==0)
		{
				START_ADC_AGAIN();
				CLEAR_HOLD_MASK();
				if (bt_a==1)
				{
					SEARCH_TRIGGER_POINT();
					MARSK_SCREEN();
					DRAW_TRIGGER_ICON();
					if (MENU==4) //SHOW
						DRAW_WAVES_HOLD_MODE();
					else
					{
						DRAW_GRIDS();
						DRAW_WAVES();
					}
					DRAW_WAVE_POS_ICON();
					DRAW_STATS_WAVE();
				}
				//if (conversion_ready==1) Hold_old=0;
				Hold_old=0;
		}
		else if (conversion_ready==1 && XY_MODE==0)
		{
			for (i=0; i<NUM_ADC; i++)
			{
				ch1Capture[i] = (uint16_t) ((adc_buffer[i] & 0x0000FFFF));
				ch2Capture[i] = (uint16_t) ( adc_buffer[i] >> 16); 
			}
			WAVE_INFO_CALCULATOR();
			SEARCH_TRIGGER_POINT();
			MARSK_SCREEN();
			DRAW_TRIGGER_ICON();
			if (hRange<6) //SHOW
					DRAW_WAVES_HOLD_MODE();
			else
			{
					DRAW_GRIDS();
					DRAW_WAVES();
			}
			DRAW_WAVE_POS_ICON();
			DRAW_STATS_WAVE();
			START_ADC_AGAIN();
			NUM_ADC=NUM_ADC_new;
		}
		else if (bt_a==1 && XY_MODE==0)
		{
			SEARCH_TRIGGER_POINT();
			MARSK_SCREEN();
			DRAW_TRIGGER_ICON();
			if (MENU==4) //SHOW
				DRAW_WAVES_HOLD_MODE();
			else
			{
				DRAW_GRIDS();
				DRAW_WAVES();
			}
			DRAW_WAVE_POS_ICON();
			DRAW_STATS_WAVE();
		}
		
		//==================================================================================
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_10);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(80000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSRC_PCLK2_DIV_4);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**ADC1 GPIO Configuration
  PA2   ------> ADC1_IN2
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* ADC1 DMA Init */

  /* ADC1 Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_VERYHIGH);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_WORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_WORD);

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_DUAL_REG_SIMULT;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_EXT_TIM3_TRGO;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_2);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_2, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC2);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**ADC2 GPIO Configuration
  PA4   ------> ADC2_IN4
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
  LL_ADC_Init(ADC2, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  LL_ADC_REG_Init(ADC2, &ADC_REG_InitStruct);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_4);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 399;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM3);
  LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_UPDATE);
  LL_TIM_DisableMasterSlaveMode(TIM3);
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 99;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 799;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 399;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOC, LED_Pin|vRangeA_SW_Pin|vRangeB_SW_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, NC_Pin|RST_ST7789_Pin);

  /**/
  LL_GPIO_SetOutputPin(DC_ST7789_GPIO_Port, DC_ST7789_Pin);

  /**/
  GPIO_InitStruct.Pin = LED_Pin|vRangeA_SW_Pin|vRangeB_SW_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = NC_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(NC_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = DC_ST7789_Pin|RST_ST7789_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTA, LL_GPIO_AF_EXTI_LINE6);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTA, LL_GPIO_AF_EXTI_LINE7);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE11);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE12);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTA, LL_GPIO_AF_EXTI_LINE9);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTA, LL_GPIO_AF_EXTI_LINE10);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_6;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_7;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_11;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_12;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_9;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_10;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  LL_GPIO_SetPinPull(LEFT_Button_GPIO_Port, LEFT_Button_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinPull(RIGHT_Button_GPIO_Port, RIGHT_Button_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinPull(HOLD_Button_GPIO_Port, HOLD_Button_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinPull(SELECT_Button_GPIO_Port, SELECT_Button_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinPull(DOWN_Button_GPIO_Port, DOWN_Button_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinPull(UP_Button_GPIO_Port, UP_Button_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinMode(LEFT_Button_GPIO_Port, LEFT_Button_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(RIGHT_Button_GPIO_Port, RIGHT_Button_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(HOLD_Button_GPIO_Port, HOLD_Button_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(SELECT_Button_GPIO_Port, SELECT_Button_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(DOWN_Button_GPIO_Port, DOWN_Button_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(UP_Button_GPIO_Port, UP_Button_Pin, LL_GPIO_MODE_INPUT);

  /* EXTI interrupt init*/
  NVIC_SetPriority(EXTI9_5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(EXTI9_5_IRQn);
  NVIC_SetPriority(EXTI15_10_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void plotLineSegment(int16_t transposedPt1, int16_t transposedPt2,  int index, uint16_t color)
{
	// range checks
	if(transposedPt1 > (GRID_HEIGHT + vOffset-2))   //voffset=240/2=120
		transposedPt1 = GRID_HEIGHT + vOffset-1;
	if(transposedPt1 < vOffset+1)
		transposedPt1 = vOffset+1;
	if(transposedPt2 > (GRID_HEIGHT + vOffset-2))
		transposedPt2 = GRID_HEIGHT + vOffset-1;
	if(transposedPt2 < vOffset+1)
		transposedPt2 = vOffset+1;
  
  	// draw the line segments
  	//Note.. If Pt1 < Pt2 here it leads to strange drawing artifacts where the 
  	//verticval line sign seems to be flipped... Somewhere a bug in the graphics library...
  	if (transposedPt1<transposedPt2)
  	{
  		ST7789_DrawFastVLine(index + hOffset,transposedPt1,transposedPt2,color);
   	 	//ST7789_DrawLine(index + hOffset, transposedPt1, index + hOffset, transposedPt2, color);
  	}
  	else
  	{
  		ST7789_DrawFastVLine(index + hOffset,transposedPt2,transposedPt1,color);
		//ST7789_DrawLine(index + hOffset, transposedPt2, index + hOffset, transposedPt1, color);
	                  //x1= 1+0           y1              x2=1+0                y2       color
  	} 
}

void MARSK_SCREEN(void)
{
	switch (hRange)
	{
		case  0: ST7789_WriteString(0, 0, "200ms", Font_7x10, WHITE, BLACK);break;
		case  1: ST7789_WriteString(0, 0, "100ms", Font_7x10, WHITE, BLACK);break;
		case  2: ST7789_WriteString(0, 0, "50ms ", Font_7x10, WHITE, BLACK);break;
		case  3: ST7789_WriteString(0, 0, "20ms ", Font_7x10, WHITE, BLACK);break;
		case  4: ST7789_WriteString(0, 0, "10ms ", Font_7x10, WHITE, BLACK);break;
		case  5: ST7789_WriteString(0, 0, "5ms  ", Font_7x10, WHITE, BLACK);break;
		case  6: ST7789_WriteString(0, 0, "2ms  ", Font_7x10, WHITE, BLACK);break;
		case  7: ST7789_WriteString(0, 0, "1ms  ", Font_7x10, WHITE, BLACK);break;
		case  8: ST7789_WriteString(0, 0, "500us", Font_7x10, WHITE, BLACK);break;
		case  9: ST7789_WriteString(0, 0, "200us", Font_7x10, WHITE, BLACK);break;
		case 10: ST7789_WriteString(0, 0, "100us", Font_7x10, WHITE, BLACK);break;
		case 11: ST7789_WriteString(0, 0, "50us ", Font_7x10, WHITE, BLACK);break;
		case 12: ST7789_WriteString(0, 0, "20us ", Font_7x10, WHITE, BLACK);break;
		case 13: ST7789_WriteString(0, 0, "10us ", Font_7x10, WHITE, BLACK);break;
		case 14: ST7789_WriteString(0, 0, "5us  ", Font_7x10, WHITE, BLACK);break;
	}
	uint8_t X_pos=183;
	switch (vRangeB)
	{
		case  0:  ST7789_WriteString(211,0, "0.1V", Font_7x10, WHITE, BLACK);break;
		case  1:  ST7789_WriteString(211,0, "0.2V", Font_7x10, WHITE, BLACK);break;
		case  2:  ST7789_WriteString(211,0, "0.5V", Font_7x10, WHITE, BLACK);break;
		case  3: {ST7789_WriteString(211,0, "  1V", Font_7x10, WHITE, BLACK);ST7789_WriteString(X_pos,0, "  ", Font_7x10, WHITE, BLACK);X_pos+=14;break;}
		case  4: {ST7789_WriteString(211,0, "  2V", Font_7x10, WHITE, BLACK);ST7789_WriteString(X_pos,0, "  ", Font_7x10, WHITE, BLACK);X_pos+=14;break;}
		case  5: {ST7789_WriteString(211,0, "  5V", Font_7x10, WHITE, BLACK);ST7789_WriteString(X_pos,0, "  ", Font_7x10, WHITE, BLACK);X_pos+=14;break;}
		case  6: {ST7789_WriteString(211,0, " 10V", Font_7x10, WHITE, BLACK);ST7789_WriteString(X_pos,0,  " ", Font_7x10, WHITE, BLACK);X_pos+= 7;break;}
	}
	switch (vRangeA)
	{
		case  0:  ST7789_WriteString(X_pos,0, "0.1/", Font_7x10, WHITE, BLACK);break;
		case  1:  ST7789_WriteString(X_pos,0, "0.2/", Font_7x10, WHITE, BLACK);break;
		case  2:  ST7789_WriteString(X_pos,0, "0.5/", Font_7x10, WHITE, BLACK);break;
		case  3:  ST7789_WriteString(X_pos,0, "  1/", Font_7x10, WHITE, BLACK);break;
		case  4:  ST7789_WriteString(X_pos,0, "  2/", Font_7x10, WHITE, BLACK);break;
		case  5:  ST7789_WriteString(X_pos,0, "  5/", Font_7x10, WHITE, BLACK);break;
		case  6:  ST7789_WriteString(X_pos,0, " 10/", Font_7x10, WHITE, BLACK);break;
	}
	
	ST7789_WriteString(0,  230,  "Vpp:", Font_7x10, WHITE, BLACK);
	ST7789_WriteString(81, 230, "Duty:", Font_7x10, WHITE, BLACK);
	ST7789_WriteString(163,230, "Freq:", Font_7x10, WHITE, BLACK);
	if (MENU==1)
	{
		ST7789_WriteString(60, 0, " CH1   ", Font_7x10, YELLOW, BLACK);
		ST7789_WriteString(102, 0, "      ", Font_7x10, YELLOW, BLACK);
		if (show_mode<3)  ST7789_WriteString(140, 0, "     ", Font_7x10, YELLOW, BLACK);
	}
	if (MENU==2)
	{
		ST7789_WriteString(60, 0, " CH2 ", Font_7x10, YELLOW, BLACK);
		if (show_mode!=2)  ST7789_WriteString(140, 0, "     ", Font_7x10, YELLOW, BLACK);
	}
	if (MENU==3)
	{
		ST7789_WriteString(60, 0, "MOVE ", Font_7x10, YELLOW, BLACK);
		if (position_wave==0) ST7789_WriteString(140, 0, " CH1 ", Font_7x10, YELLOW, BLACK);
		else ST7789_WriteString(140, 0, " CH2 ", Font_7x10, YELLOW, BLACK);
	}
	else if (MENU==4)
	{
		ST7789_WriteString(60, 0, "SHOW ", Font_7x10, YELLOW, BLACK);
		if      (show_mode==1) ST7789_WriteString(140, 0, "CH1+2", Font_7x10, YELLOW, BLACK);
		else if (show_mode==2) ST7789_WriteString(140, 0, " CH1 ", Font_7x10, YELLOW, BLACK);
		else if (show_mode==3) ST7789_WriteString(140, 0, " CH2 ", Font_7x10, YELLOW, BLACK);
	}
	else if (MENU==5)
	{
		ST7789_WriteString(60, 0, "TRIGG", Font_7x10, YELLOW, BLACK);
		if (trigger_source==1 || trigger_source==3) ST7789_WriteString(140, 0, " CH1 ", Font_7x10, YELLOW, BLACK);
		else ST7789_WriteString(140, 0, " CH2 ", Font_7x10, YELLOW, BLACK);
	}
	else if (MENU==6)
	{
		ST7789_WriteString(60, 0, "T-Pos", Font_7x10, YELLOW, BLACK);
		ST7789_WriteString(140, 0, " X-Y ", Font_7x10, YELLOW, BLACK);
	}
	else if (MENU==7)
	{
		ST7789_WriteString(60, 0, "Length:", Font_7x10, YELLOW, BLACK);
		switch (Legh_data)
		{
			case 1:
			{
				ST7789_WriteString(115, 0, " 256pt", Font_7x10, YELLOW, BLACK);break;
			}
			case 2:
			{
				ST7789_WriteString(115, 0, " 512pt", Font_7x10, YELLOW, BLACK);break;
			}
			case 3:
			{
				ST7789_WriteString(115, 0, " 768pt", Font_7x10, YELLOW, BLACK);break;
			}
			case 4:
			{
				ST7789_WriteString(115, 0, "1024pt", Font_7x10, YELLOW, BLACK);break;
			}
			case 5:
			{
				ST7789_WriteString(115, 0, "1280pt", Font_7x10, YELLOW, BLACK);break;
			}
			case 6:
			{
				ST7789_WriteString(115, 0, "1536pt", Font_7x10, YELLOW, BLACK);break;
			}
			case 7:
			{
				ST7789_WriteString(115, 0, "1792pt", Font_7x10, YELLOW, BLACK);break;
			}
			case 8:
			{
				ST7789_WriteString(115, 0, "2048pt", Font_7x10, YELLOW, BLACK);break;
			}
		}
		//Length_data=1;// 1:256pt:     2:512pt;     3:768pt;    4:1024pt  5:1280pt  6:1536pt  7:1792pt  8:2048pt
	}
}
void DRAW_TRIGGER_ICON(void)
{
	if (trigger_source%2==0) trigger_color = GREEN; else  trigger_color = MAGENTA;
		
	if (trigger_source>2)
	{
			ST7789_DrawFastHLine(121, 125, 0, BLACK);
			ST7789_DrawFastHLine(115, 119,10, BLACK);
			ST7789_DrawFastHLine(119, 121, 4, BLACK);
			ST7789_DrawFastHLine(117, 123, 6, BLACK);

			ST7789_DrawFastHLine(115, 119, 0, trigger_color);
			ST7789_DrawFastVLine(120,   0,10, trigger_color);
			ST7789_DrawFastHLine(121, 125,10, trigger_color);
			ST7789_DrawFastHLine(119, 121, 6, trigger_color);
			ST7789_DrawFastHLine(118, 122, 5, trigger_color);
			ST7789_DrawFastHLine(117, 123, 4, trigger_color);
	}
	else
	{
			ST7789_DrawFastHLine(115, 119, 0, BLACK);
			ST7789_DrawFastHLine(121, 125,10, BLACK);
			ST7789_DrawFastHLine(119, 121, 6, BLACK);
			ST7789_DrawFastHLine(117, 123, 4, BLACK);

			ST7789_DrawFastHLine(121, 125, 0, trigger_color);
			ST7789_DrawFastVLine(120,   0,10, trigger_color);
			ST7789_DrawFastHLine(115, 119,10, trigger_color);
			ST7789_DrawFastHLine(119, 121, 4, trigger_color);
			ST7789_DrawFastHLine(118, 122, 5, trigger_color);
			ST7789_DrawFastHLine(117, 123, 6, trigger_color);
	}
}
void SEARCH_TRIGGER_POINT(void)
{
	//Search trigger point:
	//uint16_t start = trigger_pos_Y+10;
	//i = NUM_SAMPLES+trigger_pos_Y-240-2;
	//uint16_t trigger_extr_level=Stats.Vmaxf-(Stats.Vmaxf-Stats.Vminf)/4;
	ADC_2_GRID_A = vRangeValue[vRangeA];
	ADC_2_GRID_B = vRangeValue[vRangeB];
	if (XY_MODE==0)
	{
		i = NUM_ADC-242;
		if (trigger_source%2==1) trigger_level = (int16_t)(zeroVoltageA1+(trigger_pos_X-120)*ADC_2_GRID_A/GRID_HEIGHT); 
		else                     trigger_level = (int16_t)(zeroVoltageA2+(trigger_pos_X-120)*ADC_2_GRID_B/GRID_HEIGHT); 
		uint16_t start = trigger_pos_Y+10;
		if (trigger_source==1) {while (i>start && ((ch1Capture[i - 1] > trigger_level) || (ch1Capture[i] <= trigger_level))) i--;}; // -_  rise
		if (trigger_source==2) {while (i>start && ((ch2Capture[i - 1] > trigger_level) || (ch2Capture[i] <= trigger_level))) i--;}; // -_  rise
		if (trigger_source==3) {while (i>start && ((ch1Capture[i - 1] < trigger_level) || (ch1Capture[i] >= trigger_level))) i--;}; // _   fall
		if (trigger_source==4) {while (i>start && ((ch2Capture[i - 1] < trigger_level) || (ch2Capture[i] >= trigger_level))) i--;}; // _-  fall
		i--;// vi ly do quet nguoc tu duoi len. bo di neu quet tu dau den cuoi.
	}
	
	yCursorsSnap[A1] = yCursors[A1]; // set y in the middle
	yCursorsSnap[A2] = yCursors[A2]; // set y in the middle
	trigger_point = i-trigger_pos_Y;
	ZOOM_DATA();
	if (trigger_source%2==1) trigger_pos = GRID_HEIGHT + vOffset + yCursorsSnap[A1] - (trigger_pos_X-120);
	else                     trigger_pos = GRID_HEIGHT + vOffset + yCursorsSnap[A2] - (trigger_pos_X-120);
}
void WAVE_INFO_CALCULATOR(void)
{
	//calculate average
	int  d;
	long long sum=0;
	//int  dataAve;                             // 10 x average value (use 10x value to keep accuracy. so, max=10230)
	//search max and min value
	uint16_t dataMin = 4096;                    // min value initialize to big number
	uint16_t dataMax = 0;                       // max value initialize to small number
	if (Stats_source==1)  //Chennal A
	{
		for (int o = 0; o < NUM_ADC; o++)      // serach max min value
		{
			d = ch1Capture[o];                //= 0->220
			sum = sum + d;
			if (d < dataMin) {dataMin = d;}       // update min
			if (d > dataMax) {dataMax = d;}       // updata max
		}
	}
	else  //stats_source=2 //Chennal B
	{
		for (int o = 0; o < NUM_ADC; o++)      // serach max min value
		{
			d = ch2Capture[o];                //= 0->220
			sum = sum + d;
			if (d < dataMin) {dataMin = d;}       // update min
			if (d > dataMax) {dataMax = d;}       // updata max
		}
	}
	//dataAve = (sum+10)/50;                       // Average value calculation (calculated by 10 times to improve accuracy)
	int swingCenter;                              // center of wave (half of p-p)
	float p0 = 0;                                 // 1-st posi edge
	float p1 = 0;                                 // total length of cycles
	float p2 = 0;                                 // total length of pulse high time
	float pFine = 0;                              // fine position (0-1.0)
	float lastPosiEdge;                           // last positive edge position

	float pPeriod;                                // pulse period
	float pWidth;                                 // pulse width

	int p1Count = 0;                              // wave cycle count
	int p2Count = 0;                              // High time count

	uint8_t a0Detected = 0;
	//boolean b0Detected = false;
	uint8_t posiSerch = 1;                      // true when serching posi edge
	Stats.Vmaxf = dataMax; Stats.Vminf = dataMin; 
	swingCenter = (3 * (dataMin + dataMax)) / 2;   // calculate wave center value

	for (int i = 1; i < NUM_ADC - 2; i++)        // scan all over the buffer
	{
		if (posiSerch == 1)    // posi slope (frequency serch)
		{
			if ((sum3(i) <= swingCenter) && (sum3(i+1)> swingCenter))   // if across the center when rising (+-3data used to eliminate noize)
			{
				pFine = (float)(swingCenter - sum3(i)) / ((swingCenter - sum3(i)) + (sum3(i+1) - swingCenter) );  // fine cross point calc.
				if (a0Detected == 0)               // if 1-st cross
				{ 
					a0Detected = 1;                     // set find flag
					p0 = i + pFine;                        // save this position as startposition
				}
				else
				{
					p1 = i + pFine - p0;                   // record length (length of n*cycle time)
					p1Count++;
				}
				lastPosiEdge = i + pFine;                // record location for Pw calcration
				posiSerch = 0;
			}
		}
		else   // nega slope serch (duration serch)
		{
			if ((sum3(i) >= swingCenter) && (sum3(i + 1) < swingCenter))   // if across the center when falling (+-3data used to eliminate noize)
			{
				pFine = (float)(sum3(i) - swingCenter) / ((sum3(i) - swingCenter) + (swingCenter - sum3(i + 1)) );
				if (a0Detected == 1) 
				{
					p2 = p2 + (i + pFine - lastPosiEdge);  // calucurate pulse width and accumurate it
					p2Count++;
				}
				posiSerch = 1;
			}
		}
	}

	pPeriod = p1 / p1Count;                 // pulse period
	pWidth  = p2 / p2Count;                  // palse width
	trigger_extra_point=pWidth/2;

	Stats.freq = 1.0 / ((hRangeValue[hRange] * pPeriod) / 20.0)+0.5; // frequency
	Stats.duty = 1000.0 * pWidth / pPeriod;  
}

void DRAW_GRIDS(void)
{
	ST7789_DrawFastHLine(0, 239,  20, TRACE);  //   -----
	ST7789_DrawFastHLine(0, 239,  40, TRACE);
	ST7789_DrawFastHLine(0, 239,  60, TRACE);
	ST7789_DrawFastHLine(0, 239,  80, TRACE);
	ST7789_DrawFastHLine(0, 239, 100, TRACE);
	ST7789_DrawFastHLine(0, 239, 120, TRACE_CTR);
	ST7789_DrawFastHLine(0, 239, 140, TRACE);
	ST7789_DrawFastHLine(0, 239, 160, TRACE);
	ST7789_DrawFastHLine(0, 239, 180, TRACE);
	ST7789_DrawFastHLine(0, 239, 200, TRACE);
	ST7789_DrawFastHLine(0, 239, 220, TRACE);
	
	ST7789_DrawFastVLine(40,  20, 219, TRACE);  //     | | | |
	ST7789_DrawFastVLine(80,  20, 219, TRACE);
	ST7789_DrawFastVLine(120, 20, 219, TRACE_CTR);
	ST7789_DrawFastVLine(160, 20, 219, TRACE);
	ST7789_DrawFastVLine(200, 20, 219, TRACE);
	ST7789_DrawFastVLine(20,  20, 219, TRACE);
	ST7789_DrawFastVLine(60,  20, 219, TRACE);
	ST7789_DrawFastVLine(100, 20, 219, TRACE);
	ST7789_DrawFastVLine(140, 20, 219, TRACE);
	ST7789_DrawFastVLine(180, 20, 219, TRACE);
	ST7789_DrawFastVLine(220, 20, 219, TRACE);
	

	if (old_trigger_pos_X>=20 && old_trigger_pos_X<=219)
	{
		for (int x=0; x<240; x=x+4)
		{
			ST7789_DrawFastHLine(x, x+1, old_trigger_pos_X, BLACK);  //------------
		}
	}
	if (trigger_pos>=20 && trigger_pos<=219)
	{
		for (int x=0; x<240; x=x+4)
		{
			ST7789_DrawFastHLine(x, x+1, trigger_pos, GRAY);  //------------
		}
	}
	old_trigger_pos_X = trigger_pos;

	trigger_pos_drawing=trigger_pos_Y;
	for (int x=20; x<219; x=x+4)
	{
		ST7789_DrawFastVLine(old_trigger_pos_Y, x, x+1, BLACK);  // |
		ST7789_DrawFastVLine(trigger_pos_drawing, x, x+1, GRAY); // |
	}
	old_trigger_pos_Y = trigger_pos_drawing;
}

void DRAW_HOLD_MASK(void)
{
	ST7789_DrawFastVLine(0  ,  21 , 30 , RED);
	ST7789_DrawFastVLine(239,  21 , 30 , RED);
	ST7789_DrawFastVLine(0  ,  210, 219, RED);
	ST7789_DrawFastVLine(239,  210, 219, RED);
	ST7789_DrawFastHLine(0  ,  9  , 21 , RED);
	ST7789_DrawFastHLine(230,  239, 21 , RED);
	ST7789_DrawFastHLine(0  ,  9  , 219, RED);
	ST7789_DrawFastHLine(230,  239, 219, RED);
}

void CLEAR_HOLD_MASK(void)
{
	ST7789_DrawFastVLine(0  ,  21 , 30 , BLACK);
	ST7789_DrawFastVLine(239,  21 , 30 , BLACK);
	ST7789_DrawFastVLine(0  ,  210, 219, BLACK);
	ST7789_DrawFastVLine(239,  210, 219, BLACK);
	ST7789_DrawFastHLine(0  ,  9  , 21 , BLACK);
	ST7789_DrawFastHLine(230,  239, 21 , BLACK);
	ST7789_DrawFastHLine(0  ,  9  , 219, BLACK);
	ST7789_DrawFastHLine(230,  239, 219, BLACK);
}

void DRAW_STATS_WAVE(void)
{
	if (Stats_source==1) Stats_color = MAGENTA; else Stats_color = GREEN;
	//Vpp:
	int vpp;
	if ((Stats_source==1 && vRangeA>3)||(Stats_source==2 && vRangeB>3))
			 vpp = (int)(Stats.Vmaxf-Stats.Vminf)*3.3/(40.95*0.03022);//   vpp = (Stats.Vmaxf-Stats.Vminf)*3.3/(40.95*gain);
	else
			 vpp = (int)(Stats.Vmaxf-Stats.Vminf)*3.3/(40.95*0.25);   //   vpp = (Stats.Vmaxf-Stats.Vminf)*3.3/(40.95*gain);
	if (vpp<1000)
	{
		ST7789_WriteString(33, 230, ".",  Font_7x10, Stats_color, BLACK); 
		ST7789_WriteNumber(28, 230,  vpp/100, Stats_color, BLACK); 
		ST7789_WriteNumber(39, 230, (vpp%100)/10, Stats_color, BLACK);
		ST7789_WriteNumber(46, 230, (vpp%100)%10, Stats_color, BLACK);
	}
	else
	{
		ST7789_WriteString(40, 230, ".",  Font_7x10, Stats_color, BLACK); 
		ST7789_WriteNumber(28, 230, vpp/1000, Stats_color, BLACK); 
		ST7789_WriteNumber(35, 230, (vpp%1000)/100, Stats_color, BLACK);
		ST7789_WriteNumber(46, 230, (vpp%1000)%100, Stats_color, BLACK);
	}
	//duty:
	if (Stats.duty>1000) Stats.duty=1000;
	ST7789_WriteString(127, 230,  ". ",  Font_7x10, Stats_color, BLACK);
	ST7789_WriteNumber(115, 230, (int) Stats.duty/100, Stats_color, BLACK);
	ST7789_WriteNumber(122, 230, (int) (Stats.duty%100)/10, Stats_color, BLACK);
	ST7789_WriteNumber(132, 230, (int) (Stats.duty%100)%10, Stats_color, BLACK);
	ST7789_WriteString(139, 230, "%",  Font_7x10, Stats_color, BLACK);
		
	if (Stats.freq>0 && Stats.freq<999999)
	{
		int tem =Stats.freq/1; uint8_t first=0;int dec=0;
		dec=tem/100000; if (dec>0) first=1; if (first) ST7789_WriteNumber(197, 230, dec, Stats_color, BLACK); else ST7789_WriteString(197, 230, " ", Font_7x10, Stats_color, BLACK); tem = tem%100000;
		dec=tem/10000;  if (dec>0) first=1; if (first) ST7789_WriteNumber(204, 230, dec, Stats_color, BLACK); else ST7789_WriteString(204, 230, " ", Font_7x10, Stats_color, BLACK); tem = tem%10000;
		dec=tem/1000;   if (dec>0) first=1; if (first) ST7789_WriteNumber(211, 230, dec, Stats_color, BLACK); else ST7789_WriteString(211, 230, " ", Font_7x10, Stats_color, BLACK); tem = tem%1000;
		dec=tem/100;    if (dec>0) first=1; if (first) ST7789_WriteNumber(218, 230, dec, Stats_color, BLACK); else ST7789_WriteString(218, 230, " ", Font_7x10, Stats_color, BLACK); tem = tem%100;
		dec=tem/10;     if (dec>0) first=1; if (first) ST7789_WriteNumber(225, 230, dec, Stats_color, BLACK); else ST7789_WriteString(225, 230, " ", Font_7x10, Stats_color, BLACK); tem = tem%10;
		dec=tem/1;      if (dec>0) first=1; if (first) ST7789_WriteNumber(232, 230, dec, Stats_color, BLACK); else ST7789_WriteString(232, 230, "-", Font_7x10, Stats_color, BLACK); 
	}
	else ST7789_WriteString(197, 230, "      ", Font_7x10, Stats_color, BLACK);
}
void START_ADC_AGAIN(void)
{
	//Start ADC sample:
	if (conversion_ready==1)
	{
		if(vRangeA>3) vRangeA_High; else vRangeA_Low;
		if(vRangeB>3) vRangeB_High; else vRangeB_Low;
		if(XY_MODE==0)
		{
			if(hRange>7) NUM_ADC_new=2048; else if(hRange>4) NUM_ADC_new=1280; else NUM_ADC_new=1024;
		}
		else
		{
			switch (Legh_data)
			{
				case 1:
				{
					NUM_ADC_new=256;break;
				}
				case 2:
				{
					NUM_ADC_new=512;break;
				}
				case 3:
				{
					NUM_ADC_new=768;break;
				}
				case 4:
				{
					NUM_ADC_new=1024;break;
				}
				case 5:
				{
					NUM_ADC_new=1280;break;
				}
				case 6:
				{
					NUM_ADC_new=1536;break;
				}
				case 7:
				{
					NUM_ADC_new=1792;break;
				}
				case 8:
				{
					NUM_ADC_new=2048;break;
				}
			}
		}
		if (Hold==0)
		{
			conversion_ready = 0;												//start next sequence of conversion
			LL_DMA_ClearFlag_TE1(DMA1);											//just in case...
			LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);						//disable DMA so that it can be
			LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, NUM_ADC_new);			//set to the number of transfers
			LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);						//re-enable DMA
			LL_TIM_EnableCounter(TIM3);											//and THEN start TIM3 --> triggers the ADCs
			GPIOC->BSRR |= (1<<13)<<16;  //LED_ON
		}
	}
}
void DRAW_WAVES(void)
{
	
	int j = trigger_point;//sIndex + xCursorSnap;
	if(j >= NUM_ADC) j = j - NUM_ADC;
	val1 =0; val2 =0;
	if(wavesOld[A1])
	{
			plotLineSegment(ch1Old[0], ch1Old[1], 0, BLACK);
	}
	if(wavesOld[A2])
	{
			plotLineSegment(ch2Old[0], ch2Old[1], 0, BLACK);
	}

	for(int i = 0, jn = j + 1;       i < GRID_WIDTH;      j=j+1, i++, jn=jn+1)	//run from 0 -> 239+1
	{
		if(jn >= NUM_ADC) jn = 0;
		if(j  >= NUM_ADC) j  = 0;
		// erase old line segment 
		if(wavesOld[A1])
		{
			plotLineSegment(ch1Old[i+1], ch1Old[i + 2], i+1, BLACK);
		}
		// erase old line segment 
		if(wavesOld[A2])
		{
			plotLineSegment(ch2Old[i+1], ch2Old[i + 2], i+1, BLACK);
		}
			

		if (trigger_source==1)
		{
			// draw new segments
			if(wavesSnap[A2])
			{
				if (hRange<13)
				{
					transposedPt1 = GRID_HEIGHT + vOffset + yCursorsSnap[A2] - (((ch2Capture[j] & 0x0FFF) - zeroVoltageA2) * GRID_HEIGHT)/ADC_2_GRID_B; //120 - (((4096 & 12bit) - 2048) * 200)/4096 =  100
					transposedPt2 = GRID_HEIGHT + vOffset + yCursorsSnap[A2] - (((ch2Capture[jn]& 0x0FFF) - zeroVoltageA2) * GRID_HEIGHT)/ADC_2_GRID_B; // (((   0 & 12bit) - 2048) * 200)/4096 = -100
				}
				else
				{
					transposedPt1 = GRID_HEIGHT + vOffset + yCursorsSnap[A2] - (((ch2Capture_zoom[i ] & 0x0FFF) - zeroVoltageA2) * GRID_HEIGHT)/ADC_2_GRID_B; //120 - (((4096 & 12bit) - 2048) * 200)/4096 =  100
					transposedPt2 = GRID_HEIGHT + vOffset + yCursorsSnap[A2] - (((ch2Capture_zoom[i+1]& 0x0FFF) - zeroVoltageA2) * GRID_HEIGHT)/ADC_2_GRID_B; // (((   0 & 12bit) - 2048) * 200)/4096 = -100
				}
				if (transposedPt1>220) transposedPt1=220;if (transposedPt2>220) transposedPt2=220;if (transposedPt1<20) transposedPt1=20;if (transposedPt2<20) transposedPt2=20;
				ch2Old[i] = transposedPt1;
				if (i==239) ch2Old[i+1] = transposedPt2;
				// draw the line segment:
				plotLineSegment(transposedPt1, transposedPt2, i, GREEN);
			}
			// draw new segments
			if(wavesSnap[A1])
			{
				if (hRange<13)
				{
					transposedPt1 = GRID_HEIGHT + vOffset + yCursorsSnap[A1] - (((ch1Capture[j] & 0x0FFF) - zeroVoltageA1) * GRID_HEIGHT)/ADC_2_GRID_A; //120 - (((4096 & 12bit) - 2048) * 200)/4096 =  100
					transposedPt2 = GRID_HEIGHT + vOffset + yCursorsSnap[A1] - (((ch1Capture[jn]& 0x0FFF) - zeroVoltageA1) * GRID_HEIGHT)/ADC_2_GRID_A; // (((   0 & 12bit) - 2048) * 200)/4096 = -100
				}
				else
				{
					transposedPt1 = GRID_HEIGHT + vOffset + yCursorsSnap[A1] - (((ch1Capture_zoom[i ] & 0x0FFF) - zeroVoltageA1) * GRID_HEIGHT)/ADC_2_GRID_A; //120 - (((4096 & 12bit) - 2048) * 200)/4096 =  100
					transposedPt2 = GRID_HEIGHT + vOffset + yCursorsSnap[A1] - (((ch1Capture_zoom[i+1]& 0x0FFF) - zeroVoltageA1) * GRID_HEIGHT)/ADC_2_GRID_A; // (((   0 & 12bit) - 2048) * 200)/4096 = -100
				}
				if (transposedPt1>220) transposedPt1=220;if (transposedPt2>220) transposedPt2=220;if (transposedPt1<20) transposedPt1=20;if (transposedPt2<20) transposedPt2=20;
				ch1Old[i] = transposedPt1;
				if (i==239) ch1Old[i+1] = transposedPt2;
				// draw the line segment:
				plotLineSegment(transposedPt1, transposedPt2, i, MAGENTA); 
			}
		}
		else
		{
			// draw new segments
			if(wavesSnap[A1])
			{
				if (hRange<13)
				{
					transposedPt1 = GRID_HEIGHT + vOffset + yCursorsSnap[A1] - (((ch1Capture[j] & 0x0FFF) - zeroVoltageA1) * GRID_HEIGHT)/ADC_2_GRID_A; //120 - (((4096 & 12bit) - 2048) * 200)/4096 =  100
					transposedPt2 = GRID_HEIGHT + vOffset + yCursorsSnap[A1] - (((ch1Capture[jn]& 0x0FFF) - zeroVoltageA1) * GRID_HEIGHT)/ADC_2_GRID_A; // (((   0 & 12bit) - 2048) * 200)/4096 = -100
				}
				else
				{
					transposedPt1 = GRID_HEIGHT + vOffset + yCursorsSnap[A1] - (((ch1Capture_zoom[i ] & 0x0FFF) - zeroVoltageA1) * GRID_HEIGHT)/ADC_2_GRID_A; //120 - (((4096 & 12bit) - 2048) * 200)/4096 =  100
					transposedPt2 = GRID_HEIGHT + vOffset + yCursorsSnap[A1] - (((ch1Capture_zoom[i+1]& 0x0FFF) - zeroVoltageA1) * GRID_HEIGHT)/ADC_2_GRID_A; // (((   0 & 12bit) - 2048) * 200)/4096 = -100
				}
				if (transposedPt1>220) transposedPt1=220;if (transposedPt2>220) transposedPt2=220;if (transposedPt1<20) transposedPt1=20;if (transposedPt2<20) transposedPt2=20;
				ch1Old[i] = transposedPt1;
				if (i==239) ch1Old[i+1] = transposedPt2;
				// draw the line segment:
				plotLineSegment(transposedPt1, transposedPt2, i, MAGENTA); 
			}
			// draw new segments
			if(wavesSnap[A2])
			{
				if (hRange<13)
				{
					transposedPt1 = GRID_HEIGHT + vOffset + yCursorsSnap[A2] - (((ch2Capture[j] & 0x0FFF) - zeroVoltageA2) * GRID_HEIGHT)/ADC_2_GRID_B; //120 - (((4096 & 12bit) - 2048) * 200)/4096 =  100
					transposedPt2 = GRID_HEIGHT + vOffset + yCursorsSnap[A2] - (((ch2Capture[jn]& 0x0FFF) - zeroVoltageA2) * GRID_HEIGHT)/ADC_2_GRID_B; // (((   0 & 12bit) - 2048) * 200)/4096 = -100
				}
				else
				{
					transposedPt1 = GRID_HEIGHT + vOffset + yCursorsSnap[A2] - (((ch2Capture_zoom[i ] & 0x0FFF) - zeroVoltageA2) * GRID_HEIGHT)/ADC_2_GRID_B; //120 - (((4096 & 12bit) - 2048) * 200)/4096 =  100
					transposedPt2 = GRID_HEIGHT + vOffset + yCursorsSnap[A2] - (((ch2Capture_zoom[i+1]& 0x0FFF) - zeroVoltageA2) * GRID_HEIGHT)/ADC_2_GRID_B; // (((   0 & 12bit) - 2048) * 200)/4096 = -100
				}
				if (transposedPt1>220) transposedPt1=220;if (transposedPt2>220) transposedPt2=220;if (transposedPt1<20) transposedPt1=20;if (transposedPt2<20) transposedPt2=20;
				ch2Old[i] = transposedPt1;
				if (i==239) ch2Old[i+1] = transposedPt2;
				// draw the line segment:
				plotLineSegment(transposedPt1, transposedPt2, i, GREEN);
			}
		}
	}
	// store the drawn parameters to old storage


	
	yCursorsOld[A1] = yCursorsSnap[A1];
	yCursorsOld[A2] = yCursorsSnap[A2];
	
	ADC_2_GRID_OLD_A = ADC_2_GRID_A;
	ADC_2_GRID_OLD_B = ADC_2_GRID_B;
}
void DRAW_WAVES_HOLD_MODE(void)
{
	int j = trigger_point;//sIndex + xCursorSnap;
	if(j >= NUM_ADC) j = j - NUM_ADC;
	val1 =0; val2 =0;
	if(wavesOld[A1])
	{
			plotLineSegment(ch1Old[0], ch1Old[1], 0, BLACK);
	}
	if(wavesOld[A2])
	{
			plotLineSegment(ch2Old[0], ch2Old[1], 0, BLACK);
	}
	for(int i = 0, jn = j + 1;       i < GRID_WIDTH;      j=j+1, i++, jn=jn+1)	//run from 0 -> 239+1
	{
		if(jn >= NUM_ADC) jn = 0;
		if(j  >= NUM_ADC) j  = 0;
			// erase old line segment 
			if(wavesOld[A1])
			{
				plotLineSegment(ch1Old[i+1], ch1Old[i + 2], i+1, BLACK);
			}
			if(wavesOld[A2])
			{
				plotLineSegment(ch2Old[i+1], ch2Old[i + 2], i+1, BLACK);
			}
			//draw again grid when change Show_mode in Hold:
	}
	DRAW_GRIDS();
	j = trigger_point;//sIndex + xCursorSnap;
	if(j >= NUM_ADC) j = j - NUM_ADC;
	val1 =0; val2 =0;
	for(int i = 0, jn = j + 1;       i < GRID_WIDTH;      j=j+1, i++, jn=jn+1)	//run from 0 -> 239+1
	{
		if (trigger_source==1)
		{
			// draw new segments
			if(wavesSnap[A2])
			{
				if (hRange<13)
				{
					transposedPt1 = GRID_HEIGHT + vOffset + yCursorsSnap[A2] - (((ch2Capture[j] & 0x0FFF) - zeroVoltageA2) * GRID_HEIGHT)/ADC_2_GRID_B; //120 - (((4096 & 12bit) - 2048) * 200)/4096 =  100
					transposedPt2 = GRID_HEIGHT + vOffset + yCursorsSnap[A2] - (((ch2Capture[jn]& 0x0FFF) - zeroVoltageA2) * GRID_HEIGHT)/ADC_2_GRID_B; // (((   0 & 12bit) - 2048) * 200)/4096 = -100
				}
				else
				{
					transposedPt1 = GRID_HEIGHT + vOffset + yCursorsSnap[A2] - (((ch2Capture_zoom[i ] & 0x0FFF) - zeroVoltageA2) * GRID_HEIGHT)/ADC_2_GRID_B; //120 - (((4096 & 12bit) - 2048) * 200)/4096 =  100
					transposedPt2 = GRID_HEIGHT + vOffset + yCursorsSnap[A2] - (((ch2Capture_zoom[i+1]& 0x0FFF) - zeroVoltageA2) * GRID_HEIGHT)/ADC_2_GRID_B; // (((   0 & 12bit) - 2048) * 200)/4096 = -100
				}
				if (transposedPt1>220) transposedPt1=220;if (transposedPt2>220) transposedPt2=220;if (transposedPt1<20) transposedPt1=20;if (transposedPt2<20) transposedPt2=20;
				ch2Old[i] = transposedPt1;
				if (i==239) ch2Old[i+1] = transposedPt2;
				// draw the line segment:
				plotLineSegment(transposedPt1, transposedPt2, i, GREEN);
			}
			// draw new segments
			if(wavesSnap[A1])
			{
				if (hRange<13)
				{
					transposedPt1 = GRID_HEIGHT + vOffset + yCursorsSnap[A1] - (((ch1Capture[j] & 0x0FFF) - zeroVoltageA1) * GRID_HEIGHT)/ADC_2_GRID_A; //120 - (((4096 & 12bit) - 2048) * 200)/4096 =  100
					transposedPt2 = GRID_HEIGHT + vOffset + yCursorsSnap[A1] - (((ch1Capture[jn]& 0x0FFF) - zeroVoltageA1) * GRID_HEIGHT)/ADC_2_GRID_A; // (((   0 & 12bit) - 2048) * 200)/4096 = -100
				}
				else
				{
					transposedPt1 = GRID_HEIGHT + vOffset + yCursorsSnap[A1] - (((ch1Capture_zoom[i ] & 0x0FFF) - zeroVoltageA1) * GRID_HEIGHT)/ADC_2_GRID_A; //120 - (((4096 & 12bit) - 2048) * 200)/4096 =  100
					transposedPt2 = GRID_HEIGHT + vOffset + yCursorsSnap[A1] - (((ch1Capture_zoom[i+1]& 0x0FFF) - zeroVoltageA1) * GRID_HEIGHT)/ADC_2_GRID_A; // (((   0 & 12bit) - 2048) * 200)/4096 = -100
				}
				if (transposedPt1>220) transposedPt1=220;if (transposedPt2>220) transposedPt2=220;if (transposedPt1<20) transposedPt1=20;if (transposedPt2<20) transposedPt2=20;
				ch1Old[i] = transposedPt1;
				if (i==239) ch1Old[i+1] = transposedPt2;
				// draw the line segment:
				plotLineSegment(transposedPt1, transposedPt2, i, MAGENTA); 
			}
		}
		else
		{
			// draw new segments
			if(wavesSnap[A1])
			{
				if (hRange<13)
				{
					transposedPt1 = GRID_HEIGHT + vOffset + yCursorsSnap[A1] - (((ch1Capture[j] & 0x0FFF) - zeroVoltageA1) * GRID_HEIGHT)/ADC_2_GRID_A; //120 - (((4096 & 12bit) - 2048) * 200)/4096 =  100
					transposedPt2 = GRID_HEIGHT + vOffset + yCursorsSnap[A1] - (((ch1Capture[jn]& 0x0FFF) - zeroVoltageA1) * GRID_HEIGHT)/ADC_2_GRID_A; // (((   0 & 12bit) - 2048) * 200)/4096 = -100
				}
				else
				{
					transposedPt1 = GRID_HEIGHT + vOffset + yCursorsSnap[A1] - (((ch1Capture_zoom[i ] & 0x0FFF) - zeroVoltageA1) * GRID_HEIGHT)/ADC_2_GRID_A; //120 - (((4096 & 12bit) - 2048) * 200)/4096 =  100
					transposedPt2 = GRID_HEIGHT + vOffset + yCursorsSnap[A1] - (((ch1Capture_zoom[i+1]& 0x0FFF) - zeroVoltageA1) * GRID_HEIGHT)/ADC_2_GRID_A; // (((   0 & 12bit) - 2048) * 200)/4096 = -100
				}
				if (transposedPt1>220) transposedPt1=220;if (transposedPt2>220) transposedPt2=220;if (transposedPt1<20) transposedPt1=20;if (transposedPt2<20) transposedPt2=20;
				ch1Old[i] = transposedPt1;
				if (i==239) ch1Old[i+1] = transposedPt2;
				// draw the line segment:
				plotLineSegment(transposedPt1, transposedPt2, i, MAGENTA); 
			}
			// draw new segments
			if(wavesSnap[A2])
			{
				if (hRange<13)
				{
					transposedPt1 = GRID_HEIGHT + vOffset + yCursorsSnap[A2] - (((ch2Capture[j] & 0x0FFF) - zeroVoltageA2) * GRID_HEIGHT)/ADC_2_GRID_B; //120 - (((4096 & 12bit) - 2048) * 200)/4096 =  100
					transposedPt2 = GRID_HEIGHT + vOffset + yCursorsSnap[A2] - (((ch2Capture[jn]& 0x0FFF) - zeroVoltageA2) * GRID_HEIGHT)/ADC_2_GRID_B; // (((   0 & 12bit) - 2048) * 200)/4096 = -100
				}
				else
				{
					transposedPt1 = GRID_HEIGHT + vOffset + yCursorsSnap[A2] - (((ch2Capture_zoom[i ] & 0x0FFF) - zeroVoltageA2) * GRID_HEIGHT)/ADC_2_GRID_B; //120 - (((4096 & 12bit) - 2048) * 200)/4096 =  100
					transposedPt2 = GRID_HEIGHT + vOffset + yCursorsSnap[A2] - (((ch2Capture_zoom[i+1]& 0x0FFF) - zeroVoltageA2) * GRID_HEIGHT)/ADC_2_GRID_B; // (((   0 & 12bit) - 2048) * 200)/4096 = -100
				}
				if (transposedPt1>220) transposedPt1=220;if (transposedPt2>220) transposedPt2=220;if (transposedPt1<20) transposedPt1=20;if (transposedPt2<20) transposedPt2=20;
				ch2Old[i] = transposedPt1;
				if (i==239) ch2Old[i+1] = transposedPt2;
				// draw the line segment:
				plotLineSegment(transposedPt1, transposedPt2, i, GREEN);
			}
		}
	}
	// store the drawn parameters to old storage

	yCursorsOld[A1] = yCursorsSnap[A1];
	yCursorsOld[A2] = yCursorsSnap[A2];
	
	ADC_2_GRID_OLD_A = ADC_2_GRID_A;
	ADC_2_GRID_OLD_B = ADC_2_GRID_B;
}
void ZOOM_DATA(void)
{
	if (hRange==13)  //zoom x2
	{
		uint16_t pos = trigger_pos_Y/2;
		uint16_t ch1[120+2], ch2[120+2];
		for (int n=0; n<122; n++)  //     trigger_pos_Y -> 120
		{
			ch1[n] = ch1Capture[i-pos+n];
			ch2[n] = ch2Capture[i-pos+n];
		}
		if (trigger_pos_Y%2) pos = i-trigger_pos_Y-1; else pos = i-trigger_pos_Y;
		for (int n=0; n<121; n++)  //  0 -> trigger_pos_Y
		{
			uint32_t tem = ch1[n] + ch1[n+1];
			ch1Capture_zoom[n*2] = ch1[n]; ch1Capture_zoom[n*2+1] = tem/2;

			tem = ch2[n] + ch2[n+1];
			ch2Capture_zoom[n*2] = ch2[n]; ch2Capture_zoom[n*2+1] = tem/2;
		}
	}
	else if (hRange==14) //zoom x4
	{
		uint16_t pos = trigger_pos_Y/4; if ( trigger_pos_Y%4 > 0) pos++;
		uint16_t ch1[60+1], ch2[60+1];
		for (int n=0; n<62; n++)
		{
			ch1[n] = ch1Capture[i-pos+n];
			ch2[n] = ch2Capture[i-pos+n];
		}
		if (trigger_pos_Y%4) pos = i-trigger_pos_Y+trigger_pos_Y%4-4; else pos = i-trigger_pos_Y;
		for (int n=0; n<60+1; n++)
		{
			uint32_t tem=0;

			ch1Capture_zoom[n*4  ] = ch1[n];
			tem = ch1[n] + ch1[n+1]; tem=tem/2;
			ch1Capture_zoom[n*4+2] = tem;
			if (ch1[n]<ch1[n+1])
			{
				tem = (ch1[n+1]-ch1[n])/4;
				ch1Capture_zoom[n*4+1] = ch1[n  ] + tem;
				ch1Capture_zoom[n*4+3] = ch1[n+1] - tem;
			}
			else
			{
				tem = (ch1[n]-ch1[n+1])/4;
				ch1Capture_zoom[n*4+1] = ch1[n  ] - tem;
				ch1Capture_zoom[n*4+3] = ch1[n+1] + tem;
			}
			

			ch2Capture_zoom[n*4  ] = ch2[n];
			tem = ch2[n] + ch2[n+1]; tem=tem/2;
			ch2Capture_zoom[n*4+2] = tem;
			if (ch2[n]<ch2[n+1])
			{
				tem = (ch2[n+1]-ch2[n])/4;
				ch2Capture_zoom[n*4+1] = ch2[n  ] + tem;
				ch2Capture_zoom[n*4+3] = ch2[n+1] - tem;
			}
			else
			{
				tem = (ch2[n]-ch2[n+1])/4;
				ch2Capture_zoom[n*4+1] = ch2[n  ] - tem;
				ch2Capture_zoom[n*4+3] = ch2[n+1] + tem;
			}
		}
	}
}
void DRAW_WAVE_POS_ICON(void)
{
	if(wavesOld[A1])
	{
		ST7789_DrawFastVLine(0,old_zeropos_A1-2,old_zeropos_A1+2, BLACK);
		ST7789_DrawFastVLine(1,old_zeropos_A1-2,old_zeropos_A1+2, BLACK);
		ST7789_DrawFastVLine(2,old_zeropos_A1-1,old_zeropos_A1+1, BLACK);
		ST7789_DrawPixel(3, old_zeropos_A1, BLACK);
	}
	if(wavesOld[A2])
	{
		ST7789_DrawFastVLine(239,old_zeropos_A2-2,old_zeropos_A2+2, BLACK);
		ST7789_DrawFastVLine(238,old_zeropos_A2-2,old_zeropos_A2+2, BLACK);
		ST7789_DrawFastVLine(237,old_zeropos_A2-1,old_zeropos_A2+1, BLACK);
		ST7789_DrawPixel(236, old_zeropos_A2, BLACK);
	}
	if (show_mode<3)
	{
		val1 = GRID_HEIGHT + vOffset + yCursorsSnap[A1]; old_zeropos_A1=val1;
		if(wavesSnap[A1] && show_mode<3)
		{
			ST7789_DrawFastVLine(0,val1-2,val1+2, MAGENTA);
			ST7789_DrawFastVLine(1,val1-2,val1+2, MAGENTA);
			ST7789_DrawFastVLine(2,val1-1,val1+1, MAGENTA);
			ST7789_DrawPixel(3, val1, MAGENTA);
		}
	}
	wavesOld[A1] = wavesSnap[A1];
	if (show_mode!=2)
	{
		val2 = GRID_HEIGHT + vOffset + yCursorsSnap[A2]; old_zeropos_A2=val2;
		if(wavesSnap[A2])
		{
			ST7789_DrawFastVLine(239,val2-2,val2+2, GREEN);
			ST7789_DrawFastVLine(238,val2-2,val2+2, GREEN);
			ST7789_DrawFastVLine(237,val2-1,val2+1, GREEN);
			ST7789_DrawPixel(236, val2, GREEN);
		}
	}
	wavesOld[A2] = wavesSnap[A2];
}

void XY_DRAW(void)
{
	if (NUM_ADC_old>NUM_ADC)
	{
		for(int i = NUM_ADC-1; i < NUM_ADC_old; i++)
		{
				ST7789_DrawPixel(ch1Capture[i], ch2Capture[i], BLACK);
		}
	}
	for(int i = 0; i < NUM_ADC; i++)
	{
			ST7789_DrawPixel(ch1Capture[i], ch2Capture[i], BLACK);
			if (hRange<13)
			{
				val1 = (uint16_t) ((adc_buffer[i] & 0x0000FFFF));
				val2 = (uint16_t) ( adc_buffer[i] >> 16);
				transposedPt1 = GRID_HEIGHT + vOffset + yCursorsSnap[A1] - ((val1 - zeroVoltageA1) * GRID_HEIGHT)/ADC_2_GRID_A; // (((4096 & 12bit) - 2048) * 200)/4096 =  100
				transposedPt2 = GRID_HEIGHT + vOffset + yCursorsSnap[A2] + ((val2 - zeroVoltageA2) * GRID_HEIGHT)/ADC_2_GRID_B; // (((   0 & 12bit) - 2048) * 200)/4096 = -100
				ch1Capture[i] = transposedPt1;ch2Capture[i] = transposedPt2;
				ST7789_DrawPixel(transposedPt1, transposedPt2, GREEN);
			}
	 }
	NUM_ADC_old = NUM_ADC;
	 
	yCursorsOld[A1] = yCursorsSnap[A1];
	yCursorsOld[A2] = yCursorsSnap[A2];
	
	ADC_2_GRID_OLD_A = ADC_2_GRID_A;
	ADC_2_GRID_OLD_B = ADC_2_GRID_B;
	 
	
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
