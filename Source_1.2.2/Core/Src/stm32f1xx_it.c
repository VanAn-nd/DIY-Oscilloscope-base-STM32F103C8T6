/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define GRID_HEIGHT			((uint16_t)200) //210
#define A1 					0
#define A2 					1
extern volatile uint8_t MENU; //1-2-3-4-5
extern volatile uint8_t position_wave; // A-B
extern volatile int16_t trigger_pos_X;
extern volatile uint8_t trigger_pos_Y;
extern volatile int16_t trigger_pos;
volatile float heso_k[7] = {0.1,0.2,0.5,1.0,2.0,5.0,10.0};
volatile int16_t timeout;
volatile uint16_t count=0;
volatile uint8_t SELECT_Bt_Hold=0;
extern volatile int8_t conversion_ready;
extern uint8_t show_mode; // A-B
extern volatile int hRange;											//0, 1
extern volatile int vRangeA;
extern volatile int vRangeB;
extern volatile uint8_t vOffset;
extern volatile int16_t yCursors[2];
extern volatile uint8_t  Hold;
extern volatile uint8_t  Bt_pressed;
extern volatile uint8_t  data_flag;
extern volatile uint8_t  Stats_source;
extern volatile uint8_t trigger_source; //1-4;
extern volatile uint8_t XY_MODE;
extern volatile uint8_t Legh_data;  //xy mode
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_spi1_tx;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	
	if (timeout > 0) 
	{
		timeout--;			//250ms										//when enabled: every 1 ms
		if(SELECT_Bt_Hold==1 && (GPIOB->IDR &(1<<12)))  //short SELECT pressed
		{
			if (MENU<6) MENU++; else MENU=1;
			if (MENU==1)
			{
				if (show_mode>=3)  {MENU=2;Stats_source=2;}
			}
			if (MENU==2)
			{
				if (show_mode==2) {MENU=3;Stats_source=1;}
			}
			if (MENU==3)
			{
				if(XY_MODE)MENU=7;
				else
				{
					if (show_mode==2) position_wave=0;
					if (show_mode==3) position_wave=1;
				}
			}
			SELECT_Bt_Hold=0;//SELECT butom
			Bt_pressed=1;
			count=30;
		}
	}
	else //timeout=0
	{
		if (count>0)
		{
			count--;     //300ms   hold button
			if(SELECT_Bt_Hold==1 && (GPIOB->IDR &(1<<12))) //short SELECT pressed
			{
				if (MENU<6) MENU++; else MENU=1;
				if (MENU==1)
				{
					if (show_mode>=3)  {MENU=2;Stats_source=2;}
				}
				if (MENU==2)
				{
					if (show_mode==2) {MENU=3;Stats_source=1;}
				}
				if (MENU==3)
				{
					if(XY_MODE)MENU=7;
					else
					{
						if (show_mode==2) position_wave=0;
						if (show_mode==3) position_wave=1;
					}
				}
				SELECT_Bt_Hold=0;//SELECT butom
				Bt_pressed=1;
				count=30;
			}
		}
		else  //count=0
		{
			if(!(GPIOA->IDR &(1<<6))) //left butom
			{
				if (MENU==6) //Trigger position
				{
					if(trigger_pos_Y > 0)  trigger_pos_Y-=1;
				}
				Bt_pressed=1;
			}
			else if(!(GPIOA->IDR &(1<<7)))  //right butom
			{
				if (MENU==6) //Trigger position
				{
					if(trigger_pos_Y < 239)  trigger_pos_Y+=1;
				}
				Bt_pressed=1;
			}
			else if(!(GPIOA->IDR &(1<<9)))  //down butom
			{
				if (MENU==3)
				{
					if (yCursors[position_wave]<0) yCursors[position_wave]+=1;
				}
				else if (MENU==6)
				{
					if (trigger_source%2==1) trigger_pos = GRID_HEIGHT + vOffset + yCursors[A1] - (trigger_pos_X-120);
					else                     trigger_pos = GRID_HEIGHT + vOffset + yCursors[A2] - (trigger_pos_X-120);
					if(trigger_pos<219)  trigger_pos_X-=1;
				}
				Bt_pressed=1;
			}
			else if(!(GPIOA->IDR &(1<<10)))   //up butom
			{
				if (MENU==3)
				{
					if (yCursors[position_wave]>-200) yCursors[position_wave]-=1;
				}
				else if (MENU==6)  //show: A / B / A+B
				{
					if (trigger_source%2==1) trigger_pos = GRID_HEIGHT + vOffset + yCursors[A1] - (trigger_pos_X-120);
					else                     trigger_pos = GRID_HEIGHT + vOffset + yCursors[A2] - (trigger_pos_X-120);
					if(trigger_pos>20)  trigger_pos_X+=1;
				}
				Bt_pressed=1;
			}
			else if(!(GPIOB->IDR &(1<<12))) 
			{
				if(SELECT_Bt_Hold==1)  //long SELECT pressed
				{
					if(XY_MODE==0) XY_MODE=1; else XY_MODE=0;
					SELECT_Bt_Hold=0;Hold=0;
				}
				Bt_pressed=1;
			}
			else
			{
				LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_6);  //LEFT
				LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_7);  //RIGHT
				LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_9);	//DOWN
				LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_10); //UP
				LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_11); //HOLD
				LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_12); //SELECT
				LL_SYSTICK_DisableIT();													//disable THIS interrupt
				data_flag=1;
			}
			count=30;
		}
	}
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
	LL_TIM_DisableCounter(TIM3);												//so no more triggering of the ADCs
	if (LL_DMA_IsActiveFlag_TC1(DMA1))
	{
		while(LL_DMA_IsActiveFlag_TC1(DMA1)) LL_DMA_ClearFlag_TC1(DMA1);
		conversion_ready = 1;
		GPIOC->BSRR |= (1<<13);
	}
  /* USER CODE END DMA1_Channel1_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_6) != RESET)
  {
    /* USER CODE BEGIN LL_EXTI_LINE_6 */
		LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_6);
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_6);
		if (MENU==1 || MENU==2 || MENU==7)
		{
			if (hRange>0) hRange--;  //0->14 timebase
			switch(hRange)													//set TIM3 to required timebase setting
			{
			case 0:
				LL_TIM_SetPrescaler(TIM3, 99);
				LL_TIM_SetAutoReload(TIM3, 7999);								// 200ms/div
				break;
			case 1:
				LL_TIM_SetPrescaler(TIM3, 99);
				LL_TIM_SetAutoReload(TIM3, 3999);								// 100ms/div
				break;
			case 2:
				LL_TIM_SetPrescaler(TIM3, 99);
				LL_TIM_SetAutoReload(TIM3, 1999);								// 50ms/div
				break;
			case 3:
				LL_TIM_SetPrescaler(TIM3, 99);
				LL_TIM_SetAutoReload(TIM3, 799);								// 20ms/div
				break;
			case 4:
				LL_TIM_SetPrescaler(TIM3, 0);
				LL_TIM_SetAutoReload(TIM3, 39999);								// 10ms/div
				break;
			case 5:
				LL_TIM_SetPrescaler(TIM3, 0);
				LL_TIM_SetAutoReload(TIM3, 19999);								// 5ms/div
				break;
			case 6:
				LL_TIM_SetPrescaler(TIM3, 0);
				LL_TIM_SetAutoReload(TIM3, 7999);								// 2ms/div
				break;
			case 7:
				LL_TIM_SetPrescaler(TIM3, 0);
				LL_TIM_SetAutoReload(TIM3, 3999);								// 1ms/div
				break;
			case 8:
				LL_TIM_SetPrescaler(TIM3, 0);
				LL_TIM_SetAutoReload(TIM3, 1999);								// 500us/div
				break;
			case 9:
				LL_TIM_SetPrescaler(TIM3, 0);
				LL_TIM_SetAutoReload(TIM3, 799);								// 200us/div
				break;
			case 10:
				LL_TIM_SetPrescaler(TIM3, 0);
				LL_TIM_SetAutoReload(TIM3, 399);								// 100us/div
				break;
			case 11:
				LL_TIM_SetPrescaler(TIM3, 0);
				LL_TIM_SetAutoReload(TIM3, 199);								// 50us/div
				break;
			case 12:
				LL_TIM_SetPrescaler(TIM3, 0);
				LL_TIM_SetAutoReload(TIM3, 79);									// 20us/div ADC at 1Msps
				break;
			case 13:
				LL_TIM_SetPrescaler(TIM3, 0);
				LL_TIM_SetAutoReload(TIM3, 79);									// 10us/div ADC at 1Msps (x2 increase sampling rate by code).
				break;
			case 14:
				LL_TIM_SetPrescaler(TIM3, 0);
				LL_TIM_SetAutoReload(TIM3, 79);									// 5us/div ADC at 1Msps (x4 increase sampling rate by code).
				break;
			default:															//should never happen
				LL_TIM_SetPrescaler(TIM3, 0);
				LL_TIM_SetAutoReload(TIM3, 1999);
				break;
			}
			Hold=0;
		}
		else if (MENU==3)
		{
			position_wave=0;
		}
		else if (MENU==4)
		{
			if(show_mode<3) Stats_source=1;
		}
		else if (MENU==5) //Trigger mode
		{
			if (show_mode<3)
			{
				if      (trigger_source==2) {trigger_source=1;trigger_pos_X = 120+((trigger_pos_X-120)*heso_k[vRangeB]/heso_k[vRangeA]);}
				else if (trigger_source==4) {trigger_source=3;trigger_pos_X = 120+((trigger_pos_X-120)*heso_k[vRangeB]/heso_k[vRangeA]);}
			}
			else
			{
				if      (trigger_source==1) {trigger_source=2;trigger_pos_X = 120+((trigger_pos_X-120)*heso_k[vRangeA]/heso_k[vRangeB]);}   
				else if (trigger_source==3) {trigger_source=4;trigger_pos_X = 120+((trigger_pos_X-120)*heso_k[vRangeA]/heso_k[vRangeB]);}
			}
		}
		else  if (MENU==6)
		{
			if(trigger_pos_Y>0)  trigger_pos_Y--; count=300;
		}
		timeout = 300;
		LL_SYSTICK_EnableIT();
    /* USER CODE END LL_EXTI_LINE_6 */
  }
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_7) != RESET)
  {
    /* USER CODE BEGIN LL_EXTI_LINE_7 */
		LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_7);
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_7);
		if (MENU==1 || MENU==2 || MENU==7)
		{
			if (hRange<14) hRange++;   //0->14 timebase
			if(hRange>12 && XY_MODE==1) hRange=12;
			switch(hRange)													//set TIM3 to required timebase setting
			{
			case 0:
				LL_TIM_SetPrescaler(TIM3, 99);
				LL_TIM_SetAutoReload(TIM3, 7999);								// 200ms/div
				break;
			case 1:
				LL_TIM_SetPrescaler(TIM3, 99);
				LL_TIM_SetAutoReload(TIM3, 3999);								// 100ms/div
				break;
			case 2:
				LL_TIM_SetPrescaler(TIM3, 99);
				LL_TIM_SetAutoReload(TIM3, 1999);								// 50ms/div
				break;
			case 3:
				LL_TIM_SetPrescaler(TIM3, 99);
				LL_TIM_SetAutoReload(TIM3, 799);								// 20ms/div
				break;
			case 4:
				LL_TIM_SetPrescaler(TIM3, 0);
				LL_TIM_SetAutoReload(TIM3, 39999);								// 10ms/div
				break;
			case 5:
				LL_TIM_SetPrescaler(TIM3, 0);
				LL_TIM_SetAutoReload(TIM3, 19999);								// 5ms/div
				break;
			case 6:
				LL_TIM_SetPrescaler(TIM3, 0);
				LL_TIM_SetAutoReload(TIM3, 7999);								// 2ms/div
				break;
			case 7:
				LL_TIM_SetPrescaler(TIM3, 0);
				LL_TIM_SetAutoReload(TIM3, 3999);								// 1ms/div
				break;
			case 8:
				LL_TIM_SetPrescaler(TIM3, 0);
				LL_TIM_SetAutoReload(TIM3, 1999);								// 500us/div
				break;
			case 9:
				LL_TIM_SetPrescaler(TIM3, 0);
				LL_TIM_SetAutoReload(TIM3, 799);								// 200us/div
				break;
			case 10:
				LL_TIM_SetPrescaler(TIM3, 0);
				LL_TIM_SetAutoReload(TIM3, 399);								// 100us/div
				break;
			case 11:
				LL_TIM_SetPrescaler(TIM3, 0);
				LL_TIM_SetAutoReload(TIM3, 199);								// 50us/div
				break;
			case 12:
				LL_TIM_SetPrescaler(TIM3, 0);
				LL_TIM_SetAutoReload(TIM3, 79);									// 20us/div
				break;
			case 13:
				LL_TIM_SetPrescaler(TIM3, 0);
				LL_TIM_SetAutoReload(TIM3, 79);									// 10us/div 2MHz
				break;
			case 14:
				LL_TIM_SetPrescaler(TIM3, 0);
				LL_TIM_SetAutoReload(TIM3, 79);									// 5us/div  2MHz
				break;
			default:															//should never happen
				LL_TIM_SetPrescaler(TIM3, 0);
				LL_TIM_SetAutoReload(TIM3, 1999);
				break;
			}
			Hold=0;
		}
		else if (MENU==3)
		{
			position_wave=1;
		}
		else if (MENU==4)
		{
			if(show_mode!=2) Stats_source=2;
		}
		else if (MENU==5)
		{
			if (show_mode!=2)
			{
				if      (trigger_source==1) {trigger_source=2;trigger_pos_X = 120+((trigger_pos_X-120)*heso_k[vRangeA]/heso_k[vRangeB]);}
				else if (trigger_source==3) {trigger_source=4;trigger_pos_X = 120+((trigger_pos_X-120)*heso_k[vRangeA]/heso_k[vRangeB]);}
			}
			else
			{
				if      (trigger_source==2) {trigger_source=1;trigger_pos_X = 120+((trigger_pos_X-120)*heso_k[vRangeB]/heso_k[vRangeA]);}   
				else if (trigger_source==4) {trigger_source=3;trigger_pos_X = 120+((trigger_pos_X-120)*heso_k[vRangeB]/heso_k[vRangeA]);}
			}
		}
		else  if (MENU==6)
		{
			if(trigger_pos_Y<239)  trigger_pos_Y++; count=300;
		}
		timeout = 300;
		LL_SYSTICK_EnableIT();
    /* USER CODE END LL_EXTI_LINE_7 */
  }
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_9) != RESET)
  {
    /* USER CODE BEGIN LL_EXTI_LINE_9 */
		LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_9);
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_9);
		if(MENU==1)
		{
			if (vRangeA>0)
			{
				vRangeA--;  //CH1
				if (trigger_source%2==1) trigger_pos_X = 120+((trigger_pos_X-120)*heso_k[vRangeA+1]/heso_k[vRangeA]);
				Hold=0;
			}
		}
		else if(MENU==2)
		{
			if (vRangeB>0)
			{
				vRangeB--; //CH1
				if (trigger_source%2==0) trigger_pos_X = 120+((trigger_pos_X-120)*heso_k[vRangeB+1]/heso_k[vRangeB]);
				Hold=0;
			}
		}
		else if (MENU==3)
		{
			if(!XY_MODE)
			{
				if (yCursors[position_wave]<0) yCursors[position_wave]+=1; count=300;
			}else MENU=7;
		}
		else if (MENU==4)
		{
			if(!XY_MODE)
			{
				if(show_mode<3) show_mode++;
				else show_mode=1;
				if(show_mode==2) 
				{
					Stats_source=1;
					if      (trigger_source==2) {trigger_source=1;}   
					else if (trigger_source==4) {trigger_source=3;}
				}
				else if(show_mode==3) 
				{
					Stats_source=2;
					if      (trigger_source==1) {trigger_source=2;}   
					else if (trigger_source==3) {trigger_source=4;}
				}
			}else MENU=7;
		}
		else if (MENU==5)
		{
			if(!XY_MODE)
			{
				if      (trigger_source==1) trigger_source=3;
				else if (trigger_source==2) trigger_source=4;
			}else MENU=7;
		}
		else  if (MENU==6)
		{
			if(!XY_MODE)
			{
				if (trigger_source%2==1) trigger_pos = GRID_HEIGHT + vOffset + yCursors[A1] - (trigger_pos_X-120);
				else                     trigger_pos = GRID_HEIGHT + vOffset + yCursors[A2] - (trigger_pos_X-120);
				if(trigger_pos<219)  trigger_pos_X-=1; count=300;
			}else MENU=7;
		}
		if(MENU==7)
		{
			if(Legh_data>1)Legh_data--; //1->8
		}
		timeout = 300;
		LL_SYSTICK_EnableIT();
    /* USER CODE END LL_EXTI_LINE_9 */
  }
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */
	Bt_pressed=1;
  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_10) != RESET)
  {
    /* USER CODE BEGIN LL_EXTI_LINE_10 */
		LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_10);
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_10);
		if(MENU==1)
		{
			if (vRangeA<6)
			{
				vRangeA++; //CH1
				if (trigger_source%2==1) trigger_pos_X = 120+((trigger_pos_X-120)*heso_k[vRangeA-1]/heso_k[vRangeA]);
				Hold=0;
			}
		}
		else if(MENU==2)
		{
			if (vRangeB<6)
			{
				vRangeB++; //CH2
				if (trigger_source%2==0) trigger_pos_X = 120+((trigger_pos_X-120)*heso_k[vRangeB-1]/heso_k[vRangeB]);
				Hold=0;
			}
		}
		else if (MENU==3)
		{
			if(!XY_MODE)
			{
				if (yCursors[position_wave]>-200) yCursors[position_wave]-=1; count=300;
			}else MENU=7;
		}
		else if (MENU==4)  //show: A / B / A+B
		{
			if(!XY_MODE)
			{
				if(show_mode>1) show_mode--; 
				else show_mode=3;
				if(show_mode==2) 
				{
					Stats_source=1;
					if      (trigger_source==2) {trigger_source=1;}   
					else if (trigger_source==4) {trigger_source=3;}
				}
				else if(show_mode==3) 
				{
					Stats_source=2;
					if      (trigger_source==1) {trigger_source=2;}   
					else if (trigger_source==3) {trigger_source=4;}
				}
			}else MENU=7;
		}
		else if (MENU==5) //TRIGGER select
		{
			if(!XY_MODE)
			{
				if 		  (trigger_source==3) trigger_source=1;
				else if (trigger_source==4) trigger_source=2;
			}else MENU=7;
		}
		else  if (MENU==6)
		{
			if(!XY_MODE)
			{
				if (trigger_source%2==1) trigger_pos = GRID_HEIGHT + vOffset + yCursors[A1] - (trigger_pos_X-120);
				else                     trigger_pos = GRID_HEIGHT + vOffset + yCursors[A2] - (trigger_pos_X-120);
				if(trigger_pos>20)  trigger_pos_X+=1; count=300;
			}else MENU=7;
		}
		if (MENU==7)
		{
			if(Legh_data<8) Legh_data++; //1->8
		}
		timeout = 300;
		LL_SYSTICK_EnableIT();
    /* USER CODE END LL_EXTI_LINE_10 */
  }
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_11) != RESET)
  {
    /* USER CODE BEGIN LL_EXTI_LINE_11 */
		LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_11);  //HOLD
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_11);
		if (Hold==0) Hold=1; else Hold=0;
		timeout = 250;
		LL_SYSTICK_EnableIT();
    /* USER CODE END LL_EXTI_LINE_11 */
  }
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_12) != RESET)
  {
    /* USER CODE BEGIN LL_EXTI_LINE_12 */
		LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_12);  //SELECT
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_12);
		SELECT_Bt_Hold=1;
		timeout = 250;count=500;
		LL_SYSTICK_EnableIT();
    /* USER CODE END LL_EXTI_LINE_12 */
  }
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */
	Bt_pressed=1;
  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
