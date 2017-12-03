#include "stm32f10x.h"
#include "platform_config.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#define TxBufferSize   8
#define RxBufferSize   8

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
USART_InitTypeDef USART_InitStructure;
EXTI_InitTypeDef   EXTI_InitStructure;

uint16_t CCR1_Val = 0x0000;
uint16_t CCR2_Val = 0x0000;
uint16_t CCR3_Val = 0x0000;
uint16_t CCR4_Val = 0x0000;
uint16_t PrescalerValue = 0;

uint8_t TxBuffer[] = "USART Interrupt Example: USARTy using Interrupt";
uint8_t RxBuffer[RxBufferSize];
__IO uint8_t TxCounter = 0x00;
__IO uint8_t RxCounter = 0x00; 
uint8_t NbrOfDataToTransfer = TxBufferSize;
uint8_t NbrOfDataToRead = RxBufferSize;
uint8_t UsartCmdReceived = 0x00;
uint8_t ButtonFlag = 0x00;
uint8_t Button1Count = 0x00;
uint8_t Button2Count = 0x00;
uint8_t Button3Count = 0x00;
uint8_t Button4Count = 0x00;

/* Private functions ---------------------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void TIM3_Configuration(void);
void NVIC_Configuration(void);
void USART1_Configuration(void);
void EXTI0_Configuration(void);
void EXTI10_Configuration(void);
void EXTI11_Configuration(void);
void EXTI12_Configuration(void);

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
   *   this is done through SystemInit() function which is called from startup
   *   file (startup_stm32f10x_xx.s) before to branch to application main.
   *   To reconfigure the default setting of SystemInit() function, refer to
   *   system_stm32f10x.c file
   */
  
  /* System clocks configuration ---------------------------------------------*/
  RCC_Configuration();
  
  /* GPIO configuration ------------------------------------------------------*/
  GPIO_Configuration();

  /* TIM3 configuration ------------------------------------------------------*/
  TIM3_Configuration();

  /* EXTI0 configuration ----------------------------------------------------*/  
  EXTI0_Configuration();
  
  /* EXTI10 configuration ----------------------------------------------------*/  
  EXTI10_Configuration();
  
  /* EXTI11 configuration ----------------------------------------------------*/  
  EXTI11_Configuration();
  
  /* EXTI12 configuration ----------------------------------------------------*/  
  EXTI12_Configuration();    
  
  /* USART1 configuration ----------------------------------------------------*/  
  USART1_Configuration();  
  
  /* NVIC configuration ---------- -------------------------------------------*/
  NVIC_Configuration();

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
  /* Enable USART1 */
  USART_Cmd(USART1, ENABLE);
  /*----------------------------------------------------------------------- */   
  while (1)
  {
    if (UsartCmdReceived)
    {
      UsartCmdReceived = 0x00;
      RxCounter = 0x00;
      CCR1_Val = ((uint16_t)(*(RxBuffer+0))<<8)+(uint16_t)(*(RxBuffer+1));
      CCR2_Val = ((uint16_t)(*(RxBuffer+2))<<8)+(uint16_t)(*(RxBuffer+3));
      CCR3_Val = ((uint16_t)(*(RxBuffer+4))<<8)+(uint16_t)(*(RxBuffer+5));
      CCR4_Val = ((uint16_t)(*(RxBuffer+6))<<8)+(uint16_t)(*(RxBuffer+7));
      
      /* Re-configuration Timer */      
      TIM_Cmd(TIM3, DISABLE);
      TIM3_Configuration();
      TIM_Cmd(TIM3, ENABLE);
      
      /* Disable the USART1 Receive interrupt */
      USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);      
    }
    
    if (ButtonFlag)
    {
      if (Button1Count == 0)
        CCR1_Val = 0;
      else
        CCR1_Val = 2400*Button1Count-1;
      
      if (Button2Count == 0)
        CCR2_Val = 0;
      else
        CCR2_Val = 2400*Button2Count-1;

      if (Button3Count == 0)
        CCR3_Val = 0;
      else
        CCR3_Val = 2400*Button3Count-1;
      
      if (Button4Count == 0)
        CCR4_Val = 0;
      else
        CCR4_Val = 2400*Button4Count-1;

      /* Re-configuration Timer */      
      TIM_Cmd(TIM3, DISABLE);
      TIM3_Configuration();
      TIM_Cmd(TIM3, ENABLE);
      ButtonFlag = 0x00;
    }    
  }
}

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void RCC_Configuration(void)
{
  /* PCLK2 = HCLK/2 */
  RCC_PCLK2Config(RCC_HCLK_Div2);

  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* GPIOA, GPIOB, USART1 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
                         RCC_APB2Periph_GPIOC |
                         RCC_APB2Periph_AFIO  |
                         RCC_APB2Periph_USART1, ENABLE);
}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Configure USART1 Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* Configure PA.00 pin as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* Configure PC.00 pin as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  /* Configure USART1 Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);  

  /*GPIOB Configuration: TIM3 channel1, 2, 3 and 4 */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);	
}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */
void TIM3_Configuration(void)
{
  /* -----------------------------------------------------------------------
    TIM3 Configuration: generate 4 PWM signals with 4 different duty cycles:
    The TIM3CLK frequency is set to SystemCoreClock (Hz), to get TIM3 counter
    clock at 24 MHz the Prescaler is computed as following:
     - Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density
    and Connectivity line devices and to 24 MHz for Low-Density Value line and
    Medium-Density Value line devices

    The TIM3 is running at 36 KHz: TIM3 Frequency = TIM3 counter clock/(ARR + 1)
                                                  = 24 MHz / 666 = 36 KHz
    TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
    TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 = 37.5%
    TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR)* 100 = 25%
    TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR)* 100 = 12.5%
  ----------------------------------------------------------------------- */  
  /* Compute the prescaler value */
  //PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;
  PrescalerValue = (uint16_t) (SystemCoreClock / 24000) - 1;
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 23999;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM3, ENABLE);
}

/**
  * @brief  Configure the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

  /* Enable the USARTy Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable and set EXTI0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable and set EXTI10 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);    
}

/**
  * @brief  Configure PA.00 in interrupt mode
  * @param  None
  * @retval None
  */
void EXTI0_Configuration(void)
{
  /* Connect EXTI0 Line to PA.00 pin */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}

/**
  * @brief  Configure PC.10 in interrupt mode
  * @param  None
  * @retval None
  */
void EXTI10_Configuration(void)
{
  /* Connect EXTI10 Line to PC.10 pin */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource10);

  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line10;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}

/**
  * @brief  Configure PC.10 in interrupt mode
  * @param  None
  * @retval None
  */
void EXTI11_Configuration(void)
{
  /* Connect EXTI11 Line to PC.11 pin */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource11);

  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line11;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}

/**
  * @brief  Configure PC.10 in interrupt mode
  * @param  None
  * @retval None
  */
void EXTI12_Configuration(void)
{
  /* Connect EXTI10 Line to PC.10 pin */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource12);

  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line12;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}


/**
  * @brief  Configure the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
void USART1_Configuration(void)
{
  USART_InitTypeDef   USART_InitStructure;
  
  USART_DeInit(USART1);
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(USART1,&USART_InitStructure);
  /* Enable USARTy Receive and Transmit interrupts */
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}

#endif
/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
