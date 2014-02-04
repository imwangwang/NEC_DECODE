#include "rc5_decode.h"

__IO uint32_t RC5Counter = 0;

uint16_t  RC5MinT = 0;
uint16_t  RC5MaxT = 0;
uint16_t  RC5Min2T = 0;
uint16_t  RC5Max2T = 0;
static uint32_t TIMCLKValueKHz = 0;
uint16_t RC5TimeOut = 0;
__IO uint32_t RC5_Data = 0;
RC5_Frame_TypeDef RC5_FRAME;
extern __IO uint8_t RFDemoStatus;

static uint8_t RC5_GetPulseLength (uint32_t pulseLength);
static void RC5_modifyLastBit(tRC5_lastBitType bit);
static void RC5_WriteBit(uint8_t bitVal);
static uint32_t TIM_GetCounterCLKValue(void);

void RC5_DeInit(void)
{ 
  /*
	TIM_DeInit(IR_TIM);
  GPIO_DeInit(IR_GPIO_PORT);
  */


	GPIO_InitTypeDef GPIO_InitStructure;
	  NVIC_InitTypeDef NVIC_InitStructure;
	  TIM_ICInitTypeDef TIM_ICInitStructure;

	  /*  Clock Configuration for TIMER */
	  RCC_APB1PeriphClockCmd(IR_TIM_CLK , ENABLE);

	  /* Enable GPIO clock */
	  RCC_AHB1PeriphClockCmd(IR_GPIO_PORT_CLK , ENABLE);

	  /* Pin configuration: input floating */
	  GPIO_InitStructure.GPIO_Pin = IR_GPIO_PIN;
	  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	  GPIO_Init(IR_GPIO_PORT, &GPIO_InitStructure);

	  GPIO_PinAFConfig(IR_GPIO_PORT,IR_GPIO_SOURCE,GPIO_AF_TIM2);

	  /* Enable the TIM global Interrupt */
	  NVIC_InitStructure.NVIC_IRQChannel = IR_TIM_IRQn ;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	  NVIC_Init(&NVIC_InitStructure);

	  /* TIMER frequency input */
	  TIM_PrescalerConfig(IR_TIM, TIM_PRESCALER, TIM_PSCReloadMode_Immediate);

	  TIM_ICStructInit(&TIM_ICInitStructure);

	  /* TIM configuration */
	  TIM_ICInitStructure.TIM_Channel = IR_TIM_Channel;
	  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
	  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	  TIM_ICInitStructure.TIM_ICFilter = 0x0;

	  TIM_PWMIConfig(IR_TIM, &TIM_ICInitStructure);

	  /* Select the TIM Input Trigger: TI2FP2 */
	  TIM_SelectInputTrigger(IR_TIM, TIM_TS_TI2FP2);

	  /* Select the slave Mode: Reset Mode */
	  TIM_SelectSlaveMode(IR_TIM, TIM_SlaveMode_Reset);

	  /* Enable the Master/Slave Mode */
	  TIM_SelectMasterSlaveMode(IR_TIM, TIM_MasterSlaveMode_Enable);

	  /* Clear update flag */
	  TIM_ClearFlag(IR_TIM, TIM_FLAG_Update);

	  /* Enable the CC2/CC1 Interrupt Request */
	  TIM_ITConfig(IR_TIM, TIM_IT_CC2, DISABLE);
	  /* Enable the CC2/CC1 Interrupt Request */
	  TIM_ITConfig(IR_TIM, TIM_IT_CC1, DISABLE);

	  /* Enable the timer */
	  TIM_Cmd(IR_TIM, DISABLE);



}

void RC5_Init(void)
{ 
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;
  
  /*  Clock Configuration for TIMER */
  RCC_APB1PeriphClockCmd(IR_TIM_CLK , ENABLE);

  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(IR_GPIO_PORT_CLK , ENABLE);
 
  /* Pin configuration: input floating */
  GPIO_InitStructure.GPIO_Pin = IR_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(IR_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_PinAFConfig(IR_GPIO_PORT,IR_GPIO_SOURCE,GPIO_AF_TIM2);
  
  /* Enable the TIM global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = IR_TIM_IRQn ;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
    
  /* TIMER frequency input */
  TIM_PrescalerConfig(IR_TIM, TIM_PRESCALER, TIM_PSCReloadMode_Immediate);
  
  TIM_ICStructInit(&TIM_ICInitStructure);
  
  /* TIM configuration */
  TIM_ICInitStructure.TIM_Channel = IR_TIM_Channel;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  
  TIM_PWMIConfig(IR_TIM, &TIM_ICInitStructure); 

  /* Timer Clock */
  TIMCLKValueKHz = TIM_GetCounterCLKValue()/1000 ; 

  /* Select the TIM Input Trigger: TI2FP2 */
  TIM_SelectInputTrigger(IR_TIM, TIM_TS_TI2FP2);

  /* Select the slave Mode: Reset Mode */
  TIM_SelectSlaveMode(IR_TIM, TIM_SlaveMode_Reset);

  /* Enable the Master/Slave Mode */
  TIM_SelectMasterSlaveMode(IR_TIM, TIM_MasterSlaveMode_Enable);

  /* Configures the TIM Update Request Interrupt source: counter overflow */
  TIM_UpdateRequestConfig(IR_TIM,  TIM_UpdateSource_Global);
  
  RC5TimeOut = TIMCLKValueKHz * RC5_TIME_OUT_US/1000;

  /* Set the TIM auto-reload register for each IR protocol */
  IR_TIM->ARR = RC5TimeOut;
  
  /* Clear update flag */
  TIM_ClearFlag(IR_TIM, TIM_FLAG_Update);
    
  /* Enable the CC2/CC1 Interrupt Request */
  TIM_ITConfig(IR_TIM, TIM_IT_CC2, ENABLE);
  /* Enable the CC2/CC1 Interrupt Request */
  TIM_ITConfig(IR_TIM, TIM_IT_CC1, ENABLE);

  /* Enable the timer */
  TIM_Cmd(IR_TIM, ENABLE);  
    
  /* Bit time range */
  RC5MinT = (RC5_T_US - RC5_T_TOLERANCE_US) * TIMCLKValueKHz / 1000;
  RC5MaxT = (RC5_T_US + RC5_T_TOLERANCE_US) * TIMCLKValueKHz / 1000;
  RC5Min2T = (2 * RC5_T_US - RC5_T_TOLERANCE_US) * TIMCLKValueKHz / 1000;
  RC5Max2T = (2 * RC5_T_US + RC5_T_TOLERANCE_US) * TIMCLKValueKHz / 1000;
  
  /* Default state */
  RC5_ResetPacket();
}

void RC5_Decode(RC5_Frame_TypeDef *rc5_frame)
{ 
  /* If frame received */
  if(RC5FrameReceived != NO )  
  {
    RC5_Data = RC5TmpPacket.data ;
    /* RC5 frame field decoding */
    rc5_frame->FieldBit = (RC5TmpPacket.data >> 12) & 0x1;
    rc5_frame->ToggleBit = (RC5TmpPacket.data >> 11) & 0x1;
    rc5_frame->Address = (RC5TmpPacket.data >> 6) & 0x1F;
    rc5_frame->Command = (uint8_t)((RC5TmpPacket.data) & (uint8_t) 0x3F);

    if (((RC5TmpPacket.data >> 12) & 0x1) != 0x01)
    {
      rc5_frame->Command =  (1<<6)| rc5_frame->Command; 
    }
    /* Default state */
    RC5FrameReceived = NO;
    RC5_ResetPacket();   
  
  }
}

void RC5_ResetPacket(void)
{
  RC5TmpPacket.data = 0;
  RC5TmpPacket.bitCount = RC5_PACKET_BIT_COUNT - 1;
  RC5TmpPacket.lastBit = RC5_ONE;
  RC5TmpPacket.status = RC5_PACKET_STATUS_EMPTY;
}

void RC5_DataSampling(uint32_t rawPulseLength, uint32_t edge)
{ 
  uint8_t pulse;
  tRC5_lastBitType tmpLastBit;
  
  /* Decode the pulse length in protocol units */
  pulse = RC5_GetPulseLength(rawPulseLength);

  /* On Rising Edge */
  if (edge == 1)
  { 
    if (pulse <= RC5_2T_TIME) 
    { 
      /* Bit determination by the rising edge */
      tmpLastBit = RC5_logicTableRisingEdge[RC5TmpPacket.lastBit][pulse];
      RC5_modifyLastBit (tmpLastBit);
    }
    else
    {
      RC5_ResetPacket();
    }
  } 
  else     /* On Falling Edge */
  {
    /* If this is the first falling edge - don't compute anything */
    if (RC5TmpPacket.status & RC5_PACKET_STATUS_EMPTY)
    { 
      RC5TmpPacket.status &= (uint8_t)~RC5_PACKET_STATUS_EMPTY;
    }
    else	
    {
      if (pulse <= RC5_2T_TIME) 
      { 
        /* Bit determination by the falling edge */
        tmpLastBit = RC5_logicTableFallingEdge[RC5TmpPacket.lastBit][pulse];
        RC5_modifyLastBit (tmpLastBit);
      }
      else
      {
        RC5_ResetPacket();
      }
    }
  }
}  

static uint8_t RC5_GetPulseLength (uint32_t pulseLength)
{ 
  /* Valid bit time */
  if ((pulseLength > RC5MinT) && (pulseLength < RC5MaxT))
  { 
    /* We've found the length */
    return (RC5_1T_TIME);	/* Return the correct value */
  }
  else if ((pulseLength > RC5Min2T) && (pulseLength < RC5Max2T))
  {
    /* We've found the length */
    return (RC5_2T_TIME);/* Return the correct value */
  }
  return RC5_WRONG_TIME;/* Error */
}

static void RC5_modifyLastBit(tRC5_lastBitType bit)
{
  if (bit != RC5_NAN)
  {
    if (RC5TmpPacket.lastBit != RC5_INV)
    { 
      /* Restore the last bit */
      RC5TmpPacket.lastBit = bit;

      /* Insert one bit into the RC5 Packet */
      RC5_WriteBit(RC5TmpPacket.lastBit);
    }
    else 
    {
      RC5_ResetPacket();
    }
  }
}

static void RC5_WriteBit(uint8_t bitVal)
{
  /* First convert RC5 symbols to ones and zeros */
  if (bitVal == RC5_ONE)
  { 
    bitVal = 1;
  }
  else if (bitVal == RC5_ZER)
  {
    bitVal = 0;
  }
  else
  {
    RC5_ResetPacket();
    return;
  } 

  /* Write this particular bit to data field */
  RC5TmpPacket.data |=  bitVal; 
  
  /* Test the bit number determined */ 
  if ((RC5TmpPacket.bitCount != 0) && (RC5TmpPacket.bitCount<250))  /* If this is not the last bit */
  {
    /* Shift the data field */
    RC5TmpPacket.data = RC5TmpPacket.data << 1;
    /* And decrement the bitCount */
    RC5TmpPacket.bitCount--;
  } 
  else if((RC5TmpPacket.bitCount>=250)||(RC5TmpPacket.bitCount==0))
  {
   RC5FrameReceived = YES;
   RC5_ReceiveInterrupt();
  }
}

static uint32_t TIM_GetCounterCLKValue(void)
{
  uint32_t apbprescaler = 0, apbfrequency = 0;
  uint32_t timprescaler = 0;
  __IO RCC_ClocksTypeDef RCC_ClockFreq;   
  
  /* This function fills the RCC_ClockFreq structure with the current
  frequencies of different on chip clocks */
  RCC_GetClocksFreq((RCC_ClocksTypeDef*)&RCC_ClockFreq);
  
  /* Get the clock prescaler of APB1 */
  apbprescaler = ((RCC->CFGR >> 8) & 0x7);
  apbfrequency = RCC_ClockFreq.PCLK1_Frequency; 
  timprescaler = TIM_PRESCALER;
  
  /* If APBx clock div >= 4 */
  if (apbprescaler >= 4)
  {
    return ((apbfrequency * 2)/(timprescaler + 1));
  }
  else
  {
    return (apbfrequency/(timprescaler+ 1));
  }
}

char* RC5_GetDevice(uint8_t address){
	return (char*)rc5_devices[address];
}

char* RC5_GetCommand(uint8_t command){
	return (char*)rc5_Commands[command];
}
