#include "stm32f10x.h"                  // Device header
#include "GPIO_STM32F10x.h"             // Keil::Device:GPIO
#include "pinMap.h"



void GPIO_init(){

	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(LED_RCC_GPIO, ENABLE);
	RCC_APB2PeriphClockCmd(Switch_RCC_GPIO,ENABLE);
	
	//LED
	GPIO_InitStructure.GPIO_Pin = LED_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(LED_GPIO, &GPIO_InitStructure);

	//Switch
	GPIO_InitStructure.GPIO_Pin = Switch_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(Switch_GPIO, &GPIO_InitStructure);

}

void USART2_init(void) {
	//USART2 TX RX
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	//TX2
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//RX2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
	//USART2 ST-LINK USB
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	
	USART_InitTypeDef USART_InitStructure;
	//USART_ClockInitTypeDef USART_ClockInitStructure; 
	
	USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	USART_Init(USART2, &USART_InitStructure);
	USART_Cmd(USART2, ENABLE);
}

void SPI2_init(void){

	//MASTER MODE
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure; 
	//MOSI
	GPIO_InitStructure.GPIO_Pin = SPI_MOSI_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//SCK
	GPIO_InitStructure.GPIO_Pin = SPI_SCK_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//NSS	
	GPIO_InitStructure.GPIO_Pin = SPI_NSS_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//MISO
	GPIO_InitStructure.GPIO_Pin = SPI_MISO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	
	//SPI config
	SPI_InitTypeDef SPI2_InitStructure;
	SPI2_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI2_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI2_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI2_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI2_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI2_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI2_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI2_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	//SPI2_InitStructure.SPI_CRCPolynomial = SPI_GetCRCPolynomial(SPI2); 
	SPI_Init(SPI2, &SPI2_InitStructure);
	
	//SPI_CalculateCRC(SPI2, DISABLE);			//DISABLE CRC
	//SPI_I2S_DMACmd(SPI2,SPI_I2S_DMAReq_Rx,ENABLE);
	SPI_Cmd(SPI2,ENABLE);
	
	//SPI_BiDirectionalLineConfig(SPI2,SPI_Direction_Rx);
}


void Wheel_PWM_init(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);


	GPIO_InitTypeDef GPIO_InitStructure; 
	//Wheel_PWM_M1
	GPIO_InitStructure.GPIO_Pin = Wheel_PWM_M1_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(Wheel_PWM_M1_GPIO, &GPIO_InitStructure);
	
	//Wheel_PWM_APHASE
	GPIO_InitStructure.GPIO_Pin = Wheel_PWM_APHASE_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(Wheel_PWM_APHASE_GPIO, &GPIO_InitStructure);
	
	//Wheel_PWM_BPHASE
	GPIO_InitStructure.GPIO_Pin = Wheel_PWM_BPHASE_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(Wheel_PWM_BPHASE_GPIO, &GPIO_InitStructure);
	
	//Wheel_PWM_AENBL		~PWM
	GPIO_InitStructure.GPIO_Pin = Wheel_PWM_AENBL_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(Wheel_PWM_AENBL_GPIO, &GPIO_InitStructure);
	
	//Wheel_PWM_BENBL		~PWM
	GPIO_InitStructure.GPIO_Pin = Wheel_PWM_BENBL_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(Wheel_PWM_BENBL_GPIO, &GPIO_InitStructure);

	//Tim3 set-up 

	
	TIM_TimeBaseInitTypeDef timerInitStructure; 
	
  timerInitStructure.TIM_Prescaler = 144-1;  //1/(72Mhz/1440)=0.2ms
  timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  timerInitStructure.TIM_Period = 5000-1;  
  timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  timerInitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM3, &timerInitStructure);
  TIM_Cmd(TIM3, ENABLE);
	
	//***********PWM****setup**********
	TIM_OCInitTypeDef outputChannelInit;
	//Enable Tim3 Ch1 PWM
	outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
	outputChannelInit.TIM_Pulse = 1-1; 
	outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
	outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM3, &outputChannelInit);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	//Enable Tim3 Ch2 PWM
	outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
	outputChannelInit.TIM_Pulse = 1-1; 
	outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
	outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC2Init(TIM3, &outputChannelInit);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
}


void Wheel_Counter_init(void){
////////////////////////////////////	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2|RCC_APB1Periph_TIM4, ENABLE);
	
	
//////////////////////////////////////	GPIO_InitTypeDef GPIO_InitStructure; 
//////////////////////////////////////	//Wheel_Counter_Left	
//////////////////////////////////////	GPIO_InitStructure.GPIO_Pin = Wheel_Counter_Left_PIN;
//////////////////////////////////////	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//////////////////////////////////////	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//////////////////////////////////////	GPIO_Init(Wheel_Counter_Left_GPIO, &GPIO_InitStructure);
//////////////////////////////////////	//Wheel_Counter_Right	
//////////////////////////////////////	GPIO_InitStructure.GPIO_Pin = Wheel_Counter_Right_PIN;
//////////////////////////////////////	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//////////////////////////////////////	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//////////////////////////////////////	GPIO_Init(Wheel_Counter_Right_GPIO, &GPIO_InitStructure);

	//Timer 2 set up 
	
 
	TIM_TimeBaseInitTypeDef timerInitStructure;
	timerInitStructure.TIM_Prescaler = 0;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 10;
	timerInitStructure.TIM_ClockDivision = 0;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &timerInitStructure);
	TIM_Cmd(TIM2, ENABLE);
	timerInitStructure.TIM_Prescaler = 0;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 10;
	timerInitStructure.TIM_ClockDivision = 0;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &timerInitStructure);
	TIM_Cmd(TIM4, ENABLE);
	
	TIM_TIxExternalClockConfig(TIM2, TIM_TIxExternalCLK1Source_TI2, TIM_ICPolarity_Rising, 0);
	TIM_TIxExternalClockConfig(TIM4, TIM_TIxExternalCLK1Source_TI2, TIM_ICPolarity_Rising, 0);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_EnableIRQ(TIM4_IRQn);
	
}


void SysTick_init(void){
	SystemCoreClockUpdate();
	//1000000 -> 0.001 ms
	//10 -> 100ms
	//100 -> 10 ms
	//200 -> 5ms
	//number big delay smaller
	SysTick_Config(SystemCoreClock / 200); //call the ststick interrupt every 1us	= 0.01ms 
}
