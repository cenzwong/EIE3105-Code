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

void USART3_init(void){
	//USART3 TX RX
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	//TX2
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = USART3_Tx_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(USART3_GPIO, &GPIO_InitStructure);

	//RX2
	GPIO_InitStructure.GPIO_Pin = USART3_Rx_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(USART3_GPIO, &GPIO_InitStructure); 
	
	//USART2 ST-LINK USB
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	
	USART_InitTypeDef USART_InitStructure;
	//USART_ClockInitTypeDef USART_ClockInitStructure; 
	
	USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	USART_Init(USART3, &USART_InitStructure);
	USART_Cmd(USART3, ENABLE);
}


void USART2_init(void) {
	//USART2 TX RX
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	//TX2
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = USART2_Tx_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(USART2_GPIO, &GPIO_InitStructure);

	//RX2
	GPIO_InitStructure.GPIO_Pin = USART2_Rx_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(USART2_GPIO, &GPIO_InitStructure); 
	
	//USART2 ST-LINK USB
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	
	USART_InitTypeDef USART_InitStructure;
	//USART_ClockInitTypeDef USART_ClockInitStructure; 
	
	USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	USART_Init(USART2, &USART_InitStructure);
	USART_Cmd(USART2, ENABLE);
	
}

void USART2_ITinit(){
	NVIC_InitTypeDef NVIC_InitStructure;
	
	// Enable the USART2 TX Interrupt 
////	USART_ITConfig(USART2, USART_IT_TC, ENABLE );
////	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
////	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
////	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
////	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
////	NVIC_Init(&NVIC_InitStructure);
	
	
	// Enable the USART2 RX Interrupt
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE );
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*
	//void USART_SendData(USART_TypeDef* USARTx, uint16_t Data)//
	while(!USART_GetFlagStatus(USART2, USART_FLAG_TC));
	USART_SendData(USART2, 0xAB); //only BA is shown
*/

/*
		sprintf(buffer, "ch0=%d ch1=%d ch4=%d\r\n", ADC_values[0], ADC_values[1], ADC_values[2]);
		USART_SendString(buffer, sizeof(buffer));

*/

//self_init
void USART_SendString(USART_TypeDef* USARTx, char *pucBuffer, unsigned long ulCount)
{
    //
    // Loop while there are more characters to send.
    //
    while(ulCount--)
    {
        USART_SendData(USARTx, *pucBuffer++);// Last Version USART_SendData(USART1,(uint16_t) *pucBuffer++);
        /* Loop until the end of transmission */
        while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
        {
        }
    }
}

void USART_ReceiveString(USART_TypeDef* USARTx, char *pucBuffer){

	char ch[50] = {0};
	strcpy(pucBuffer,ch);
	//usart_send_string(_str);
	char temp = 0;
	unsigned int i = 0;

	for (i = 0; temp != '\n' ; i++)
	{
		while(USART_GetFlagStatus(USARTx, USART_FLAG_RXNE) == RESET);
		temp = USART_ReceiveData(USARTx) & 0xFF;
		ch[i] = temp;
	}
	//usart_send_string(ch);
	strcpy(pucBuffer,ch);
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
	outputChannelInit.TIM_Pulse = 200-1; 
	outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
	outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM3, &outputChannelInit);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	//Enable Tim3 Ch2 PWM
	outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
	outputChannelInit.TIM_Pulse = 200-1; 
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
	timerInitStructure.TIM_Period = 1;
	timerInitStructure.TIM_ClockDivision = 0;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &timerInitStructure);
	TIM_Cmd(TIM2, ENABLE);
	
	timerInitStructure.TIM_Prescaler = 0;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 1;
	timerInitStructure.TIM_ClockDivision = 0;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &timerInitStructure);
	TIM_Cmd(TIM4, ENABLE);
	
	TIM_TIxExternalClockConfig(TIM2, TIM_TIxExternalCLK1Source_TI2, TIM_ICPolarity_Rising, 0);
	TIM_TIxExternalClockConfig(TIM4, TIM_TIxExternalCLK1Source_TI1, TIM_ICPolarity_Rising, 0);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}


void SysTick_init(void){
	SystemCoreClockUpdate();
	//1000000 -> 0.001 ms
	//10 -> 100ms
	//100 -> 10 ms
	//200 -> 5ms
	//500 -> 2ms
	//1000 -> 1ms
	//10000 -> 0.1ms
	//number big delay smaller
	SysTick_Config(SystemCoreClock / 10000); //call the ststick interrupt every 1us	= 0.01ms 
}

void motorInit(){
		GPIO_WriteBit(Wheel_PWM_M1_GPIO,Wheel_PWM_M1_PIN,Bit_RESET); //RESET FOR ALL THE TIME
		GPIO_WriteBit(Wheel_PWM_APHASE_GPIO,GPIO_Pin_15,Bit_SET);
		GPIO_WriteBit(Wheel_PWM_BPHASE_GPIO,GPIO_Pin_0,Bit_SET);
}

