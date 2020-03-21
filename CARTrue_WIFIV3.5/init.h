//Init

//LED PB7
//Switch PB8

//motor
#define Forward Bit_SET
#define Backward Bit_RESET

void GPIO_init(void);
void USART2_init(void);
void USART2_ITinit(void);
void USART3_init(void);
void SPI2_init(void);
void Wheel_PWM_init(void);
void Wheel_Counter_init(void);
void SysTick_init(void);
void motorInit(void);
//void USART_SendString(USART_TypeDef* USARTx, char *pucBuffer, unsigned long ulCount);
//void USART_ReceiveString(USART_TypeDef* USARTx, char *pucBuffer);
