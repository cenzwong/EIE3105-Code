// Pin Usage
// Function      **  Pin Name ** Board Pin Out

//LED PB7==================
#define LED_RCC_GPIO  RCC_APB2Periph_GPIOB
#define LED_GPIO      GPIOB
#define LED_PIN       GPIO_Pin_7
#define LEDOn()       GPIO_SetBits(LED_GPIO,LED_PIN);
#define LEDOff()			GPIO_ResetBits(LED_GPIO,LED_PIN);
#define LEDToggle()		GPIO_WriteBit(LED_GPIO, LED_PIN, !GPIO_ReadOutputDataBit(LED_GPIO,LED_PIN))

//Switch PB8====================
#define Switch_RCC_GPIO  RCC_APB2Periph_GPIOB
#define Switch_GPIO      GPIOB
#define Switch_PIN       GPIO_Pin_8
//active low
#define isSwitchPressed() !GPIO_ReadInputDataBit(Switch_GPIO,Switch_PIN) 

//SPI2===================================
#define SPI_NSS_GPIO		GPIOB
#define SPI_NSS_PIN			GPIO_Pin_12

#define	SPI_SCK_GPIO		GPIOB
#define SPI_SCK_PIN			GPIO_Pin_13

#define	SPI_MISO_GPIO		GPIOB
#define SPI_MISO_PIN		GPIO_Pin_14

#define	SPI_MOSI_GPIO		GPIOB
#define SPI_MOSI_PIN		GPIO_Pin_15

//Wheel PWM=================================
#define Wheel_PWM_M1_GPIO 	GPIOC
#define Wheel_PWM_M1_PIN 		GPIO_Pin_14

#define Wheel_PWM_APHASE_GPIO 	GPIOC
#define Wheel_PWM_APHASE_PIN 		GPIO_Pin_15

#define Wheel_PWM_BPHASE_GPIO 	GPIOA
#define Wheel_PWM_BPHASE_PIN 		GPIO_Pin_0

//~PWM --> Timer 3 Ch1
#define Wheel_PWM_AENBL_GPIO 		GPIOA
#define Wheel_PWM_AENBL_PIN 		GPIO_Pin_6

//~PWM --> Timer 3 CH2
#define Wheel_PWM_BENBL_GPIO 		GPIOA
#define Wheel_PWM_BENBL_PIN 		GPIO_Pin_7

//Wheel Counter=======External intrerrupt==========
#define Wheel_Counter_Left_GPIO							GPIOB
#define Wheel_Counter_Left_PIN							GPIO_Pin_6
#define Wheel_Counter_Left_EXTI_LINE   			EXTI_Line6
#define Wheel_Counter_Left_GPIO_PORTSOURCE 	GPIO_PortSourceGPIOB
#define Wheel_Counter_Left_GPIO_PINSOURCE  	GPIO_PinSource6


#define Wheel_Counter_Right_GPIO						GPIOA
#define Wheel_Counter_Right_PIN							GPIO_Pin_1
#define Wheel_Counter_Right_EXTI_LINE   		EXTI_Line1
#define Wheel_Counter_Right_GPIO_PORTSOURCE 	GPIO_PortSourceGPIOA
#define Wheel_Counter_Right_GPIO_PINSOURCE  	GPIO_PinSource1

//USART2
#define USART2_GPIO 												GPIOA
#define USART2_Tx_PIN 											GPIO_Pin_2
#define USART2_Rx_PIN												GPIO_Pin_3
//USART3
#define USART3_GPIO 												GPIOB
#define USART3_Tx_PIN												GPIO_Pin_10
#define USART3_Rx_PIN												GPIO_Pin_11

