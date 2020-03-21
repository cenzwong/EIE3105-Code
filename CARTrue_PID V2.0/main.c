#include "stm32f10x.h"                  // Device header
#include "pinMap.h"
#include "init.h"
#include "stdbool.h"
#include "stdio.h"

//PID tunning 
#define Kp 2000
#define Ki 0
#define Kd 0

#define SPI2_NSS_HIGH() GPIO_WriteBit(SPI_NSS_GPIO,SPI_NSS_PIN,Bit_SET)
#define SPI2_NSS_LOW() GPIO_WriteBit(SPI_NSS_GPIO,SPI_NSS_PIN,Bit_RESET)
#define abs(_abs) _abs?_abs:-_abs

void	motorInit(void);
uint8_t getSensorValue();
void dataToarray(uint8_t data, bool* array);
void motorOutput(BitAction R_FW_BW, BitAction L_FW_BW,uint16_t R_SPEED, uint16_t L_SPEED);
uint8_t sumIRSensor();
uint8_t getPosofsensor(uint8_t);  //input data and return the position
int8_t getSensorError(uint8_t input); //return the error

__IO int8_t current_error = 0;
__IO int8_t previous_error = 0;
__IO int32_t integral = 0;
__IO int32_t derivative = 0;
void Control();


#define Forward Bit_SET
#define Backward Bit_RESET

//global speed
__IO uint16_t L_Speed = 0;
__IO uint16_t R_Speed = 0;

//wheel counter value
__IO uint64_t Left_counter = 0;
__IO uint64_t Right_counter = 0;

__IO uint8_t data = 0; //direct receive from the sensor
bool IRdata[8] = {0}; //mapped into different position

void delay(int t);

//RWheel can as low as 603
//#define FAST_MINSPEED 616 
//#define FAST_MAXSPEED 1615

#define SLOW_MINSPEED	900
#define SLOW_MAXSPEED	2700

#define MAX_POWER 2000
//#define FAST_SPEED(_1to1000) _1to1000+615
#define SLOW_SPEED(_1to2000) _1to2000*1+1000

int main(void) {
	
	GPIO_init();
	USART2_init();
	SPI2_init();
	Wheel_PWM_init();
	SysTick_init();
	Wheel_Counter_init();
	motorInit();
	
	//**************** start-up meeting ***************
	while(!USART_GetFlagStatus(USART2, USART_FLAG_TC));
	USART_SendData(USART2, 0xAB); //only BA is shown
	
	while(!USART_GetFlagStatus(USART2, USART_FLAG_TC));
	USART_SendData(USART2, 0xCD);
	//**************** start-up meeting******************
	delay(5000);
	motorOutput(Forward, Forward,500, 500);
	
	while(1) {
		if(isSwitchPressed()){motorOutput(Forward, Forward,0, 0);}
		//100ms
		


//////		if(Left_counter% 10 == 0){
//////				while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
//////				USART_SendData(USART2, 0xBB);
//////		}
//////		
//////		if(Right_counter% 10 == 1){
//////				while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
//////				USART_SendData(USART2, 0xCC);
//////		}
		
		//update the PWM
		motorOutput(Forward, Forward,R_Speed, L_Speed);
	}
}

uint8_t getSensorValue(){
		
		uint8_t SPI_receive_val;
		//Generate Clock to load data
		SPI2_NSS_HIGH();
		SPI_I2S_SendData(SPI2,0xAA);
		
		//Send dummy and receive data from shift register
		SPI2_NSS_LOW();
		while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
		SPI_I2S_SendData(SPI2,0xAA);
		while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
		SPI_receive_val = SPI_I2S_ReceiveData(SPI2);
		SPI2_NSS_HIGH();
	

	
		//DEBUG========================
////////		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
////////		USART_SendData(USART2, SPI_receive_val);
		
		return SPI_receive_val;
}

void delay(int t) {
	t = t*1000;
	int i, j;
	for(i=0; i<t; i++)
		j++;
}
void TIM4_IRQHandler(void) {
	//Left counter
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET){
	
		Left_counter++;
		
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
}


void TIM2_IRQHandler(void) {
	//RIGHT counter
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){
			Right_counter++;

		
			TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}                                                 
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}



void SysTick_Handler(){
	//20 -> 50 ms
		data = getSensorValue();	
		data = ~data;	//flip the data to the 1 when it lay on the black line
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
	USART_SendData(USART2, data);
		//dataToarray(data, IRdata);
		
	//update the R_Speed and L_Speed
	Control();
}

void dataToarray(uint8_t dataIn, bool* array){
			
////////////////////////////		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
////////////////////////////		USART_SendData(USART2, 0xff);
			for(uint8_t i = 0; i < 8; i++){
				array[i] = dataIn & (1 << i);
//////////////////						while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
//////////////////						USART_SendData(USART2, array[i]);
			}
//////////////////////		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
//////////////////////		USART_SendData(USART2, 0xff);
}




void motorOutput(BitAction R_FW_BW, BitAction L_FW_BW,uint16_t R_SPEED, uint16_t L_SPEED){
			//input FORWARD or BACKWARD and speed from 0 to MAX_POWER			
			if(R_SPEED < 0){	
				//MIN
				TIM_SetCompare1(TIM3, 0); 
				R_Speed = 0;
			} else if(R_SPEED > MAX_POWER){
				//MAX
				TIM_SetCompare1(TIM3, SLOW_SPEED(MAX_POWER)); 
				R_Speed = MAX_POWER;
			}	else{
				//NORM
				TIM_SetCompare1(TIM3, SLOW_SPEED(R_SPEED)); 
				R_Speed = R_SPEED;
			}
			
			if(L_SPEED < 0){	
				TIM_SetCompare2(TIM3, 0); 
				L_Speed = 0;
			}
			else if(L_SPEED > MAX_POWER){
				TIM_SetCompare2(TIM3, SLOW_SPEED(MAX_POWER));
				L_Speed = MAX_POWER;
			}
			else{
				TIM_SetCompare2(TIM3, SLOW_SPEED(L_SPEED));
				L_Speed = L_SPEED;
			}

}

void motorInit(){
		GPIO_WriteBit(Wheel_PWM_M1_GPIO,Wheel_PWM_M1_PIN,Bit_RESET); //RESET FOR ALL THE TIME
		GPIO_WriteBit(Wheel_PWM_APHASE_GPIO,GPIO_Pin_15,Forward);
		GPIO_WriteBit(Wheel_PWM_BPHASE_GPIO,GPIO_Pin_0,Forward);
}

uint8_t sumIRSensor(){
	uint8_t count1 = 0;;
	for(uint8_t i = 0; i < 8; i++){
		if(IRdata[i] == 1){
			count1++;
		}
	}
	return count1;
}

uint8_t getPosofsensor(uint8_t dataIn){
		for(uint8_t i = 0; i < 8; i++){
				if(dataIn && (1<<i)){return i;}
		}	
}

int8_t getSensorError(uint8_t input){
	
	switch(input){
		
		//=========RIGHT===============
		//0001 1000
		case 0x18: return 0; break;
		
		//0000 1000
		case 0x08: return 0; break;
		
		//0000 1100
		case 0x0C: return 1; break;
		
		//0000 0100
		case 0x04: return 2; break;
		
		//0000 0110
		case 0x06: return 3; break;
		
		//0000 0010
		case 0x02: return 4; break;
		
		//0000 0011
		case 0x03: return 5; break;
		
		//0000 0001
		case 0x01: return 6; break;
		
		//================LEFT================
		//0001 0000
		case 0x10: return 0; break;
		
		//0011 0000
		case 0x30: return -1; break;
		
		//0010 0000
		case 0x20: return -2; break;
		
		//0110 0000
		case 0x60: return -3; break;
		
		//0100 0000
		case 0x40: return -4; break;
		
		//1100 0000
		case 0xC0: return -5;	break;
		
		//1000 0000
		case 0x80: return -6; break;
		
		//default
		default: return 8;
	}

}


void Control(){
	current_error = getSensorError(data);
	
	if( abs(current_error) <= 1){integral = 0; LEDOn();}
	else if(current_error == 8){}
	else{integral += current_error; LEDOff();}
	
	if(integral > 200){ integral = 200; }
	else if(integral < -200){ integral = -200;}
	
	derivative  = current_error - previous_error;
	
	R_Speed += (Kp*current_error + Ki*integral + Kd*derivative);
	L_Speed -= (Kp*current_error + Ki*integral + Kd*derivative);
	
	previous_error = current_error;
}