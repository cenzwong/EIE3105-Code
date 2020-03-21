#include "stm32f10x.h"                  // Device header
#include "pinMap.h"
#include "init.h"
#include "stdbool.h"
#include "stdio.h"
#include "misc.h"
#include "string.h"
#include "math.h"

//ki0.6 kd0.2

#define __abs(_abs) (_abs>0)?_abs:-_abs
#define __MIN(a,b) (((a)<(b))?(a):(b))
#define __MAX(a,b) (((a)>(b))?(a):(b))

#define ANGLE(X1,Y1,X2,Y2) (float)atan((float)((int64_t)Y1 - (int64_t)Y2)\
															/(float)((int64_t)X1 - (int64_t)X2));

//==================CAR dimension========
//Wheel to wheel distance (10cm)
#define L_wheel2wheel 10.0
//wheel radius (5cm)
#define R_wheelradius 5.0
//#define 

//=============END OF CAR dimension======


//=============receiving message==================

		//==decode var==
		__IO bool decodeFlag = false;
		int decode(char num5, char num6, char num8, char num9, char num14, char num15, \
					char num16, char num17, char num18, char num19, char num20, char num21, \
						char num22, char num23, char num24, char num25);

		//CRD001XXXYYYSSS: RED CAR object at (XXX, YYY)

				//enum {ATReply, ConnectReply, RecevieData} ESPStage = ATReply;
				/*
					Ball ballBlue("BBE", Scalar(100,135,50), Scalar(106,174,224));
					Ball ballOrange("BOE", Scalar(18,200,50), Scalar(22,255,224));
					Ball ballPink("BPK", Scalar(170,200,50), Scalar(177,250,224));
					Ball ballYellow("BYW", Scalar(29,160,50), Scalar(32,244,224));
					Ball ballRed("BRD", Scalar(177,179,50), Scalar(181,253,224));
					Car carYellow("CYW", Scalar(26,181,50), Scalar(35,212,250));
					Car carBlue("CBE", Scalar(105,172,50), Scalar(117,240,250));
				//Car carRed("CRD", Scalar(175,179,50), Scalar(185,253,224));
					Car carRed("CRD", Scalar(0,229,50), Scalar(3,253,224));
					Car carViolet("CVT", Scalar(118,130,50), Scalar(128,160,224));
				*/

				//=====================BALL BALL World==========

				//receive message

				//use Capital letter as the char
				#define CHARDEC2NUM(CHAR) (int)(CHAR&0x0F)        
				#define CHARHEX2NUM(CHAR) (int)( CHARDEC2NUM(CHAR) + 9*(CHAR>>6))

				//======decode area========
				//ground
				#define fieldXMin 0x02C
				#define fieldYMin 0x051
				#define fieldXMax 0x4C7
				#define fieldYMax 0x298

				#define BALL 'B'
				#define CAR 'C'

				#define BLUE 	'B'
				#define ORANGE 	'O'
				#define PINK 	'P'
				#define YELLOW 	'Y'
				#define RED 	'R'
				#define VIOLET 	'V'
				#define NULLL 'L'

				typedef struct{
						char Obj;
						char myColor;
						int64_t XPos;
						int64_t YPos;
						
						int64_t RPos;
						double Angle;
				}BallCarProp;

				//>>>>>>>>>>>>>>>>>>>>===========CONFIG ZONE===================<<<<<<<<<<<<<<<<<<<<<<<<<
				BallCarProp Sing_Car = {CAR, YELLOW, 0, 0};
				BallCarProp Football = {BALL, PINK, 0, 0};
				
				typedef struct{
					char Obj;
					char myFColor;
					char myBColor;
					
					int64_t XPos;
					int64_t YPos;
					
					int64_t XFPos;
					int64_t YFPos;
					int64_t XBPos;
					int64_t YBPos;
					
					int64_t XResting;
					int64_t YResting;
					
					float targetAngle;
					float myAngle;
				
				}twoCarProp;
				twoCarProp Cenz_Car = {CAR, BLUE, VIOLET, 0, 0, 0, 0, 0.0, 0};
				//float Cenz_Car_Angle = 0;

				__IO uint8_t receive_pointer = 0;

				float BallCarAngle = 0;
//=============END of receiving message==================
//PID
typedef struct{
		
		float Kp;
		float Ki;
		float Kd;
		
		float target_value;
		
		float current_error;
		float previous_error;
		
		float derivative_error;
		float integral_error;

		float error_max;
	}PID;
				

//=====ESP01=================
			//ESP01
			void clearString(char *pucBuffer);
			//#define initESP AT+CIPSTART=\"UDP\",\"0\",0,3105,2\r\n
			//AT+CIPSTART="UDP","0",0,3105,2
			const char initESP[] = "AT+CIPSTART=\"UDP\",\"0\",0,3105,2\r\n";
			const char AT[] = "AT\r\n";
			char buffer[50] = {'\0'};
			char receive[50] = {'\0'};
			void ESP01_init(void);
			enum {ATReply, ConnectReply, RecevieData} ESPStage = ATReply;
//=====ESP01=================

//motor
#define Forward Bit_SET
#define Backward Bit_RESET

#define	MOTOR_R(FORBACK) GPIO_WriteBit(Wheel_PWM_APHASE_GPIO,GPIO_Pin_15,FORBACK);
#define MOTOR_L(FORBACK) GPIO_WriteBit(Wheel_PWM_BPHASE_GPIO,GPIO_Pin_0,FORBACK);			

BitAction L_isForward = Forward;
BitAction R_isForward = Forward;
void motorOutput(int32_t R_SPEED, int32_t L_SPEED, BitAction RB_F, BitAction LB_F);
void control();
void setCounterMark(int16_t LeftMark, int16_t RightMark);
			
//wheel counter
__IO uint64_t current_Right_counter = 0;
__IO uint64_t current_Left_counter = 0;
int64_t right_counter_mark = 0;
int64_t left_counter_mark = 0;
int64_t Left_Counter = 0;
int64_t Right_Counter = 0;

void Control_wheel();
			
__IO uint64_t sysTick = 0;

//global speed
__IO int32_t L_Speed = 2200;
__IO int32_t R_Speed = 2200;

__IO int64_t LSetMark = 0;
__IO int64_t RSetMark = 0;

void delay(int t);

#define MAX_POWER 3000	//PWM control
#define MIN_POWER 1900

uint32_t SysTickLeftCountMark = 0;
uint32_t SysTickRightCountMark = 0;
//target speed
//int16_t getwheel_count_Rspeed(){return (int16_t)((60000.0/(control_speed + control_angvel)));}  //return will be from 20 to 255 
//int16_t getwheel_count_Lspeed(){return (int16_t)((60000.0/(control_speed - control_angvel)));}

enum {Wait, Scan, Go, Hit, Miss, Back} StateCar = Wait;

bool isballCarScan();
void CarGo();



int main(void) {
	
	GPIO_init();
	USART2_init();
	USART3_init();
	SPI2_init();
	Wheel_PWM_init();
	SysTick_init();
	Wheel_Counter_init();
	motorInit();
	motorOutput(0,0,R_isForward,L_isForward);
	
	while(!USART_GetFlagStatus(USART3, USART_FLAG_TC));
	USART_SendData(USART3, 'A'); //only BA is shown
	
	
	ESP01_init();
	LEDOff();
//		R_Speed = control_speed + control_angvel;
//		L_Speed = control_speed - control_angvel;
		delay(1000);
		
		R_Speed = 0;
		L_Speed = 0;
		
		//init();
//		while(!isSwitchPressed());
//			delay(10000);
//			LEDOn();
//			motorOutput(2200,2200,Forward,Forward);
//			delay(10000);
//			LEDOff();
			
			
	while(1) {	
		if(decodeFlag){
			decode( receive[5], receive[6], receive[8], receive[9], receive[14],\
							receive[15], receive[16], receive[17], receive[18], receive[19],\
							receive[20], receive[21], receive[22],receive[23], receive[24], receive[25]);
		
			//update the current angle between the Car and the ball
			BallCarAngle = ANGLE( Football.XPos,\
														Football.YPos,\
														Cenz_Car.XBPos,\
														Cenz_Car.YBPos);
//					if(sysTick%10){
//						sprintf(buffer, "BallX %u ",Football.XPos);
//						USART_SendString(USART3,buffer, strlen(buffer));
//						
//						sprintf(buffer, "BallY %u \r\n \r\n",Football.YPos);
//						USART_SendString(USART3,buffer, strlen(buffer));
//					}

		}
	
		switch(StateCar){
			case Wait:
				if(Football.XPos < 660){
					//the football is in range of sight
					LEDOn();
					Cenz_Car.XResting = Cenz_Car.XBPos;
					Cenz_Car.YResting = Cenz_Car.YBPos;
					StateCar = Scan;
					
					if(sysTick%10){
						sprintf(buffer, "\r\n \r\n WAIT MODE %u \r\n \r\n",Football.XPos);
						USART_SendString(USART3,buffer, strlen(buffer));
					}

				}else{
					//do nothing
					LEDToggle();
				}
				break;
				
			case Scan:
				
					if(sysTick%10){
						sprintf(buffer, "\r\n \r\n Scan MODE \r\n \r\n");
						USART_SendString(USART3,buffer, strlen(buffer));
					}
				
			
				isballCarScan();
				break;
				
			case Go:
					//Scan the ball
					//rotate until the Car is well oriented with the ball
				CarGo();
				
				break;
			case Hit:
				
					LEDToggle();
			
				break;
			case Miss:
				break;
			case Back:
				break;
			default:
				motorOutput(0, 0, R_isForward,L_isForward);
		}								
		
	}
}


void TIM4_IRQHandler(void) {
	//Left counter
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET){	
		current_Left_counter++;		
		
//		sprintf(buffer, "L %u",current_Left_counter);
//		USART_SendString(USART3,buffer, sizeof(buffer));
		
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);		
	}
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
}

void TIM2_IRQHandler(void) {
	//RIGHT counter
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){
		current_Right_counter++;

//		sprintf(buffer, "R %u\r\n", current_Right_counter);
//		USART_SendString(USART3,buffer, sizeof(buffer));
		
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}                                                 
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}


void SysTick_Handler(){
	//10000 -> 01ms
	sysTick++;
	//update the R_Speed and L_Speed --> insert data
}



void motorOutput(int32_t R_SPEED, int32_t L_SPEED, BitAction RB_F, BitAction LB_F){			
	//input FORWARD or BACKWARD and speed from 0 to MAX_POWER			
//	
//	if(R_SPEED >0){MOTOR_R(Forward);}else{MOTOR_R(Backward);}
//	if(L_SPEED >0){MOTOR_L(Forward);}else{MOTOR_L(Backward);}
		MOTOR_R(RB_F);
		MOTOR_L(LB_F);
	
		//R
		if(R_SPEED > MAX_POWER){TIM_SetCompare1(TIM3, MAX_POWER); R_Speed = MAX_POWER;LEDOn();}
		else if(R_SPEED == 0){
			TIM_SetCompare1(TIM3, 0); 
		}else{
			TIM_SetCompare1(TIM3, R_SPEED);LEDOff();
		}
		
		//L
		if(L_SPEED > MAX_POWER){
			TIM_SetCompare2(TIM3, MAX_POWER); L_Speed = MAX_POWER;LEDOn();
		}	else if(L_SPEED == 0){
			TIM_SetCompare2(TIM3, 0); 
		}else{
			TIM_SetCompare2(TIM3, L_SPEED);LEDOff();
		}
	
	
}


void clearString(char *pucBuffer){
	unsigned long ulCount = sizeof(pucBuffer);
	while(ulCount){
		ulCount--;
		pucBuffer[ulCount] = '\0';
	}
}

void ESP01_init(){
	
	while(!USART_GetFlagStatus(USART3, USART_FLAG_TC));
	USART_SendData(USART3, 'B'); //only BA is shown
	
	//================clear the buffer==========
	while(!USART_GetFlagStatus(USART2, USART_FLAG_TC));
	USART_SendData(USART2, '\r'); //only BA is shown
	
	while(!USART_GetFlagStatus(USART2, USART_FLAG_TC));
	USART_SendData(USART2, '\n'); //only BA is shown
	//===============init ATCMD===============
	LEDOn();	
	int compareFlag = 1;
	char temp = 0;
	while(compareFlag != 0){
		//AT+CIPSTART="UDP","0",0,3105,2

//USART_SendString(USART3, initESP, strlen(initESP));

//////////		USART_ReceiveString(USART2, receive);		//\r\r\n
//////////		USART_SendString(USART3, receive, strlen(receive));
//////////		
//////////		compareFlag = strcmp(receive,"CONNECT\r\n");
//////////		USART_ReceiveString(USART2, receive); //\r\n
//////////		USART_SendString(USART3, receive, strlen(receive));
		//return 0 if it is correct
					if(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == SET){
					temp = USART_ReceiveData(USART2);
					
					while(!USART_GetFlagStatus(USART3, USART_FLAG_TC));
					USART_SendData(USART3, temp); 
					
					if(temp == '+'){compareFlag = 0;}
				}
				
				if(isSwitchPressed()){
						USART_SendString(USART2, initESP, strlen(initESP));
				}
	}
		
		//delay(10000);

	LEDOff();
	
	//init the int of the usart
	if(compareFlag==0){
		USART2_ITinit();
	}
}


void USART2_IRQHandler() {
	static char usart_stage = 'n';
	static char temp[50] = {'\0'};
	
	if(USART_GetITStatus(USART2, USART_IT_RXNE)!= RESET) {
		
		temp[receive_pointer] = USART_ReceiveData(USART2); 	

		if(temp[receive_pointer] == '\n' ){
			for(uint8_t i = 0; i < 20; i++){	receive[i] = temp[i];	}
			decodeFlag = true; //run the decode message
			receive_pointer = 0; //reset the pointer
		}else{
			receive_pointer++;
		}
		
	}	
	

}

int decode(char num5, char num6, char num8, char num9, char num14, char num15, \
						char num16, char num17, char num18, char num19, char num20, char num21,\
						char num22, char num23, char num24, char num25){
	////CRD001XXXYYYSSS: RED CAR object at (XXX, YYY)
	////+IPD,21:CBE38d0970cc0770aa025
	uint8_t codelen  = CHARDEC2NUM(num5)*10 + CHARDEC2NUM(num6);
	uint32_t xpos = 0;
	uint32_t ypos = 0;
	static bool initB = true;
	static bool initCF = true;
	static bool initCB = true;
	static bool initAng = true;
	static uint8_t flagflag = 1;
	
	static uint8_t input_angle_counter = 0;
	static float input_angle_temp = 0;
	
	//=====two button====
	int64_t xpos1 = 0;
	int64_t xpos2 = 0;
	int64_t ypos1 = 0;
	int64_t ypos2 = 0;
	static bool initpos1 = true;
	static bool initpos2 = true;
	
	//LEDToggle();
	
	if(codelen == 15){
		
		switch(num8){

			case BALL:
				if(num9 == Football.myColor){
				
									 xpos = (CHARHEX2NUM(num14) << 2*4) \
												| (CHARHEX2NUM(num15) << 1*4) \
												| (CHARHEX2NUM(num16) << 0);
									 ypos = (CHARHEX2NUM(num17) << 2*4) \
												| (CHARHEX2NUM(num18) << 1*4) \
												| (CHARHEX2NUM(num19) << 0);
								if(initB){Football.XPos = xpos; Football.YPos = ypos; initB = false;}
								if( abs((int64_t)Football.XPos - (int64_t)xpos) < 200){Football.XPos = xpos;}
								if( abs((int64_t)Football.YPos - (int64_t)ypos) < 200){Football.YPos = ypos;}
					
				}else{ return 0; }
				
				break;
			
			case CAR:
				if(num9 == Cenz_Car.myFColor){
				
									xpos = (CHARHEX2NUM(num14) << 2*4) \
												| (CHARHEX2NUM(num15) << 1*4) \
												| (CHARHEX2NUM(num16) << 0);
									ypos = (CHARHEX2NUM(num17) << 2*4) \
												| (CHARHEX2NUM(num18) << 1*4) \
												| (CHARHEX2NUM(num19) << 0);
					if(initCF){Cenz_Car.XFPos = xpos; Cenz_Car.YFPos = ypos; initCF = false;}
					if( abs((int64_t)Cenz_Car.XFPos - (int64_t)xpos) < 200){Cenz_Car.XFPos = xpos;}
					if( abs((int64_t)Cenz_Car.YFPos - (int64_t)ypos) < 200){Cenz_Car.YFPos = ypos;}
				}
				
				if(num9 == Cenz_Car.myBColor){
				
									xpos = (CHARHEX2NUM(num14) << 2*4) \
												| (CHARHEX2NUM(num15) << 1*4) \
												| (CHARHEX2NUM(num16) << 0);
									ypos = (CHARHEX2NUM(num17) << 2*4) \
												| (CHARHEX2NUM(num18) << 1*4) \
												| (CHARHEX2NUM(num19) << 0);
					if(initCB){Cenz_Car.XBPos = xpos; Cenz_Car.YBPos = ypos; initCB = false;}
					if( abs((int64_t)Cenz_Car.XBPos - (int64_t)xpos) < 200){Cenz_Car.XBPos = xpos;}
					if( abs((int64_t)Cenz_Car.YBPos - (int64_t)ypos) < 200){Cenz_Car.YBPos = ypos;}
				}
				
				if((Cenz_Car.XFPos - Cenz_Car.XBPos) > 0){
						

					input_angle_temp = ANGLE(Cenz_Car.XFPos,Cenz_Car.YFPos,Cenz_Car.XBPos,Cenz_Car.YBPos);
////					input_angle_temp = (float)atan((float)((int64_t)Cenz_Car_Front.YPos - (int64_t)Cenz_Car_Back.YPos)\
////															/(float)((int64_t)Cenz_Car_Front.XPos - (int64_t)Cenz_Car_Back.XPos));
				//if(initAng){Cenz_Car_Angle = input_angle_temp;initAng = false;}
				//Cenz_Car_Angle = (Cenz_Car_Angle*2.0 + input_angle_temp)/3.0;
				//input_angle_counter++;
				
						Cenz_Car.myAngle = input_angle_temp;
						Cenz_Car.XPos = (Cenz_Car.XBPos + Cenz_Car.XFPos)/2;
						Cenz_Car.YPos = (Cenz_Car.YBPos + Cenz_Car.YFPos)/2;
						
//				if(isSwitchPressed()){
//////////////						sprintf(buffer, "YYXX=%u %u %u %u \r\n", Cenz_Car_Front.YPos, Cenz_Car_Back.YPos, Cenz_Car_Front.XPos,Cenz_Car_Back.XPos);
//////////////						USART_SendString(USART3,buffer, sizeof(buffer));		
//////////////						sprintf(buffer, "%f \r\n", input_angle_temp);
//////////////						USART_SendString(USART3,buffer, sizeof(buffer));
//				}
				break;
				}
			
//	if(isSwitchPressed()){
//			sprintf(buffer, "bc?%c x=%u y=%u \r\n", 'C', Football.XPos, Football.YPos);
//			USART_SendString(USART3,buffer, sizeof(buffer));
//	}		
	
	decodeFlag = false;
	}
	
//////////////////////////////////////////////////	if(codelen == 21){
//////////////////////////////////////////////////	
//////////////////////////////////////////////////		if(num8 == CAR && num9 == Cenz_Car.myColor){
//////////////////////////////////////////////////				xpos1 = (CHARHEX2NUM(num14) << 2*4) \
//////////////////////////////////////////////////							| (CHARHEX2NUM(num15) << 1*4) \
//////////////////////////////////////////////////							| (CHARHEX2NUM(num16) << 0);
//////////////////////////////////////////////////				ypos1 = (CHARHEX2NUM(num17) << 2*4) \
//////////////////////////////////////////////////							| (CHARHEX2NUM(num18) << 1*4) \
//////////////////////////////////////////////////							| (CHARHEX2NUM(num19) << 0);
//////////////////////////////////////////////////			
//////////////////////////////////////////////////				xpos2 = (CHARHEX2NUM(num20) << 2*4) \
//////////////////////////////////////////////////							| (CHARHEX2NUM(num21) << 1*4) \
//////////////////////////////////////////////////							| (CHARHEX2NUM(num22) << 0);
//////////////////////////////////////////////////				ypos2 = (CHARHEX2NUM(num23) << 2*4) \
//////////////////////////////////////////////////							| (CHARHEX2NUM(num24) << 1*4) \
//////////////////////////////////////////////////							| (CHARHEX2NUM(num25) << 0);			
//////////////////////////////////////////////////			
//////////////////////////////////////////////////				flagflag = ((xpos1 - xpos2) > 0)?1:2;
//////////////////////////////////////////////////						
//////////////////////////////////////////////////			if(initpos1){Cenz_Car.XFPos = __MAX(xpos1,xpos2); Cenz_Car_Back.YPos = ypos; initpos1 = false;}
//////////////////////////////////////////////////			if(initpos2){Cenz_Car_Back.XPos = xpos; Cenz_Car_Back.YPos = ypos; initpos1 = false;}
//////////////////////////////////////////////////			
//////////////////////////////////////////////////			
//////////////////////////////////////////////////			if(!initpos1){
//////////////////////////////////////////////////					if(abs(Cenz_Car.XFPos - __MAX(xpos1,xpos2)) > 50){flagflag = 3;} // ignore all the data
//////////////////////////////////////////////////			}
//////////////////////////////////////////////////			initpos1 = false;
//////////////////////////////////////////////////			if(!initpos2){
//////////////////////////////////////////////////				if(abs(Cenz_Car.XBPos - __MIN(xpos1,xpos2)) > 50){flagflag = 3;} // ignore all the data
//////////////////////////////////////////////////			}
//////////////////////////////////////////////////			initpos2 = false;
//////////////////////////////////////////////////			
//////////////////////////////////////////////////			switch(flagflag){
//////////////////////////////////////////////////				
//////////////////////////////////////////////////					case 1:
//////////////////////////////////////////////////						Cenz_Car.XFPos = xpos1;
//////////////////////////////////////////////////						Cenz_Car.YFPos = ypos1;
//////////////////////////////////////////////////						
//////////////////////////////////////////////////						Cenz_Car.XBPos = xpos2;
//////////////////////////////////////////////////						Cenz_Car.YBPos = ypos2;
//////////////////////////////////////////////////						
//////////////////////////////////////////////////						break;
//////////////////////////////////////////////////					case 2:
//////////////////////////////////////////////////						
//////////////////////////////////////////////////						Cenz_Car.XFPos = xpos2;
//////////////////////////////////////////////////						Cenz_Car.YFPos = ypos2;
//////////////////////////////////////////////////						
//////////////////////////////////////////////////						Cenz_Car.XBPos = xpos1;
//////////////////////////////////////////////////						Cenz_Car.YBPos = ypos1;
//////////////////////////////////////////////////						
//////////////////////////////////////////////////						break;
//////////////////////////////////////////////////					default:
//////////////////////////////////////////////////						break;
//////////////////////////////////////////////////				
//////////////////////////////////////////////////				}
//////////////////////////////////////////////////		
//////////////////////////////////////////////////				input_angle_temp = (float)atan((float)((int64_t)Cenz_Car.YFPos - (int64_t)Cenz_Car.YBPos)\
//////////////////////////////////////////////////													/(float)((int64_t)Cenz_Car.XFPos - (int64_t)Cenz_Car.YFPos ));
//////////////////////////////////////////////////				//if(initAng){Cenz_Car_Angle = input_angle_temp;initAng = false;}
//////////////////////////////////////////////////				//Cenz_Car_Angle = (Cenz_Car_Angle*2.0 + input_angle_temp)/3.0;
//////////////////////////////////////////////////				
////////////////////////////////////////////////////				if(isSwitchPressed()){
////////////////////////////////////////////////////////////////////////////////						sprintf(buffer, "YYXX=%u %u %u %u \r\n", Cenz_Car.YFPos, Cenz_Car_Back.YPos, Cenz_Car_Front.XPos,Cenz_Car_Back.XPos);
////////////////////////////////////////////////////////////////////////////////						USART_SendString(USART3,buffer, sizeof(buffer));
//////////////////////////////////////////////////				
//////////////////////////////////////////////////						sprintf(buffer, "%f \r\n", input_angle_temp);
//////////////////////////////////////////////////						USART_SendString(USART3,buffer, sizeof(buffer));
//////////////////////////////////////////////////				
////////////////////////////////////////////////////				}
//////////////////////////////////////////////////				
//////////////////////////////////////////////////		}
	

		return 1;
	}
}

void delay(int t) {
	t = t*1000;
	int i, j;
	for(i=0; i<t; i++)
		if(i%100000 == 0){
			LEDToggle();
		}
		j++;
}

		
void Control(){
//	Cenz_Car.XPos
//	Cenz_Car.YPos
	
	static float Kp_Y = 0.1;
	static float Kp_X = 0.1;
	
	
	int64_t delta_XPos = Football.XPos - Cenz_Car.XPos;
	int64_t delta_Ypos = Football.YPos - Cenz_Car.YPos;
	
	if(delta_Ypos > 0){
		left_counter_mark = Kp_Y * delta_Ypos;	
	}else{
		right_counter_mark = Kp_X * delta_Ypos;
	}
	
	if(delta_XPos > 0){
		right_counter_mark += delta_XPos;
		left_counter_mark += delta_XPos;
	}
	

}

void setCounterMark(int16_t LeftMark, int16_t RightMark){
	if(LeftMark < 0){MOTOR_L(Backward);}else{MOTOR_L(Forward);}
	if(RightMark < 0){MOTOR_R(Backward);}else{MOTOR_R(Forward);}
	
	left_counter_mark = current_Left_counter + 5*abs(LeftMark);
	right_counter_mark = current_Right_counter + 5*abs(RightMark);
}


void Control_wheel(){
	
	//===============declare local variable=================
	static int64_t LFlagtimerMark = 0;
	static int64_t RFlagtimerMark = 0;
	
	static bool isLMotorReady = true;
	static bool isRMotorReady = true;
	

	static int64_t current_errorTime = 0;
	static int64_t previous_errorTime = 0;
	static int64_t derivative_errorTime = 0;
	static int64_t integral_errorTime = 0;
	static int64_t steady_current_error = 0;
	static int64_t steady_previous_error = 0;
	static int64_t steady_derivative = 0;
	
	//the error is arround 9
	float Kp_error = 0.75;
	float Ki_error = 0.0;
	float Kd_error = 0.0;
	float Ks_error = 0.0;
	float Ksd_error = 0.0;
	
	static int64_t L_SpeedAdj = 2100;
	static int64_t R_SpeedAdj = 2100;
	
	static bool LfirstFlag = false;
	static bool RfirstFlag = false;
	
		if((int64_t)(right_counter_mark - current_Right_counter) > 0){
			//===============RIGHT=======================//
			R_Speed  = R_SpeedAdj;
			isRMotorReady = false;
			RfirstFlag = false;
		} else {
			//stop
			R_Speed = 0; 
			isRMotorReady = true;
			if(!RfirstFlag){
				RFlagtimerMark = sysTick;
				RfirstFlag = true;
			}
		}
			
		if((int64_t)(left_counter_mark - current_Left_counter) > 0){
			//================LEFT=====================//
			L_Speed  = L_SpeedAdj;
			isLMotorReady = false;
			LfirstFlag = false;
		}	else {
			//stop
			L_Speed = 0;
   		isLMotorReady = true;
			
			if(!LfirstFlag){
				LFlagtimerMark = sysTick;
				LfirstFlag = true;
			}
		}	
		
		
		//The faster will have the smaller value since the clock is counting
		if(isRMotorReady&&isLMotorReady){
			current_errorTime = (LFlagtimerMark - RFlagtimerMark);
			
			//steady_current_error = (LSetMark - RSetMark) - (current_Left_counter - current_Right_counter);
			steady_derivative = steady_current_error - steady_previous_error;
			
			//integral_errorTime = (LSetMark - RSetMark) - ();
			integral_errorTime += current_errorTime;
			integral_errorTime = (abs(current_errorTime) < 100)?0:integral_errorTime+current_errorTime;
			if(abs(current_errorTime) > 150){integral_errorTime = (integral_errorTime>0)?150:-150;}
			
			
			derivative_errorTime = current_errorTime - previous_errorTime;
			
			int64_t PID_errorTime =  (Kp_error*current_errorTime \
															+ Ki_error* integral_errorTime\
															+ Kd_error*derivative_errorTime\
															+ Ks_error*steady_current_error\
															+ Ksd_error*steady_derivative);
			
			L_SpeedAdj += PID_errorTime; 
			R_SpeedAdj -= PID_errorTime;
			
			//restore the speed due to round down error =.=
			//L_SpeedAdj += control_speed - (L_SpeedAdj+R_SpeedAdj)/2;
			//R_SpeedAdj += control_speed - (L_SpeedAdj+R_SpeedAdj)/2;
			
			previous_errorTime = current_errorTime;
			steady_current_error = steady_previous_error;
			//if the errorTime is a negative num -> indicate that L is faster than R
			//if the errorTime is a positive num -> indicate that R is faster than L
		}
			//		if(sysTick%1000 == 0){
//			
////			sprintf(buffer, "LSA %i ",integral_errorTime);
////			USART_SendString(USART3,buffer, strlen(buffer));
////			
////			sprintf(buffer, "RSA %i ",R_SpeedAdj);
////			USART_SendString(USART3,buffer, strlen(buffer));
//			
////			sprintf(buffer, "LSA %i ",L_SpeedAdj);
////			USART_SendString(USART3,buffer, strlen(buffer));
//		}
			


}





bool isballCarScan(){
	motorOutput(2200,2200,Forward, Backward);
	
	static PID rotation;
	
	rotation.Kp = 0.5;
	rotation.Ki = 0;
	rotation.Kd = 0;
	
	rotation.target_value = BallCarAngle;
	
	rotation.current_error = rotation.target_value - Cenz_Car.myAngle;
	
	rotation.integral_error += rotation.current_error;
	
	if(rotation.current_error == 0){rotation.integral_error = 0;}
	
	if(abs(rotation.current_error) > rotation.error_max){rotation.integral_error = 0;}
	
	rotation.derivative_error = rotation.current_error - rotation.previous_error;
	
	//LSetMark += rotation.Kp*rotation.current_error + rotation.Ki*rotation.integral_error + rotation.Kd*rotation.derivative_error;	
	
	if(abs(rotation.current_error) < 0.1){
		LEDOn();
		motorOutput(0, 0, Forward,Forward);
		StateCar = Go;
		return true;
	}
	
	
	
	//rotate clockwise
//	LSetMark = 1;
//	RSetMark = -1;
//	setCounterMark(LSetMark,RSetMark);
	//Control_wheel();
	
	return false;
}

void CarGo(){
	static int RSpeed = 2200;
	static int LSpeed = 2200;
	
	LEDOn();
	
	PID CarGoForward;
	
	CarGoForward.Kp = 0.01;
	
	CarGoForward.target_value = BallCarAngle;
	
	CarGoForward.current_error = CarGoForward.target_value - Cenz_Car.myAngle;
	
	LSpeed += CarGoForward.Kp*CarGoForward.current_error;
	RSpeed -= CarGoForward.Kp*CarGoForward.current_error;
	
	motorOutput(RSpeed,LSpeed,Forward,Forward);
	
	if(Cenz_Car.XPos > Football.XPos){
		StateCar = Miss;
	}
	
  int Xdelta = (Cenz_Car.XPos - Football.XPos);
	int Ydelta = (Cenz_Car.YPos - Football.YPos);
	
	if(Xdelta < 70 && Ydelta < 70){
		StateCar = Hit;
	}
	
	if(BallCarAngle > 0.1){

	}else{
		//Control_wheel();
	}

}



