#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "lcd.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
/*------------------------------------------------------------

Before writing the function, please make sure in stm32f10x_conf.h, the following #include lines are uncommented
			"stm32f10x_gpio.h"
			"stm32f10x_fsmc.h" 
			"stm32f10x_rcc.h" 
			"stm32f10x_adc.h"
			"misc.h" 
-------------------------------------------------------------*/

#define CLK1 GPIO_Pin_4 // PA.4
#define DATA1 GPIO_Pin_5 // PA.5
#define CLK2 GPIO_Pin_0 // PB.0
#define DATA2 GPIO_Pin_1 // PB.1
#define CLK3 GPIO_Pin_10 // PC.10
#define DATA3 GPIO_Pin_11 // PC.11
#define MOTOR1 GPIO_Pin_12 // PC.12 for food
#define MOTOR2 GPIO_Pin_9 // PC.9 for cat teaser
#define PUMP1 GPIO_Pin_2 // PA.2 for fountain
#define PUMP2 GPIO_Pin_3 // PA.3 for adding water
#define IS GPIO_Pin_6 // PC.6 for infared sensor
#define BS GPIO_Pin_7 // PC.7 for beam sensor
#define BT_Address ((uint8_t)0xF4) // address of the bluetooth handler
#define USART_WakeUp_addr ((uint16_t)0xFFF0 | BT_Address) // address mark of the bluetooth handler
#define BT_Received ((uint16_t)0x42) // mark to indicate to the phone
#define NoFood ((uint16_t)0x73) 
#define AddFood ((uint16_t)0x57)
#define StartTeaser ((uint16_t)0x35)
#define StopTeaser ((uint16_t)0x28)

GPIO_InitTypeDef GPIO_InitStructure;
// ADC_InitTypeDef ADC_InitStructure;
USART_InitTypeDef USART_InitStructure;

void Delayus(int duration);
void PS_GPIO_Config(void);
void Motor1_GPIO_Config(void);
void Pump_GPIO_Config(void);
void ISBS_GPIO_Config(void);
void BT_Usart_Config(void);
void test_GPIO_Config(void);

unsigned long GetDataFromPS1(void);
unsigned long GetDataFromPS2(void);
unsigned long GetDataFromPS3(void);

void DriveMotor1(void);
void DriveMotor2(void);
void DrivePump1(void);
void DrivePump2(void);
void FoodAlert(void);
void BT_Usart_WakeUp(void);
void test_main(void);


/*
void ADC_configuration(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	// ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);
	
	// RCC_APB2Periph_ADC1; // enable clock
	RCC_ADCCLKConfig(RCC_PCLK2_Div2);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_55Cycles5);
	ADC_Cmd(ADC1, ENABLE);
	
	// Calibration
	ADC_ResetCalibration(ADC1);
	while (ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1));
}*/

int main(void){	
	//PS_GPIO_Config();
	//Motor1_GPIO_Config();
	//Pump_GPIO_Config();
	//ISBS_GPIO_Config();
	BT_Usart_Config();
	test_GPIO_Config();
	LCD_INIT();
	//unsigned long data1, data2, data3;

	while ( 1 ){
		//DriveMotor1();
		//DriveMotor2();
		//DrivePump1();
		//DrivePump2();
		//FoodAlert();
		//BT_Usart_WakeUp();
		test_main();
	}		
}

void Delayus(int duration){
		while(duration--) 
		{
			int i=0x02;				
			while(i--)
			__asm("nop");
		}
}

void PS_GPIO_Config(void){ 
	
	// initialize PA.4 and PA.5 for the first pressure sensor
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	// initialize clock
	GPIO_InitStructure.GPIO_Pin = CLK1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
	// initialize data
	GPIO_InitStructure.GPIO_Pin = DATA1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOA,&GPIO_InitStructure); 

	// initialzice PB.0 and PB.1 for the second pressure sensor
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	// initialize clock
	GPIO_InitStructure.GPIO_Pin = CLK2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOB,&GPIO_InitStructure); 
	// initialize data
	GPIO_InitStructure.GPIO_Pin = DATA2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOB,&GPIO_InitStructure); 
	
	// initialzice PC.10 and PC.11 for the third pressure sensor
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	// initialize clock
	GPIO_InitStructure.GPIO_Pin = CLK3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOC,&GPIO_InitStructure); 
	// initialize data
	GPIO_InitStructure.GPIO_Pin = DATA3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOC,&GPIO_InitStructure); 

}

unsigned long GetDataFromPS1(void){
	unsigned long temp=0;
	int i;
	GPIO_SetBits(GPIOA, DATA1);
	GPIO_ResetBits(GPIOA, CLK1);
	while (GPIO_ReadInputDataBit(GPIOA, DATA1)){}
	Delayus(1);
	for (i=0; i<24; i++){
		GPIO_SetBits(GPIOA,CLK1);	   //CLK1=1 
		temp=temp<<1; 
		Delayus(1);  
		GPIO_ResetBits(GPIOA, CLK1);	   //CLK1=0 
		if(GPIO_ReadInputDataBit(GPIOA, DATA1))   //DATA1=1 
		temp=temp+1; 
		Delayus(1); 
	}
	GPIO_SetBits(GPIOA, CLK1); 
	temp = temp^0x800000; 
	Delayus(1); 
	GPIO_ResetBits(GPIOA, CLK1); 
	Delayus(1);  
	return temp;
}

unsigned long GetDataFromPS2(void){
	unsigned long temp=0;
	int i;
	GPIO_SetBits(GPIOB, DATA2);
	GPIO_ResetBits(GPIOB, CLK2);
	while (GPIO_ReadInputDataBit(GPIOB, DATA2)){}
	Delayus(1);
	for (i=0; i<24; i++){
		GPIO_SetBits(GPIOB,CLK2);	   //CLK2=1 
		temp=temp<<1; 
		Delayus(1);  
		GPIO_ResetBits(GPIOB, CLK2);	   //CLK2=0 
		if(GPIO_ReadInputDataBit(GPIOB, DATA2))   //DATA2=1 
		temp=temp+1; 
		Delayus(1); 
	}
	GPIO_SetBits(GPIOB, CLK2); 
	temp = temp^0x800000; 
	Delayus(1); 
	GPIO_ResetBits(GPIOB, CLK2); 
	Delayus(1);  
	return temp;
}

unsigned long GetDataFromPS3(void){
	unsigned long temp=0;
	int i;
	GPIO_SetBits(GPIOC, DATA3);
	GPIO_ResetBits(GPIOC, CLK3);
	while (GPIO_ReadInputDataBit(GPIOC, DATA3)){}
	Delayus(1);
	for (i=0; i<24; i++){
		GPIO_SetBits(GPIOC,CLK3);	   //CLK3=1 
		temp=temp<<1; 
		Delayus(1);  
		GPIO_ResetBits(GPIOC, CLK3);	   //CLK3=0 
		if(GPIO_ReadInputDataBit(GPIOC, DATA3))   //DATA3=1 
		temp=temp+1; 
		Delayus(1); 
	}
	GPIO_SetBits(GPIOC, CLK3); 
	temp = temp^0x800000; 
	Delayus(1); 
	GPIO_ResetBits(GPIOC, CLK3); 
	Delayus(1);  
	return temp;
}

void Motor_GPIO_Config(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	// initialize output for motor
	GPIO_InitStructure.GPIO_Pin = MOTOR1 | MOTOR2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOC,&GPIO_InitStructure); 
	// note: this signal will go through an AND gate with 5V to generate 5V for motor.
}

/*
void DriveMotor1(void){
	uint16_t RD = USART_ReceiveData(USART1);
	if ((RD&0x00FF)==(AddFood&0x00FF)){
		GPIO_SetBits(GPIOC, MOTOR1);
		while (GetDataFromPS3()<){}
		GPIO_ResetBits(GPIOC, MOTOR1);
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET){}
		USART_SendData(USART1, BT_Received);
	}
}

void DriveMotor2(void){
	uint16_t RD = USART_ReceiveData(USART1);
	if ((RD&0x00FF)==(StartTeaser&0x00FF)){
		GPIO_SetBits(GPIOC, MOTOR2);
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET){}
		USART_SendData(USART1, BT_Received);
	}
	else if ((RD&0x00FF)==(StopTeaser&0x00FF)) {
		GPIO_ResetBits(GPIOC, MOTOR2);
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET){}
		USART_SendData(USART1, BT_Received);
	}
}

void Pump_GPIO_Config(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	// initialize output for pump1 to form the fountain and pump2 to add water
	GPIO_InitStructure.GPIO_Pin = PUMP1 | PUMP2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
	// note: this signal will go through an AND gate with 5V to generate 5V for pumps.
}
void DrivePump1(void){
	if ((GetDataFromPS1()>/) && (!GPIO_ReadInputDataBit(GPIOC, IS))){
		GPIO_SetBits(GPIOA, PUMP1);
		while (GetDataFromPS1()>){}
		Delayus(5000000);
		GPIO_ResetBits(GPIOA, PUMP1);
	}
}

void DrivePump2(void){
	if (GetDataFromPS2()<){
		GPIO_SetBits(GPIOA, PUMP2);
		while (GetDataFromPS2()<){};
		GPIO_ResetBits(GPIOA, PUMP2);
	}
}

void ISBS_GPIO_Config(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	// initialize output for motor
	GPIO_InitStructure.GPIO_Pin = IS | BS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOC,&GPIO_InitStructure);	
}

void FoodAlert(void){
	if (GPIO_ReadInputDataBit(GPIOC, BS)){
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET){}
		USART_SendData(USART1, NoFood);
	}
}
*/
void BT_Usart_Config(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
	// USART_1_Tx  PA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  //USART1_RX   PA.10  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
  GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
	USART_Init(USART1, &USART_InitStructure);
  // USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); 
  USART_Cmd(USART1, ENABLE);
  // USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE); 
}

void BT_Usart_WakeUp(void){
	USART_SetAddress(USART1, BT_Address);
	USART_WakeUpConfig(USART1, USART_WakeUp_addr);
	GPIO_SetBits(GPIOA, GPIO_Pin_10);
	uint16_t RD = USART_ReceiveData(USART1);
	if ((RD&0x00FF)==(BT_Address&0x00FF)){
		USART_ReceiverWakeUpCmd(USART1, DISABLE);
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET){}
		USART_SendData(USART1, BT_Received);
	}		
}

// test code
/*============================================================
void test_GPIO_Config(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	// initialize clock
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
}

void test_main(void){
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6)){
		GPIO_SetBits(GPIOA, CLK1 | DATA1 | PUMP1 | PUMP2);
		GPIO_SetBits(GPIOB, CLK2 | DATA2);
		GPIO_SetBits(GPIOC, CLK3 | DATA3 | MOTOR1 | MOTOR2);
	}
}

#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "misc.h"
  
int i;
  
//ErrorStatus HSEStartUpStatus;
  
void NVIC_Configuration(void);
void GPIO_Configuration(void);
void USART_Configuration(void);
void USART1_IRQHandler(void);
void UARTSend(const unsigned char *pucBuffer, unsigned long ulCount);
  
int main(void)
{
    usart_rxtx();
    while(1)
    {
  
    }
}
  
/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/
  
/**
  * @brief  This function handles USARTx global interrupt request
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
    if ((USART1->SR & USART_FLAG_RXNE) != (u16)RESET)
    {
        uint16_t i = USART_ReceiveData(USART1);
        if(i == '1'){
            GPIO_WriteBit(GPIOA,GPIO_Pin_8,Bit_SET);        // Set '1' on PA8
            UARTSend("LED ON\r\n",sizeof("LED ON\r\n"));    // Send message to UART1
        }
        else if(i == '0'){
            GPIO_WriteBit(GPIOA,GPIO_Pin_8,Bit_RESET);      // Set '0' on PA8
            UARTSend("LED OFF\r\n",sizeof("LED OFF\r\n"));
        }
    }
}
  
void usart_rxtx(void)
{
    const unsigned char welcome_str[] = " Welcome to Bluetooth!\r\n";
  
    /* Enable USART1 and GPIOA clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
  
    /* NVIC Configuration */
    NVIC_Configuration();
  
    /* Configure the GPIOs */
    GPIO_Configuration();
  
    /* Configure the USART1 */
    USART_Configuration();
  
    /* Enable the USART1 Receive interrupt: this interrupt is generated when the
         USART1 receive data register is not empty */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  
    /* print welcome information */
    UARTSend(welcome_str, sizeof(welcome_str));
}
  
/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports
*******************************************************************************/
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Configure (PA.8) as output */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure); // Save
  
  /* Configure USART1 Tx (PA.09) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* Configure USART1 Rx (PA.10) as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}
  
/*******************************************************************************
* Function Name  : USART_Configuration
* Description    : Configures the USART1
*******************************************************************************/
void USART_Configuration(void)
{
  USART_InitTypeDef USART_InitStructure;
  
/* USART1 configuration ------------------------------------------------------*/
  USART_InitStructure.USART_BaudRate = 9600;        // Baud Rate
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  USART_Init(USART1, &USART_InitStructure);
  
  /* Enable USART1 */
  USART_Cmd(USART1, ENABLE);
}
  
/**
  * @brief  Configures the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
  
/*******************************************************************************
* Function Name  : UARTSend
* Description    : Send a string to the UART.
* Input          : - pucBuffer: buffers to be printed.
*                : - ulCount  : buffer's length
*******************************************************************************/
void UARTSendFunc(const unsigned char *pucBuffer, unsigned long ulCount)
{
    //
    // Loop while there are more characters to send.
    //
    while(ulCount--)
    {
        USART_SendData(USART1, (uint16_t) *pucBuffer++);
        /* Loop until the end of transmission */
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
        {
        }
    }
}
============================================================*/

