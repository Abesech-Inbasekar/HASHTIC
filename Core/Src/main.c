/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can b found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"

//#include "stdlib.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define gain1      0.018
#define offset1    1.0995
#define gain2      0.0006     //0.0006
#define offset2     0.0195    //-0.0982
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int counter = 0;
int counterir = 0;

uint32_t SpeedTime2=0;
int k=0;
int a =0;

int sum_count=0;
float dist=0;

float R1=10.0;
float R2=0.47;

float voltage_input;

float dist1=0;
//uint8_t Receive;
uint32_t targetSpeed;

uint32_t SpeedTime=0;

uint32_t SpeedTime1=0;

float rps;
int POD_ID ;
//int POD_ID = 345;
uint16_t speed_dummy = 5656;
float speed_enc;
//uint8_t ACK = 0;

uint8_t Receive[11];
int spd=0;
uint8_t sed[4] ={0};
int  FinalTime;

uint8_t Time[2] ={0};
uint8_t Pod[3];


__IO uint32_t MotorTimeFed = 0;

__IO uint32_t MotorTimeBack = 0;

__IO uint32_t PreviousTime = 0;

__IO uint32_t PrintTime = 0;

__IO uint32_t EnergyTime = 0;

extern __IO uint32_t LocalTime;

int Rx= 0;
int flag = 0;
int a;
int b;
int c;

int t=0;

int i=0;

float A=0.0;
float lambda=0.9;

int count = 0;
int d = 0;
int e = 0;
int cnt[20];
uint32_t u =0;
char buffer[20];

int j=0;
int s = 0;
int q = 0;
int v =0;
int p =0;

int distance=0;
float dis = 0;
int speed = 0;
//to send to server:
float Energy = 0;
int TrackVoltage = 0;
int POD_Direction = 0;
uint8_t TrackingSwitching = 0;
//int temperature = 0;
int I_current = 8;
int slope = 0;
int distravelled=0;
//jonathan variables
float distanceCovered = 0;
static uint32_t countSum = 0;
//ADC variable
uint16_t value[2];
volatile int adc_conv_cplt = 0;
float voltagePC0;
float currentPC1;
float I;
float Vdc = 0;
float Idc = 0;
float Power = 0;
float Energy1 = 0;
//
float rawCurrent = 0;
uint8_t sampleCurrent = 0;
float Current =0;
uint8_t ACK = 0;
int increment = 0;

//speed loop

//float error;
//uint8_t targetSpeed;
 //float ServerSpeed = 0;
// IR sensor
uint8_t debounce_state = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart4, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//char msg[30];
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */




	HAL_Init();

  /* USER CODE BEGIN Init */


  /* USER CODE END Init */





  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_TIM3_Init();
  MX_TIM15_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_UART_Receive_IT(&huart4, Receive, 11);
  HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1);
  HAL_I2C_Mem_Read(&hi2c1, FRAM_ADDRESS,ENERGY_ADDRESS,I2C_MEMADD_SIZE_16BIT, (uint8_t *) &Energy, sizeof(Energy), 10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //TIM1->CCR2 = 2500;

  while (1)
  {


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 	//  HAL_UART_Transmit_IT(&huart4, (uint32_t *) &speed, 10);

	/* if(a == 1)
	 {
	  if((LocalTime - SpeedTime2) >= 10)
		{
		 TIM3->CCR3 = k;
		 k+=80;
		 SpeedTime2=LocalTime;
		 if(k>=8000)
		 {
			 a = 0;
		 }
		}
	 }*/


	TIM15->CCR1 = s;  //reverse
 TIM1->CCR1 = q;  //forward
   // TIM1->CCR2 = v;  // not using
TIM3->CCR3 = p; // solenoid 0.68 max current*/
	   HAL_ADC_Start_DMA(&hadc1, (uint32_t *)value, 2);
	   Engery_Calulation();

//   if((LocalTime - PreviousTime ) >= 500)
//	 {
//		 if(distance<10){
//			 sprintf(buffer, "%d000%d%d", POD_ID, distance, Rx);
//		 }
//		 else if(distance<100){
//			 sprintf(buffer, "%d00%d%d", POD_ID, distance, Rx);
//  		 }
//		 else if(distance<1000){
//			 sprintf(buffer, "%d0%d%d", POD_ID, distance, Rx);
//		 }
//		 else{
//			 sprintf(buffer, "%d%d%d", POD_ID, distance, Rx);
//		 }
//		 printf("%s\n",buffer);
//		 PreviousTime = LocalTime;
//	 }
//	// printf("%d %d%d\n",a,b,c);
//	//  printf("%d",counter);
//	 printf("%d\n",distance);

 /*  if((LocalTime - PreviousTime ) >= 500)
	   	 {
	   		 if(speed_enc<10){
	   			 if(Energy<10){
	   				 sprintf(buffer, "%d000%d%d00%d%d%d%d%d%d", POD_ID, distance, POD_Status,Energy,TrackVoltage,distravelled,POD_Direction,TrackingSwitching,I_current);
	   			 }
	   			 else if(Energy<100){
	   				 sprintf(buffer, "%d000%d%d0%d%d%d%d%d%d", POD_ID, distance, POD_Status,Energy,TrackVoltage,distravelled,POD_Direction,TrackingSwitching,I_current);


	   			 }
	   			 else if(Energy<1000){
	   				 sprintf(buffer, "%d000%d%d%d%d%d%d%d%d", POD_ID, distance, POD_Status,Energy,TrackVoltage,distravelled,POD_Direction,TrackingSwitching,I_current);
	   			 }
	   		 }

	   		 else if(speed_enc<100){
	   			 if(Energy<10){
	   				 sprintf(buffer, "%d00%d%d00%d%d%d%d%d%d", POD_ID, distance, POD_Status,Energy,TrackVoltage,distravelled,POD_Direction,TrackingSwitching,I_current);
	   			 }
	   			 else if(Energy<100){
	   				 sprintf(buffer, "%d00%d%d0%d%d%d%d%d%d", POD_ID, distance, POD_Status,Energy,TrackVoltage,distravelled,POD_Direction,TrackingSwitching,I_current);
	   			 }
	   			 else if(Energy<1000){
	   				 sprintf(buffer, "%d00%d%d%d%d%d%d%d%d", POD_ID, distance, POD_Status,Energy,TrackVoltage,distravelled,POD_Direction,TrackingSwitching,I_current);
	   			 }
	   		 }
	   		 else if(speed_enc<1000){
	   			 if(Energy<10){
	   				 sprintf(buffer, "%d0%d%d00%d%d%d%d%d%d", POD_ID, distance, POD_Status,Energy,TrackVoltage,distravelled,POD_Direction,TrackingSwitching,I_current);
	   			 }
	   			 else if(Energy<100){
	   				 sprintf(buffer, "%d0%d%d0%d%d%d%d%d%d", POD_ID, distance, POD_Status,Energy,TrackVoltage,distravelled,POD_Direction,TrackingSwitching,I_current);
	   			 }
	   			 else if(Energy<1000){
	   				 sprintf(buffer, "%d0%d%d%d%d%d%d%d%d", POD_ID, distance, POD_Status,Energy,TrackVoltage,distravelled,POD_Direction,TrackingSwitching,I_current);
	   			 }
	   		 }
	   		 else{
	   			 if(Energy<10){
	   				 sprintf(buffer, "%d%d%d00%d%d%d%d%d%d", POD_ID, distance, POD_Status,POD_Status,TrackVoltage,distravelled,POD_Direction,TrackingSwitching,I_current);
	   			 }
	   			 else if(Energy<100){
	   				 sprintf(buffer, "%d%d%d0%d%d%d%d%d%d", POD_ID, distance, POD_Status,Energy,TrackVoltage,distravelled,POD_Direction,TrackingSwitching,I_current);
	   			 }
	   			 else if(Energy<1000){
	   				 sprintf(buffer, "%d%d%d%d%d%d%d%d%d", POD_ID, distance, POD_Status,Energy,Energy,distravelled,POD_Direction,TrackingSwitching,I_current);
	   			 }
	   		 }
	   		 printf("%s\n",buffer);
	   	 }*/


 // Motor_ControlLoop(targetSpeed);

if(flag == 1)
{
	//cnt[j] = counter;
			       countSum+=counter;
			       //ditanceCovered=((float)(countSum*10/20.0)*0.0754)/10.0;
			      distanceCovered=((float)(countSum/20.0)*0.0754); //m/sec
			      //distravelled = distanceCovered*100;
			       // distanceCovered=((float)(countSum/20.0)*0.103);
			   	 //  distance=(int)(distanceCovered * 100);
			   	  //  dis=sum_count*0.0754/20.0;
			   	    j++;
			   	    if(j>20){
			   	    	j=0;
			   	    }
			   	 rps = (float)counter*10/20.0;  //diameter is 27mm
			   //	distanceCovered=(rps*0.0754); //m/sec
			   	    counter = 0;
			   	    // speed = rps*0.0942; //diameter is 29mm
			   	    speed_enc = rps*0.075398;
			   	    //distance = speed_enc * 1000;
	    flag = 0;
}
if( Rx == 1)
 {
	 if((LocalTime - PrintTime ) >= 1000)
		   	 {
			   PrintTime  = LocalTime;
			   increment++;

			   printf("{\"POD_ID\": \"%03d\",\"speed_enc\": \"%01.3f\",\"distanceCovered\": \"%06.3f\",\"Idc\": \"%02.3f\",\"Vdc\": \"%02.3f\",\"Power\": \"%02.3f\",\"Energy\": \"%06.3f\",\"ACK\": \"%d\",\"increment\": \"%d\",\"counterir\": \"%d\",\"spd\": \"04%d\",\"TrackingSwitching\": \"%d\"}\r\n", POD_ID, speed_enc, distanceCovered,Idc,Vdc,Power,Energy,ACK,increment,counterir,spd,TrackingSwitching);
 }

	 if(Receive[10]=='1')
	 {
		 TIM3->CCR3 = 8000;

	 }
	 else
	 {
			 TIM3->CCR3 = 0;
	 }
	 if(Receive[3]=='1') // For forward direction
	  {
	     HAL_TIM_PWM_Stop(&htim15,TIM_CHANNEL_1);
	     TIM15->CCR1 = 0;
	     HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
   // if(i<=ServerSpeed)
	 	 if(i<=spd)
	 	 {
	 	  if((LocalTime - SpeedTime) >= 10)
	 		{
			 TIM1->CCR1 = i;
			 i+=10;
			 SpeedTime=LocalTime;
	 		}
	 	 }
	 //else if(i>=ServerSpeed)
	 	  else if(i>=spd)
	 	   {
	 		if((LocalTime - SpeedTime1) >= 10)
	 		{
	 	     TIM1->CCR1 = i;
			 i-=10;
			 SpeedTime1=LocalTime;
	 		 }
	 	   }
	 	}
	   if((LocalTime - MotorTimeFed) >= FinalTime)
	 	 {
	 		     HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
	 		     TIM1->CCR1 = 0;
	 		     MotorTimeFed = LocalTime;
	 		        Rx=0;
	 		        i=0;
	 		       ACK = 1;
	 	  }

	 	 if(Receive[3]=='0') // For forward direction
	 		 	   {
	 		 		  HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
	 	 	 	 	  TIM1->CCR1 = 0;
	 	 	 	 	 HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1);


	 		 		 if(i<=spd){
	 		 			// while(i<=spd){
	 		 				if((LocalTime - SpeedTime) >= 10)
	 		 				{
	 		 					TIM15->CCR1 = i;
	 							 i+=10;
	 							 SpeedTime=LocalTime;
	 		 				}


	 		 		 }
	 		 		 else if(i>=spd)
	 		 		 {
	 		 			// while(i>=spd){
	 		 				if((LocalTime - SpeedTime1) >= 10)
	 		 				{
	 		 					TIM15->CCR1 = i;
	 							 i-=10;
	 							 SpeedTime1=LocalTime;
	 		 			    }

	 		 		 }
	 		 		 }
	 	 if((LocalTime - MotorTimeBack) >= FinalTime)
	 		 	  {
	 		 		     HAL_TIM_PWM_Stop(&htim15,TIM_CHANNEL_1);
	 		 		   TIM15->CCR1 = 0;
	 		 		   MotorTimeBack = LocalTime;
	 		 		        Rx=0;
	 		 		        i=0;
	 		 		       ACK = 1;
	 		 	  }
 }
}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 39;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if(adc_conv_cplt == 0)
	{
		Read_Adc();
	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
if(GPIO_Pin == GPIO_PIN_6)
{
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)== GPIO_PIN_SET)
	{
		counter++;

	}
	//__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
}

  if(GPIO_Pin == GPIO_PIN_7 && (debounce_state == 1))
  {

    	 HAL_TIM_Base_Start_IT(&htim6);
    	 debounce_state = 0;
			//counterir++;


	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    flag=1;

    if(htim == &htim6)
    {
    	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)== GPIO_PIN_RESET)
    	{
    		debounce_state = 1;
    		counterir++;
    		HAL_TIM_Base_Stop_IT(&htim6);
    	}

    }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	Rx = 1;
	HAL_UART_Receive_IT(&huart4, Receive, 11);

         for(int k=0; k<3; k++)
         {
        	 Pod[k] = Receive[k];
         }
          POD_ID = atoi(Pod);
	     for(int i=3;i<7;i++)
		 {
		   sed[i-3] = Receive[i+1];

		  }
		  spd = atoi(sed);
		  //spd/=100;
		  //ServerSpeed = 31504*spd + 1055.2;

		  for(int j=7;j<9;j++)
		  {
		     Time[j-7] = Receive[j+1];
		  }
		   FinalTime = atoi(Time);
		   FinalTime*=1000;
		   MotorTimeFed = LocalTime;
		   MotorTimeBack = LocalTime;
		 // memset(Receive,'\0',11);
}
void Read_Adc()
{
	 // HAL_ADC_Start_DMA(&hadc1, (uint32_t *)value, 2);

		      //if(adc_conv_cplt == 1)
		    //  {
		    	  static uint16_t sample_count = 0;
	              sample_count++;
		     	  HAL_ADC_Stop_DMA(&hadc1);
		     	  voltagePC0=((float)value[0])/4095*3.3;
		     	  Vdc = value[0]*gain1 + offset1;//
		     	 //TrackVoltage = (int)Vdc;
		     	  currentPC1=((float)value[1])/4095*3300;
		     	  rawCurrent+=currentPC1;
		     	  if(sample_count == 1000)
		     	  {
		     		  sample_count = 0;
		     		 adc_conv_cplt = 1;

		     	   // sampleCurrent = 0;
		     	  }
		     	 // I = currentPC1/1.67;
		     	 // voltage and current calibration

}
void Engery_Calulation()
{
	                  if(adc_conv_cplt == 1)
		   		     	  {
		   		     		 Current=(rawCurrent/1000);
		   		     		 Idc = (Current*gain2) + offset2;
		   		     		rawCurrent = 0;
		   		     	     adc_conv_cplt = 0;

		   		     	  }
	                  if(Idc < 0)
			     	  {
			     		  Idc = 0;
			     	 }
			     	  if(Idc > 0)
			     	  {
			     		 Power =  Vdc*Idc;
			     		 Energy = Energy + ((Power/1000)*((LocalTime - EnergyTime)/1000))/3600;

			     	  }
			     	 if((LocalTime - EnergyTime ) >= 60000)
			     	  {
			     		EnergyTime  = LocalTime;
			     	    HAL_I2C_Mem_Write(&hi2c1, FRAM_ADDRESS,ENERGY_ADDRESS,I2C_MEMADD_SIZE_16BIT, (uint8_t *) &Energy, sizeof(Energy), 10);

			     	  }
			     	 //  voltage_input=voltagePC0*(R1+R2)/R2;
			     	 //Energy = Power*t
}
// closed loop
//void Motor_ControlLoop(targetSpeed)
//{
//    error=targetSpeed-speed;
//
//    if(error>0){
//
//
//		if(LocalTime - SpeedTime >= 10)
//			{
//			TIM1->CCR1 = i;
//			i+=10;
//			SpeedTime=LocalTime;
//			}
//		error=targetSpeed-speed;
//		}
//
//    if(error<0){
//
//		if(LocalTime - SpeedTime >= 10)
//			{
//			TIM1->CCR1 = i;
//			i-=10;
//			SpeedTime=LocalTime;
//			}
//		error=targetSpeed-speed;
//		}
//}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
