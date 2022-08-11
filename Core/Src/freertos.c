/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
#include "event_groups.h"//CAN����������
#include <stdio.h>//ʹ��printf����
#include <string.h>//ʹ�ø������������
#include "stdbool.h"
#include "tim.h"
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
 uint32_t ADC_Value;
 double ADC_True_Value;
 double ADC_Average_Value;
 double Sensor_Data;
 uint16_t ADC_a=0;
 uint16_t Most_num[4095]={0};//
 uint16_t ADC_i=0;
 uint16_t ADC_max=0;
 
 EventGroupHandle_t  TasksEventGroup;//����һ���¼���־��
 EventBits_t TaskEventBits=0;//�����¼���־λ
 
 #define LED0_Show_Bit      (1<<0)
 #define LED1_Show_Bit      (1<<1)
 #define LED2_Show_Bit      (1<<2)
 #define LED3_Show_Bit      (1<<3)
 #define LED4_Show_Bit      (1<<4)
 #define LED5_Show_Bit      (1<<5)
 
 
 uint8_t Senbuff0[] = {0x00};  //�������ݷ�������
 uint8_t Senbuff1[] = {0x01};  //�������ݷ�������
 uint8_t Senbuff2[] = {0x02};  //�������ݷ�������
 uint8_t Senbuff3[] = {0x03};  //�������ݷ�������
 uint8_t Senbuff4[] = {0x04};  //�������ݷ�������
 uint8_t Senbuff5[] = {0x05};  //�������ݷ�������
 
 uint16_t Limit_Level[2]={20,100};//��ż���Һλʱ��������ֵ
 uint16_t SensorData=0;     //ȫ�ֱ���
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for LED_Show */
osThreadId_t LED_ShowHandle;
const osThreadAttr_t LED_Show_attributes = {
  .name = "LED_Show",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for ADC_Recieve */
osThreadId_t ADC_RecieveHandle;
const osThreadAttr_t ADC_Recieve_attributes = {
  .name = "ADC_Recieve",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for Valve_Ctrl */
osThreadId_t Valve_CtrlHandle;
const osThreadAttr_t Valve_Ctrl_attributes = {
  .name = "Valve_Ctrl",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for Data_Process */
osThreadId_t Data_ProcessHandle;
const osThreadAttr_t Data_Process_attributes = {
  .name = "Data_Process",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void LEDShow(void *argument);
void ADCRecieve(void *argument);
void ValveCtrl(void *argument);
void DataProcess(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of LED_Show */
  LED_ShowHandle = osThreadNew(LEDShow, NULL, &LED_Show_attributes);

  /* creation of ADC_Recieve */
  ADC_RecieveHandle = osThreadNew(ADCRecieve, NULL, &ADC_Recieve_attributes);

  /* creation of Valve_Ctrl */
  Valve_CtrlHandle = osThreadNew(ValveCtrl, NULL, &Valve_Ctrl_attributes);

  /* creation of Data_Process */
  Data_ProcessHandle = osThreadNew(DataProcess, NULL, &Data_Process_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
	 TasksEventGroup = xEventGroupCreate();
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_LEDShow */
/**
* @brief Function implementing the LED_Show thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LEDShow */
void LEDShow(void *argument)
{
  /* USER CODE BEGIN LEDShow */
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  /* Infinite loop */
  for(;;)
  {
		TaskEventBits=xEventGroupWaitBits(TasksEventGroup, LED0_Show_Bit|LED1_Show_Bit|LED2_Show_Bit|LED3_Show_Bit|LED4_Show_Bit|LED5_Show_Bit,pdTRUE,pdFALSE,portMAX_DELAY);
			//�������ý�����else���ֵ����ȼ�
			if(TaskEventBits & LED0_Show_Bit)
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);
				for(uint16_t i=0;i<5;i++)
				{
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8, GPIO_PIN_RESET);
					osDelay(500);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8, GPIO_PIN_SET);
					osDelay(500);					
				}
				
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);   //����ģʽ
				HAL_UART_Transmit(&huart1, (uint8_t *)Senbuff0, sizeof(Senbuff0),0XFFFF);
				//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);   //�رշ���ģʽ
				
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13, GPIO_PIN_RESET); //�̵���ͨ�磬���Ŵ�
			}
			
			if(TaskEventBits & LED1_Show_Bit)
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);   //����ģʽ
				HAL_UART_Transmit(&huart1, (uint8_t *)Senbuff1, sizeof(Senbuff1),0XFFFF);
				//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);   //�رշ���ģʽ
			}		
			
			if(TaskEventBits & LED2_Show_Bit)
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9, GPIO_PIN_RESET);	
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);   //����ģʽ
				HAL_UART_Transmit(&huart1, (uint8_t *)Senbuff2, sizeof(Senbuff2),0XFFFF);
				//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);   //�رշ���ģʽ
			}
			
			if(TaskEventBits & LED3_Show_Bit)
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);   //����ģʽ
				HAL_UART_Transmit(&huart1, (uint8_t *)Senbuff3, sizeof(Senbuff3),0XFFFF);
				//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);   //�رշ���ģʽ
			}
			
			if(TaskEventBits & LED4_Show_Bit)
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);   //����ģʽ
				HAL_UART_Transmit(&huart1, (uint8_t *)Senbuff4, sizeof(Senbuff4),0XFFFF);
				//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);   //�رշ���ģʽ
				
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13, GPIO_PIN_SET); //�̵����ϵ磬���Ŵ�
			}
			
			if(TaskEventBits & LED5_Show_Bit)
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET);
				
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);   //����ģʽ
				HAL_UART_Transmit(&huart1, (uint8_t *)Senbuff5, sizeof(Senbuff5),0XFFFF);
				//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);   //�رշ���ģʽ

			}
		
			
			
		if(ADC_RecieveHandle!=NULL) 
			{ 			
				vTaskNotifyGiveFromISR(ADC_RecieveHandle,&xHigherPriorityTaskWoken);  //��������֪ͨ  			
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);  //�����Ҫ�Ļ�����һ�������л� 
			}


  }
  /* USER CODE END LEDShow */
}

/* USER CODE BEGIN Header_ADCRecieve */
/**
* @brief Function implementing the ADC_Recieve thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ADCRecieve */
void ADCRecieve(void *argument)
{
  /* USER CODE BEGIN ADCRecieve */
	uint32_t NotifyValue;
  /* Infinite loop */
  for(;;)
  {
				NotifyValue = ulTaskNotifyTake(pdTRUE, 20219);//��ȡ����֪ͨ
		if(NotifyValue==pdTRUE)
		{
			//printf("Enter ADCRecieve\r\n");
			
			HAL_ADC_Start(&hadc1);//��ADC
			HAL_ADC_PollForConversion(&hadc1,50);
			if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
			{
				ADC_Value = HAL_ADC_GetValue(&hadc1);
				ADC_a = ADC_Value;
				Most_num[ADC_a]++;
			}
			
			//printf("\r\n******** ADC Example ********\r\n");//���ڴ�ӡ
			//printf(" AD1 value = %1.3fV \r\n", ADC_Value*3.3f/4096);
			ADC_True_Value = (double)ADC_Value/28 - 7/16 ;
			//printf(" AD1 value = %1.3f cm\r\n", ADC_True_Value);
			
			ADC_max = Most_num[0];
			
			uint16_t k = 0; //�������Ӧ��������ž��ǳ��ִ���������
			
			for(ADC_i = 0 ; ADC_i < 4096 ; ADC_i++)
			{
				if(ADC_max<Most_num[ADC_i])
					k = ADC_i;
			}
			ADC_Average_Value = (double)k/14 - 7/8 ;
			//printf("ADC_Average_value = %1.3f cm\r\n", ADC_Average_Value);
		//Sensor_Data = ADC_Average_Value;
			
			for(ADC_i = 0 ; ADC_i<4095 ; ADC_i++)//�������,��ֹ��ջ�����Most numi-0;
				 Most_num[ADC_i] = 0;
			
		}
		//printf("ADC_Average_value = %1.3f m\r\n", ADC_Average_Value/10);
    
  }
  /* USER CODE END ADCRecieve */
}

/* USER CODE BEGIN Header_ValveCtrl */
/**
* @brief Function implementing the Valve_Ctrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ValveCtrl */
void ValveCtrl(void *argument)
{
  /* USER CODE BEGIN ValveCtrl */
	uint32_t NotifyValue;
  /* Infinite loop */
  for(;;)
  {
		NotifyValue = ulTaskNotifyTake(pdTRUE, 20219);//��ȡ����֪ͨ
		if(NotifyValue==pdTRUE)
		{
			if(1)
			{
			}
		}
    osDelay(1);
  }
  /* USER CODE END ValveCtrl */
}

/* USER CODE BEGIN Header_DataProcess */
/**lo
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DataProcess */
void DataProcess(void *argument)
{
  /* USER CODE BEGIN DataProcess */
		//BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	uint16_t a=0;
  /* Infinite loop */
  for(;;)
  {
		//printf("ADC_Average_value DDDDDDDDDDDDDDDDDDD = %1.3f cm\r\n", ADC_Average_Value);
		
		a = 50;
		
		//����4���Ϩ����͸����˸5�Σ�����ˮλ����5������
		if(a<=Limit_Level[0]) 
			xEventGroupSetBits(TasksEventGroup, LED0_Show_Bit);//����Ӧ��־λ��1
		else
			xEventGroupClearBits(TasksEventGroup, LED0_Show_Bit);

		//���1����������4��Ϩ��
		if(Limit_Level[0]<a&&a<=(Limit_Level[0]+(Limit_Level[1]-Limit_Level[0])/5))
			xEventGroupSetBits(TasksEventGroup, LED1_Show_Bit);//����Ӧ��־λ��1
		
		else
			xEventGroupClearBits(TasksEventGroup, LED1_Show_Bit);
		
		//���2����������3��Ϩ��
		if((Limit_Level[0]+(Limit_Level[1]-Limit_Level[0])/5)<a&&a<=(Limit_Level[0]+(Limit_Level[1]-Limit_Level[0])*2/5))
			xEventGroupSetBits(TasksEventGroup, LED2_Show_Bit);//����Ӧ��־λ��1
		
		else
			xEventGroupClearBits(TasksEventGroup, LED2_Show_Bit);
		
		//���3����������2��Ϩ��
		if((Limit_Level[0]+(Limit_Level[1]-Limit_Level[0])*2/5)<a&&a<=(Limit_Level[0]+(Limit_Level[1]-Limit_Level[0])*3/5))
			xEventGroupSetBits(TasksEventGroup, LED3_Show_Bit);//����Ӧ��־λ��1
		
		else
			xEventGroupClearBits(TasksEventGroup, LED3_Show_Bit);
		
		//���4����������1��Ϩ��
		if((Limit_Level[0]+(Limit_Level[1]-Limit_Level[0])*3/5)<a&&a<=(Limit_Level[0]+(Limit_Level[1]-Limit_Level[0])*4/5))
			xEventGroupSetBits(TasksEventGroup, LED4_Show_Bit);//����Ӧ��־λ��1
		
		else
			xEventGroupClearBits(TasksEventGroup, LED4_Show_Bit);
		
		//5��ȫ��
		if((Limit_Level[0]+(Limit_Level[1]-Limit_Level[0])*4/5)<a&&a<Limit_Level[1])
			xEventGroupSetBits(TasksEventGroup, LED5_Show_Bit);//����Ӧ��־λ��1
		
		else
			xEventGroupClearBits(TasksEventGroup, LED5_Show_Bit);
		
		osDelay(1);
		
  }
  /* USER CODE END DataProcess */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
