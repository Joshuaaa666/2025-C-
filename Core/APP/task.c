#include "mybsp.h"
/*---------------------串口重定-----------------向*/
#include <stdio.h>
struct __FILE
{
  int handle;
  /* Whatever you require here. If the only file you are using is */
  /* standard output using printf() for debugging, no file handling */
  /* is required. */
};
/* FILE is typedef'd in stdio.h. */
FILE __stdout;
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart1,(uint8_t*)&ch,1,50);
  /* Your implementation of fputc(). */
  return ch;
}
/*--------------------测量的数据变量区--------------*/
static float D = 0000.0000;
static float X = 0000.0000;
static float I = 0000.0000;
static float P = 0000.0000;
static float P_Max = 0000.0000;



/*--------------------屏幕显量相关变量区--------------*/
//float D_screen = 0000.0000;
//float X_screen = 0000.0000;
//float I_screen = 0000.0000;
//float P_screen = 0000.0000;
uint8_t screen_flag = 0;


/*-----------------串口相关变量--------------------*/
uint8_t openmv_rx_buff[500];
uint8_t openmv_rx_data;
uint16_t openmv_rx_idx = 0;
volatile uint8_t openmv_ready = 0; 


/*------------------外设初始化函数--------------------*/
void peri_init(void)
{
	HAL_UART_Receive_IT(&huart2,&openmv_rx_data,1);
	HAL_TIM_Base_Start_IT(&htim1);
	Circuit_Init();
}
/*------------------LED函数--------------------*/
void led_off(void)
{
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_7,GPIO_PIN_SET);
}

void led_on(void)
{
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_7,GPIO_PIN_RESET);
}


/*-------------------接收中断函数------------------*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart2)
	{
		if(openmv_rx_idx < sizeof(openmv_rx_buff) - 1)
		{
			openmv_rx_buff[openmv_rx_idx++] = openmv_rx_data;
			if(openmv_rx_data == '\n')
			{
				openmv_rx_buff[openmv_rx_idx] = '\0';
				openmv_ready = 1;
			}
		}
		
		HAL_UART_Receive_IT(&huart2, &openmv_rx_data, 1);
	}
}

/*-------------------------处理OpenMv传来的数据函数---------------*/
void rx_proc(void)
{
    static float d_temp = 0.0;
    static float x_temp = 0.0;
	
    
	if(openmv_ready == 1)
	{
		// 进入临界区：禁用串口接收中断
        __HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);
		openmv_ready = 0;
		char temp_copy_buff[50];
		strcpy(temp_copy_buff, (char*)openmv_rx_buff);
        openmv_rx_idx = 0;
        memset(openmv_rx_buff, 0, sizeof(openmv_rx_buff));

        // 退出临界区：重新启用串口接收中断
        __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);


		// 解析数据，这部分耗时操作在临界区之外
		char *token;
		token = strtok(temp_copy_buff,",");
		if(token != NULL)
		{
			d_temp = atof(token);
		}

		token = strtok(NULL,",");
		if(token != NULL)
		{
			x_temp = atof(token);
		}

        // 只有当解析成功才更新全局变量
        if (d_temp > 0.0 || x_temp > 0.0) { // 简单检查，避免0.0的情况
            D = d_temp;
            X = x_temp;
        }
	}
}
	




/*-------------------------发送给屏幕更新测量数据---------------*/
uint32_t screen_tick = 0;
void send_to_screen(void)
{
	if(HAL_GetTick() - screen_tick >= 10)//0.05秒刷新一次数据
	{
		screen_tick = HAL_GetTick();
	  printf("main.t4.txt=\"%.4f\"\xff\xff\xff",D);
	  printf("main.t5.txt=\"%.4f\"\xff\xff\xff",X);
	  printf("main.t6.txt=\"%.4f\"\xff\xff\xff",I);
	  printf("main.t7.txt=\"%.4f\"\xff\xff\xff",P);
		printf("main.t13.txt=\"%.4f\"\xff\xff\xff",P_Max);
	}
 
}


/*-----------------------发送命令给openmv开始启动----------*/
char mes[50];
uint8_t trans_flag = 0;
void openmv_task1(void)
{
	sprintf(mes,"A");
	HAL_UART_Transmit(&huart2,(uint8_t*)mes,strlen(mes),50);
}
void openmv_task2(void)
{
	sprintf(mes,"B");
	HAL_UART_Transmit(&huart2,(uint8_t*)mes,strlen(mes),50);
}
void openmv_task3(void)
{
	sprintf(mes,"3");
	HAL_UART_Transmit(&huart2,(uint8_t*)mes,strlen(mes),50);
}



/*--------------------------按键检测函数--------------------*/
unsigned char key_value = NO_KEY;

uint32_t key_tick = 0;


uint8_t get_key(void)
{
	uint8_t temp = 0;
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == 0) temp = 1;
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1) == 0) temp = 2;
	return temp;
}

 
/*--------------------------定时器中断处理函数--------------------*/
uint8_t key_val,key_down,key_old;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim1)
	{
		key_tick = HAL_GetTick();
	 Keypad_FSM_Task();
	 Key_Event_Type event = Keypad_Get_Event(&key_value);
	 if(key_value == '3')
	 {
		 key_value = 0x00;
		 trans_flag = 3;
	 }		 
			
		key_val = get_key();
		if(key_val == 1)
		{
			trans_flag = 1;
		}
		
		if(key_val == 2)
		{
			trans_flag = 2;
		}
	}
	}

/*--------------------------电压采集函数--------------------*/
uint32_t adc_tick = 0;
void adc_proc(void)
{
	
	if(HAL_GetTick() - adc_tick >= 50)
	{
	adc_tick = HAL_GetTick();
	I = Circuit_GetMultiCurrent();
	P=Circuit_GetMultiPower();
	double temp;
	int repeat_times=50;
	for(int i=0;i<repeat_times;i++)
	{
		temp = Circuit_GetSinglePower();
		P_Max = temp>P_Max ? temp : P_Max;
	}
	}
}
void task_start(void)
{

	rx_proc();
	adc_proc();
	send_to_screen();
	if(trans_flag == 1) 
	{
		trans_flag = 0;
		openmv_task1();
	}
	if(trans_flag == 2) 
	{
		trans_flag = 0;
		openmv_task2();
	}
	if(trans_flag == 3) 
	{
		trans_flag = 0;
		openmv_task3();
	}
	
}