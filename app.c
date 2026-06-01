#include "app.h"
#include "bluetooth.h"
int16_t encoder_l,encoder_r;
uint8_t encoder_read_time;
uint8_t mode = 0;//模式
uint8_t start_flag = 0;//确定
uint8_t receiveData[32];

void key_proc()
{
	static uint8_t key_val = 0, key_down = 0, key_old = 0;
	key_val = key_read();
	key_down = key_val & (key_val ^ key_old);
	key_old = key_val;
	switch (key_down) {
			case 1:
					led2_turn();
					buzzer_beep(50);
					mode++;
					if(mode >= 4)
					{
						mode = 0;
					}
			break;
			case 2:
				buzzer_beep(50);
				 start_flag = 1;
			break;
		}
}
void app_init(void)
{
	led1_on();
	
	OLED_Init();
	OLED_Clear();
	OLED_ShowString(0,0,"en_r:",OLED_8X16);
	OLED_ShowString(0,20,"en_l:",OLED_8X16);
	OLED_ShowString(0,40,"MODE:",OLED_8X16);
	OLED_Update ();
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);		// motor1 init -- L
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);		// motor2 init -- R
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // encoder1 init -- L
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // encoder2 init -- R
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, receiveData, sizeof(receiveData));
	speed_pid_init();
	turn_pid_init();
	buzzer_beep(100);
	printf("小车初始化完成");
}

void app_loop(void)
{
	key_proc();
	
	
	OLED_ShowSignedNum(60,0,encoder_l,3,OLED_8X16);
	OLED_ShowSignedNum(60,20,encoder_r,3,OLED_8X16);
	OLED_ShowNum(60,40,mode,1,OLED_8X16);
	OLED_Update ();
	if(start_flag)
    {
        switch(mode)
        {
            case 0:
                taskone_process(30);//1号病房
            break;

            case 1:
                tasktwo_process(30);//2号病房
            break;

            case 2:
               taskthree_process(30,50,-20);//3号病房
            break;

            case 3:
               taskthree_process(30,-60,20);//4号病房
            break;
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM6) {
		if (++encoder_read_time >= 30) {
			encoder_read_time = 0;
			
			encoder_l=read_encoder_l();
			encoder_r=read_encoder_r();
		}
		buzzer_task();
	}
}

struct __FILE
{
  int handle;
  /* Whatever you require here. If the only file you are using is */
  /* standard output using printf() for debugging, no file handling */
  /* is required. */
};
/* FILE is typedef’d in stdio.h. */
FILE __stdout;
int fputc(int ch,FILE *f)
{
	uint8_t temp[1] = {ch};
	
	//采用轮询方式发送1字节数据
	HAL_UART_Transmit(&huart3,temp,1,2);
	return ch;
}

int ferror(FILE *f)
{
  /* Your implementation of ferror(). */
  return 0;
}
