#include "pid.h"



PID_Typedef  speed_pid;//速度环
PID_Typedef  turn_pid; //转向环

CarState car_state = TRACK_LINE;
uint8_t cross_count = 0;
Gray_TypeDef gray_sensor;
//速度环PI控制器初始化
void speed_pid_init(void)
{
	speed_pid.target = 100.0f; // 目标初始速度 80.0f
	
	speed_pid.kp = 0.620;  	  // 比例系数0.590
	speed_pid.ki = 0.026; 	  // 积分系数0.024
	speed_pid.kd = 0.00f; 	  // 微分系数0.00		
	speed_pid.max_out =  80; 	// 输出限幅上限
	speed_pid.min_out = -80; 	// 输出限幅下限
	speed_pid.max_integral = 850; // 积分限幅
	speed_pid.int_error = 0;
	speed_pid.last_error = 0;
	speed_pid.last_actual = 0;
	speed_pid.filter_coefficient = 0.7;  // 滤波系数(越小滤波越强)
	speed_pid.last_filtered_val = 0;
}


//转向环PD控制器初始化
void turn_pid_init(void)
{
   turn_pid.kp = 45.0f;  // 比例系数45.0f; 
   turn_pid.ki = 0.00f;  // 积分系数0.0f;  
   turn_pid.kd = 0.12f; // 微分系数0.15f; 

   turn_pid.max_out =  80;//80
   turn_pid.min_out = -80;//-80
	 
   turn_pid.error = 0;
   turn_pid.last_error = 0;
   turn_pid.int_error = 0;
}

// 速度环PI控制器
// 输入：期望速度、左编码器、右编码器
// 位置式PID实现的速度环控制器
int speed_pid_task(float target_speed, float left_encoder, float right_encoder)
{
	// 左右编码器均值
    float actual_speed = (left_encoder + right_encoder) / 2.0f;
    speed_pid.actual = actual_speed;
		
	// 一阶低通滤波处理误差
    float filtered_speed = speed_pid.filter_coefficient * actual_speed +
                           (1.0f - speed_pid.filter_coefficient) * speed_pid.last_filtered_val;
    speed_pid.last_filtered_val = filtered_speed;
    speed_pid.error = target_speed - filtered_speed;
	
	// 误差积分累加
    speed_pid.int_error += speed_pid.error;
		
	// 积分限幅
    if (speed_pid.int_error > speed_pid.max_integral)
        speed_pid.int_error = speed_pid.max_integral;
    else if (speed_pid.int_error < -speed_pid.max_integral)
        speed_pid.int_error = -speed_pid.max_integral;
		
		// PI输出, 微分项d = 0
    speed_pid.out = speed_pid.kp * speed_pid.error        // p
                  + speed_pid.ki * speed_pid.int_error		// i
									+ speed_pid.kd * (speed_pid.error - speed_pid.last_error); // d
		// 输出限幅
    if (speed_pid.out > speed_pid.max_out)
        speed_pid.out = speed_pid.max_out;
    else if (speed_pid.out < speed_pid.min_out)
        speed_pid.out = speed_pid.min_out;

    speed_pid.last_error = speed_pid.error;
    return (int)speed_pid.out;
}

// 转向环PD控制器
// 位置式PID实现的转向环控制器
int turn_pid_task(float error)
{
	turn_pid.error = error;
	
	turn_pid.out = turn_pid.kp * turn_pid.error +
								 turn_pid.kd * (turn_pid.error - turn_pid.last_error);
	
	if (turn_pid.out > turn_pid.max_out)
			turn_pid.out = turn_pid.max_out;
	else if (turn_pid.out < turn_pid.min_out)
			turn_pid.out = turn_pid.min_out;
	
	turn_pid.last_error = turn_pid.error;
	
	return (int)turn_pid.out;
}
void task_bluetooth_advance(float target_speed)//前进
{
	int speed_output = speed_pid_task(target_speed, encoder_l, encoder_r);
	int pwm_left  = speed_output ;
    int pwm_right = speed_output ;
	motor_setpwm_l(pwm_right +2);
    motor_setpwm_r(pwm_left);
}
void task_bluetooth_back(float target_speed)//后退
{
	int speed_output = speed_pid_task(-target_speed, encoder_l, encoder_r);
	int pwm_left  = speed_output ;
    int pwm_right = speed_output ;
	motor_setpwm_l(pwm_right +2);
    motor_setpwm_r(pwm_left);
}


void taskone_process(float target_speed)//1号病房
{
    gray_read(&gray_sensor);
    
    float track_error = calculate_tracking_error(&gray_sensor);
    
    static uint8_t last_cross = 0;
    uint8_t now_cross = gray_is_T_cross(&gray_sensor);

    // ===== 路口计数（边沿触发）=====
    if(now_cross && !last_cross)
    {
        cross_count++;
    }
    last_cross = now_cross;

    // ===== 新增变量 =====
    static uint16_t delay_cnt = 0;

    switch(car_state)
    {
        // ===============================
        case TRACK_LINE:
        {
            if(cross_count == 1 && now_cross)
            {
                // ? 不是直接转，而是先往前冲一点
                car_state = STRAIGHT_BEFORE_TURN;
                delay_cnt = 0;
            }
        }
        break;

        // ===============================
        case STRAIGHT_BEFORE_TURN:
        {
            delay_cnt++;

            // ? 继续循迹直行一小段（脱离路口中心）
            if(delay_cnt > 0)   // 可调 40~80
            {
                car_state = TURN_LEFT_1;
                delay_cnt = 0;
            }
        }
        break;

        // ===============================
        case TURN_LEFT_1:
        {
            // ? 强制左转（不要用HAL_Delay）
            motor_setpwm_l(50);
            motor_setpwm_r(-20);

            delay_cnt++;

            if(delay_cnt > 90)  // 控制90°角度
            {
                car_state = TRACK_AFTER_TURN;
            }
            return; // ?? 必须return，否则PID会干扰
        }
        break;

        // ===============================
        case TRACK_AFTER_TURN:
        {
            if(cross_count == 2 && now_cross)
            {
                car_state = STOP_AT_SECOND;
            }
        }
        break;

        // ===============================
        case STOP_AT_SECOND:
        {
            motor_setpwm_l(0);
            motor_setpwm_r(0);
            return;
        }
        break;
    }

    // ===== 正常循迹PID =====
    int speed_output = speed_pid_task(target_speed, encoder_l, encoder_r);
    int turn_output  = turn_pid_task(track_error);

    int pwm_left  = speed_output - turn_output;
    int pwm_right = speed_output + turn_output;

    // 限幅
    if (pwm_left  >  80) pwm_left  =  80;
    if (pwm_left  < -80) pwm_left  = -80;
    if (pwm_right >  80) pwm_right =  80;
    if (pwm_right < -80) pwm_right = -80;

    motor_setpwm_l(pwm_right + 2);
    motor_setpwm_r(pwm_left);
}
void tasktwo_process(float target_speed)//2号病房
{
    gray_read(&gray_sensor);
    
    float track_error = calculate_tracking_error(&gray_sensor);
    
    static uint8_t last_cross = 0;
    uint8_t now_cross = gray_is_T_cross(&gray_sensor);

    // ===== 路口计数（边沿触发）=====
    if(now_cross && !last_cross)
    {
        cross_count++;
    }
    last_cross = now_cross;

    // ===== 新增变量 =====
    static uint16_t delay_cnt = 0;

    switch(car_state)
    {
        // ===============================
        case TRACK_LINE:
        {
            if(cross_count == 1 && now_cross)
            {
                // ? 不是直接转，而是先往前冲一点
                car_state = STRAIGHT_BEFORE_TURN;
                delay_cnt = 0;
            }
        }
        break;

        // ===============================
        case STRAIGHT_BEFORE_TURN:
        {
            delay_cnt++;

            // 继续循迹直行一小段（脱离路口中心）
            if(delay_cnt > 0)   // 可调 40~80
            {
                car_state = TURN_LEFT_1;
                delay_cnt = 0;
            }
        }
        break;
        case TURN_LEFT_1:
        {
            //  强制右转
            motor_setpwm_l(-30);
            motor_setpwm_r(50);

            delay_cnt++;

            if(delay_cnt > 90)  // 控制90°角度
            {
                car_state = TRACK_AFTER_TURN;
            }
            return; // 必须return，否则PID会干扰
        }
        break;
        case TRACK_AFTER_TURN:
        {
            if(cross_count == 2 && now_cross)
            {
                car_state = STOP_AT_SECOND;
            }
        }
        break;
        case STOP_AT_SECOND:
        {
            motor_setpwm_l(0);
            motor_setpwm_r(0);
            return;
        }
        break;
    }

    // ===== 正常循迹PID =====
    int speed_output = speed_pid_task(target_speed, encoder_l, encoder_r);
    int turn_output  = turn_pid_task(track_error);

    int pwm_left  = speed_output - turn_output;
    int pwm_right = speed_output + turn_output;

    // 限幅
    if (pwm_left  >  80) pwm_left  =  80;
    if (pwm_left  < -80) pwm_left  = -80;
    if (pwm_right >  80) pwm_right =  80;
    if (pwm_right < -80) pwm_right = -80;

    motor_setpwm_l(pwm_right + 2);
    motor_setpwm_r(pwm_left);
}
void taskthree_process(float target_speed,float target_l,float target_r)
{
    gray_read(&gray_sensor);
    
    float track_error = calculate_tracking_error(&gray_sensor);
    
    static uint8_t last_cross = 0;
    uint8_t now_cross = gray_is_T_cross(&gray_sensor);

    // ===== 路口计数 =====
    if(now_cross && !last_cross)
    {
        cross_count++;
    }
    last_cross = now_cross;

    // ===== 路口强制直行 =====
    static uint8_t cross_run_flag = 0;
    static uint16_t cross_run_cnt = 0;

    if(now_cross && cross_count == 1)
	{
		cross_run_flag = 1;
		cross_run_cnt = 0;
	}

    if(cross_run_flag)
    {
        cross_run_cnt++;

        // ?? 强制直行
        motor_setpwm_l(30);
        motor_setpwm_r(30);

        if(cross_run_cnt > 40)   // 30~60可调
        {
            cross_run_flag = 0;
        }

        return; // ?? 不走PID
    }

    // ===== 状态机 =====
    static uint16_t delay_cnt = 0;

    switch(car_state)
    {
        // ===============================
        case TRACK_LINE:
        {
            // ?? 第1路口：什么都不做（直行）

            // ?? 第2路口：准备左转
            if(cross_count == 2 && now_cross)
            {
                car_state = STRAIGHT_BEFORE_TURN;
                delay_cnt = 0;
            }

            // ?? 第3路口：停车
            if(cross_count == 3 && now_cross)
            {
                car_state = STOP_AT_SECOND;
            }
        }
        break;

        // ===============================
        case STRAIGHT_BEFORE_TURN:
        {
            delay_cnt++;

            if(delay_cnt > 0)   // 前冲
            {
                car_state = TURN_LEFT_1;
                delay_cnt = 0;
            }
        }
        break;

        // ===============================
        case TURN_LEFT_1:
        {
            motor_setpwm_l(target_l);
            motor_setpwm_r(target_r);

            delay_cnt++;

            if(delay_cnt > 90)
            {
                car_state = TRACK_AFTER_TURN;
                delay_cnt = 0;
            }
            return;
        }

        // ===============================
        case TRACK_AFTER_TURN:
        {
            // 第3路口停车
            if(cross_count == 3 && now_cross)
            {
                car_state = STOP_AT_SECOND;
            }
        }
        break;

        // ===============================
        case STOP_AT_SECOND:
        {
            motor_setpwm_l(0);
            motor_setpwm_r(0);
            return;
        }
    }

    // ===== 正常循迹 =====
    int speed_output = speed_pid_task(target_speed, encoder_l, encoder_r);
    int turn_output  = turn_pid_task(track_error);

    int pwm_left  = speed_output - turn_output;
    int pwm_right = speed_output + turn_output;

    if (pwm_left  >  80) pwm_left  =  80;
    if (pwm_left  < -80) pwm_left  = -80;
    if (pwm_right >  80) pwm_right =  80;
    if (pwm_right < -80) pwm_right = -80;

    motor_setpwm_l(pwm_right + 2);
    motor_setpwm_r(pwm_left);
}

