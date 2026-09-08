#include "my_main.h"


#include "chassis_control.h"
#include "Zifolib.h"
#include "WS2811.h"

//全局变量
Chassis_Data chassis_data = {0};
Gyro MINS500_data = {0};
Radar_HandleTypeDef radar = {0};
DT35_U DT35 = {0};
Path_HandleTypeDef path = {0};
Point_HandleTypeDef current_pose = {0};
QT_HandleTypeDef qt = {0};
uint8_t air_pum_state[4] = {0};

volatile uint8_t auto_flag = 0; // 自动标志位	
volatile uint8_t auto_finish_flag = 0;				//自动是否完成标志位
volatile uint8_t gyro_reset_flag = 0;

uint8_t direct_auto_once = 0;				//让直接跑二区的按键只用一次


//底盘初始化
void Chassis_Components_Init(void)
{
	//底盘结构体初始化
	Chassis_Init(&chassis_data, 0.075f, 0.15f);

	Path_Init(&path);
}


void App_Chassis(void *argument)
{
	static Chassis_Mode_e s_chassis_mode = low_speed;
	static uint8_t pid_flag = 1;
	static Handle_Data s_handle_data = {0};
	static Chassis_Motor s_chassis_motor = {0};
	static	Path_State s_path_state = 0;
	static uint8_t first_switch_flag = 0;
	static uint8_t switch_state = 0;

    for(;;)
    {
        //获取队列
		xQueueReceive(Handle_Data_ToChassisHandle, &s_handle_data, 1); // 手柄数据

        //按键
		key_left_value[1] = Key_Detect(&key_left[1], 1);
		key_left_value[2] = Key_Detect(&key_left[2], 2);
		key_left_value[3] = Key_Detect(&key_left[3], 1);
		key_left_value[4] = Key_Detect(&key_left[4], 1);
		key_left_value[7] = Key_Detect(&key_left[7], 1);

		key_left_value[6] = Key_Detect(&key_left[6], 1);



        // 获取摇杆值   -100 ~ 100
		if(auto_flag == 0)
		{
			chassis_data.input_world_velocity.Vx = s_handle_data.rock_data.rock_left_x;
			chassis_data.input_world_velocity.Vy = s_handle_data.rock_data.rock_left_y;
			
			if ((ABS(chassis_data.input_world_velocity.Vx) - ABS(chassis_data.input_world_velocity.Vy)) > 10)
			{
				chassis_data.input_world_velocity.Vy = 0;
			}
			else
			{
				chassis_data.input_world_velocity.Vx = 0;
			}


			// 速度规划 逐步加速 逐步减速 限制加速度  ！！！！ 注意：w向未规划 ！！！！
			// 这里规划之后Chassis_Control函数使用规划值
			Velocity_Planning(&chassis_data);
		}

		// 获取当前yaw角
		if(MINS500_data.first_flag == 1 && MINS500_data.real_yaw_1 != 0)
		{
			chassis_data.current_yaw = MINS500_data.real_yaw_1;		
		}

		//获取当前坐标
		current_pose.x = radar.x;
		current_pose.y = radar.y;
		current_pose.yaw = MINS500_data.real_yaw_1;				//这里传入陀螺仪角度制


		//限位开关重置陀螺仪
		if(HAL_GPIO_ReadPin(IO_16_GPIO_Port, IO_16_Pin) == GPIO_PIN_SET)
		{
			if(gyro_reset_flag == 0)
			{
				MINS500_data.first_flag = 0;
				gyro_reset_flag = 1;
			}
		}


		if(key_left_value[2] == 1)
		{
			s_chassis_mode = mid_speed;
		}else if(key_left_value[2] == 2)
		{
			s_chassis_mode = low_speed;
		}

		if(key_left_value[3] == 1)
		{
			chassis_data.target_yaw += 90.0f;
		}

		if(key_left_value[4] == 1)
		{
			chassis_data.target_yaw -= 90.0f;
		}


		if(key_left_value[6] == 1)
		{
			auto_flag = 0;
			path.path_state = ePREPARE;
			crawling_completed = 0;
			Change_speed(3.0f, 3.0f);
			Path_Update_Status(&path, ePREPARE);
		}


		//判断雷达是否叽叽置标志位
		if(auto_flag == 1)
		{
			if(radar.error_flag == 1 || radar.data_flag == 1)
			{
				chassis_data.ladar_err_flag = 1;
			}else
			{
				chassis_data.ladar_err_flag = 0;
			}
		}

		//强制手动 这里后面应该看一下那个路径状态机有没有啥残留物清空一下（可能导致车体疯转）
		if(key_middle[4].key == 0)
		{
			auto_flag = 0;
		}

		//根据小电脑控制走左侧还是右侧
		if(radar.route_flag == 1)
		{
			path.direction = 1;							//右侧
			path.route = right_state;					//右侧路径
			path.point_num = RIGHT_POINT_NUM;
		}else 
		{
			path.direction = 0;							//左侧
			path.route = left_state;					//左侧路径
			path.point_num = LEFT_POINT_NUM;
		}		


		if(key_middle[0].key == 1)	
		{
			chassis_data.stop_flag = 0;
		}else
		{
			chassis_data.stop_flag = 1;
		}


		//判断放哪一列
		if(key_middle[1].key == 1)
		{
			path.line = 1;
		}else
		{
			path.line = 3;
		}

		//是否斜着与竖着相切换
		if(xQueueReceive(Handle_Data_ToChassisHandle, &s_handle_data, 1) == pdTRUE)
		{
			//在收到手柄数据之后再存储这个开关状态
			if(first_switch_flag == 0)
			{
				switch_state = key_middle[2].key;
				first_switch_flag = 1;
			}
		}

		if(key_middle[2].key == 1)
		{
			path.is_switch = 1;
		}else
		{
			path.is_switch = 0;
		}

		if(switch_state != key_middle[2].key)
		{
			auto_flag = 1;
			switch_state = key_middle[2].key;
		}



		//自动模式切换
		if(key_left_value[7] == 1)
		{
			auto_flag = 1;
		}


		//留一个按键用来直接更新目标点
		if(key_left_value[1] == 1)
		{
			if(direct_auto_once == 0)
			{
				if(auto_finish_flag == 0)
				{
					auto_flag = 1;
					s_path_state = qt.R1_route_state[qt.temp];
					qt.temp ++;
					if(qt.temp > 3) qt.temp = 3; 
					Path_Update_Status(&path, s_path_state);
				}else
				{
					auto_flag = 0;
				}
				direct_auto_once = 1;
			}
		}

		//接收到路径那边的信号量之后直接将底盘速度改为低速
		if(xSemaphoreTake(Chassis_Low_SpeedHandle, 1) == pdTRUE)
		{
			s_chassis_mode = low_speed;
		}


		//底盘是否自动
		if (auto_flag == 0)
		{
			Chassis_Control(&chassis_data, s_chassis_mode, pid_flag);
		}
		else if (auto_flag == 1)
		{
			//路径状态机
			Path_Fsm(&path, &current_pose);

			//将计算出的速度赋值给底盘
			chassis_data.target_world_velocity.Vx = path.Vx;
			chassis_data.target_world_velocity.Vy = path.Vy;
			chassis_data.target_world_velocity.Vw = path.Vw;

			Chassis_Control_Auto(&chassis_data); // 自动底盘未完善
		}
		else if(auto_flag == 2)
		{
			chassis_data.target_world_velocity.Vx = pid_calc(&DT_x, 2.2f, DT35_XF);
			chassis_data.target_world_velocity.Vw = pid_calc(&pid_w, 3.14, (chassis_data.current_yaw / 180) * 3.14f);
			
			if(fabsf(DT35_XF - 2.2f) < 0.004f) 
			{
				auto_flag = 0;
			}
			Chassis_Control_Auto(&chassis_data); // 自动底盘未完善
		}
		

        for (uint8_t i = 0; i < 4; i++)
        {
            s_chassis_motor.wheel_speed[i] = chassis_data.wheel_speed[i]; // 轮向电机
            s_chassis_motor.steer_angle[i] = chassis_data.steer_angle[i]; // 舵向电机
        }

		xQueueSend(Chassis_To_CanHandle, &s_chassis_motor, 1); // 将电机数据发送给can发送任务
		osDelay(1);
	}

}

void App_LED(void *argument)
{
    static TickType_t sTick = 0;

    for (;;)
    {
        TickType_t now = xTaskGetTickCount();

        if (now - sTick >= 1000)
        {
            sTick = now;
        }

        if (now < sTick + 500)
        {
            if (!gyro_reset_flag)
            {	
                LEDStrip.pf->setColor(&LEDStrip, 7, 13, LED_COLOR_YELLOW);
            }else
			{
				LEDStrip.pf->setColor(&LEDStrip, 7, 13, LED_COLOR_GREEN);
			}
        }
        else if (now < sTick + 1000 && now > sTick + 500)
        {
            if (radar.error_flag == 1 || radar.data_flag == 1)
            {
                LEDStrip.pf->setColor(&LEDStrip, 7, 13, LED_COLOR_RED);
            }else
			{
				LEDStrip.pf->setColor(&LEDStrip, 7, 13, LED_COLOR_GREEN);
			}

        }

		switch(led_state)
		{
			case 1:
			{
				LEDStrip.pf->setColor(&LEDStrip, 1, 20, LED_COLOR_GREEN);
				break;
			}
			case 2:
			{
				LEDStrip.pf->setColor(&LEDStrip, 1, 20, LED_COLOR_BLUE);
				break;
			}
			case 3:
			{
				LEDStrip.pf->setColor(&LEDStrip, 1, 20, LED_COLOR_BLACK);
				break;
			}
			case 4:
			{
				LEDStrip.pf->setColor(&LEDStrip, 1, 20, LED_COLOR_PURPLE);
				break;
			}
			case 5:
			{
				LEDStrip.pf->setColor(&LEDStrip, 1, 20, LED_COLOR_YELLOW);
				break;
			}
			case 6:
			{
				LEDStrip.pf->setColor(&LEDStrip, 1, 20, LED_COLOR_RED);
				break;
			}
			default:
			{
				break;
			}
		}		

        osDelay(1);
    }
}

void App_print(void *argument)
{
	for(;;)
	{
		// fprint(&huart8, DT35_XF, 2.2f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,0.0f,0.0f);
		// fprint(&huart8, chassis_data.target_yaw, chassis_data.current_yaw, pid_w.pout, pid_w.dout, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,0.0f,0.0f);
//		fprint(&huart8, path.Vx, path.Vy, DT35_YR, DT_x.iout, DT_y.iout, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,0.0f,0.0f);

		
		osDelay(20);
	}
}