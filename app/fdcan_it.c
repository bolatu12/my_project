#include "my_main.h"
#include "uper_control.h"
#include "chassis_control.h"
#include "chassis.h"
#include "fdcan_it.h"
#include "STP_23L.h"
#include "usart_it.h"
#include "iwdg.h"


static void MIT_Planning(MechanicalArm_HandleTypeDef *arm);
static void Steer_Homing_FSM(void);

//定时获取MG5010E位置
uint32_t MG5010E_tick = 0;
uint32_t HT3505_tick = 0;

uint8_t ready_index = 0;
uint32_t delay_tick = 0;
uint32_t wait_tick = 0;

// 单 3508 测试句柄（方案A：C620 电调直连，挂 hfdcan2）
DJI_C620_HandleTypeDef test_3508;

/**
 * @brief:FDCAN中断回调函数
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	FDCAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];

	HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);

	if(hfdcan == &hfdcan1)
	{
//		VESC_Data_Process(&wheelLF, &rx_header, rx_data);
//		VESC_Data_Process(&wheelRF, &rx_header, rx_data);
//		VESC_Data_Process(&wheelRB, &rx_header, rx_data);
//		VESC_Data_Process(&wheelLB, &rx_header, rx_data);

//		DJI_Get_Meature(&LF_steer, &rx_header, rx_data);
//		DJI_Get_Meature(&RF_steer, &rx_header, rx_data);
 //		DJI_Get_Meature(&RB_steer, &rx_header, rx_data);
//		DJI_Get_Meature(&LB_steer, &rx_header, rx_data);
		
		DJI_C620_GetFeedback(&wheelLF, &rx_header, rx_data);
		DJI_C620_GetFeedback(&wheelRF, &rx_header, rx_data);
		DJI_C620_GetFeedback(&wheelRB, &rx_header, rx_data);
		DJI_C620_GetFeedback(&wheelLB, &rx_header, rx_data);

	}else if(hfdcan == &hfdcan2)
	{
		DJI_C620_GetFeedback(&test_3508, &rx_header, rx_data);   // 单 3508 反馈解析

//		//左臂
//		MG5010E_GetPosition(&Left_arm.Down_motor, &rx_header, rx_data);
//		RS00_Get_MotorData(&Left_arm.High_motor, &rx_header, rx_data);
//		RS05_Get_MotorData(&Left_arm.Sucker_motor, &rx_header, rx_data);

//		//右臂
//		MG5010E_GetPosition(&Right_arm.Down_motor, &rx_header, rx_data);
//		RS00_Get_MotorData(&Right_arm.High_motor, &rx_header, rx_data);
//		RS05_Get_Mo torData(&Right_arm.Sucker_motor, &rx_header, rx_data);
	}else if(hfdcan == &hfdcan3)
	{
//		RS00_Get_MotorData(&Spin_clow, &rx_header, rx_data);
//		RS05_Get_MotorData(&Translation_motor, &rx_header, rx_data);
//		DJI_Get_Meature(&Weapon_flex, &rx_header, rx_data);
//		DJI_Get_Meature(&Storage_motor, &rx_header, rx_data);
//		DJI_Get_Meature(&Left_flex, &rx_header, rx_data);
//		DJI_Get_Meature(&Right_flex, &rx_header, rx_data);
    }
}


//用于定时跑电机规划(and 定时给AK发指令 )
//void my_HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if(htim == &htim4)
//	{
//		//当检测到FDCAN总线因发送错误过多自动关闭时重启
//		if (hfdcan1.Instance->PSR & (FDCAN_PSR_BO | FDCAN_PSR_EP))
//		{
//			HAL_FDCAN_Stop(&hfdcan1);
//			FDCAN_Init(&hfdcan1);
//		}
//		if (hfdcan2.Instance->PSR & (FDCAN_PSR_BO | FDCAN_PSR_EP))
//		{
//			HAL_FDCAN_Stop(&hfdcan2);
//			FDCAN_Init(&hfdcan2);
//		}
//		if (hfdcan3.Instance->PSR & (FDCAN_PSR_BO | FDCAN_PSR_EP))
//		{
//			HAL_FDCAN_Stop(&hfdcan3);
//			FDCAN_Init(&hfdcan3);
//		}

//		//补充电机规划
//		/**left_arm**/
//		MIT_Planning(&Left_arm);
//		/**right_arm**/
//		MIT_Planning(&Right_arm);


//		//爪子旋转
//		if(Spin_T.start_flag == 1)
//		{
//			uper_motor_pos.spin_clow_pos = Pos_Planning(&Spin_T);
//			RS00_MITMode(&Spin_clow, uper_motor_pos.spin_clow_pos, 0.0f, 130.0f, 5.0f, 0.0f);

//			if(Spin_T.end_flag == 1)
//			{
//				Spin_T.start_flag = 0;
//			}
//		}

//		//定时获取左右臂大臂电机位置
//		if(Left_arm.Down_motor.hfdcan != NULL && Right_arm.Down_motor.hfdcan != NULL)
//		{
//			MG5010E_tick ++;
//			if(MG5010E_tick >= 100)
//			{
//				MG5010E_ReadPosition(&Left_arm.Down_motor);
//				MG5010E_ReadPosition(&Right_arm.Down_motor);
//				MG5010E_tick = 0;
//			}
//		}

//		//读取吸盘状态
//		if(stp23l_distance_left < L_DISTANCW_MIN)
//		{
//			Left_arm.sucker_state = 1;
//		}else
//		{
//			Left_arm.sucker_state = 0;
//		}

//		if(stp23l_distance_right < R_DISTANCE_MIN)
//		{
//			Right_arm.sucker_state = 1;
//		}else
//		{
//			Right_arm.sucker_state = 0;
//		}


//		//舵轮回零
//		if(steer_homing[0].steer->hcan != NULL && steer_homing[1].steer->hcan != NULL && steer_homing[2].steer->hcan != NULL && steer_homing[3].steer->hcan != NULL)
//		{
//			Steer_Homing_FSM();
//		}
//	}
//}


////电机MIT规划
//void MIT_Planning(MechanicalArm_HandleTypeDef *arm)
//{
//	//小臂
//	if(arm->High_T.start_flag == 1)
//	{
//		arm->high_pos = Pos_Planning(&arm->High_T);
//		RS00_MITMode(&arm->High_motor, arm->high_pos, 0.0f, 130.0f, 5.0f, 0.0f);

//		if(arm->High_T.end_flag == 1)
//		{
//			arm->High_T.start_flag = 0;
//		}
//	}


//	//吸盘
//	if(arm->Sucker_T.start_flag == 1)
//	{
//		arm->sucker_pos = Pos_Planning(&arm->Sucker_T);
//		RS05_MITMode(&arm->Sucker_motor, arm->sucker_pos, 0.0f, 100.0f, 5.0f, 0.0f);

//		if(arm->Sucker_T.end_flag == 1)
//		{
//			arm->Sucker_T.start_flag = 0;
//		}
//	}
//}


////舵轮回零
//void Steer_Homing_FSM(void)
//{
//	switch(steer_homing_state)
//	{
//		case eHOMING_IDLE:
//		{
//			//等待回零指令
//			ready_index = 0;
//			delay_tick = 0;
//			wait_tick = 0;
//			break;
//		}
//		case eHOMING_READY:
//		{
//			delay_tick ++;
//			if(delay_tick > 100)
//			{
//				if(ready_index < 4)
//				{
//					DJI_SpeedMode(steer_homing[ready_index].steer, 20.0f, 0);
//					ready_index++;
//					delay_tick = 0;
//				}
//				else
//				{
//					// 全部发送完毕，清空索引并跳转
//					ready_index = 0;
//					delay_tick = 0;
//					steer_homing_state = eHOMING_MOVING;
//				}
//			}
//			break;
//		}
//		case eHOMING_MOVING:
//		{		
//			for(uint8_t i = 0; i < 4; i++)
//			{
//				if(steer_homing[i].is_homed == 0)
//				{
//					if(HAL_GPIO_ReadPin(steer_homing[i].port, steer_homing[i].pin) == GPIO_PIN_SET)
//					{
//						DJI_SetOrigin(steer_homing[i].steer);
//						if(fabsf(steer_homing[i].steer->position) < 0.1f) // 位置接近零点
//						{
//							steer_homing[i].is_homed = 1;
//						}
//					}
//				}
//			}
//			if(steer_homing[0].is_homed && steer_homing[1].is_homed && steer_homing[2].is_homed && steer_homing[3].is_homed)
//			{
//				wait_tick ++;
//				if(wait_tick > 300)
//				{
//					steer_homing_state = eHOMING_DONE;
//					wait_tick = 0;
//				}
//			}
//			break;
//		}
//		case eHOMING_DONE:
//		{
//			break;
//		}
//	}

//}

/**
 * @brief FDCAN发送任务：底盘全向轮速度环控制（标准C620协议，直连电调）
 * @note  需在 freertos.c 中取消 osThreadNew(App_FDCAN, ...) 的注释才会运行
 */
void App_FDCAN(void *argument)
{
	static Chassis_Motor s_chassis_motor = {0};
	static uint32_t watchdog_tick = 0;
	DJI_C620_HandleTypeDef *wheel_motors[4] = {&wheelLF, &wheelRF, &wheelRB, &wheelLB};

	for(;;)
	{
		if (xQueueReceive(Chassis_To_CanHandle, &s_chassis_motor, 1) != pdPASS)
		{
			osDelay(1);
			continue;
		}

		if (wheel_motors[0]->hcan == NULL || wheel_motors[1]->hcan == NULL ||
			wheel_motors[2]->hcan == NULL || wheel_motors[3]->hcan == NULL)
		{
			osDelay(1);
			continue;
		}

		// 更新四轮目标转速（rpm）
		wheelLF.target_speed = s_chassis_motor.wheel_speed[0];
		wheelRF.target_speed = s_chassis_motor.wheel_speed[1];
		wheelRB.target_speed = s_chassis_motor.wheel_speed[2];
		wheelLB.target_speed = s_chassis_motor.wheel_speed[3];

		// 速度环：目标转速 -> 目标电流
		for(uint8_t i = 0; i < 4; i++)
		{
			DJI_C620_SpeedControl(wheel_motors[i]);
		}

		// 打包发送 0x200 控制帧
		DJI_C620_SendCurrent(wheel_motors);
		if ((HAL_GetTick() - watchdog_tick) >= 100u)
		{
			HAL_IWDG_Refresh(&hiwdg1);
			watchdog_tick = HAL_GetTick();
		}

		osDelay(1);
	}
}


/* ================= 单 3508 测试（方案A：C620 直连） ================= */

// 单电机发送（现有 DJI_C620_SendCurrent 是一次打包 4 个电机，单测别直接用）
void C620_Single_Send(DJI_C620_HandleTypeDef *h)
{
    uint8_t data[8] = {0};
    uint8_t idx = (h->id - 1) * 2;                      // id=1 => 字节 0-1
    data[idx]     = (uint8_t)((h->target_current >> 8) & 0xFF);
    data[idx + 1] = (uint8_t)(h->target_current & 0xFF);

    FDCAN_TxHeaderTypeDef TxHeader = {0};
    TxHeader.Identifier          = 0x200;               // 控制帧固定 0x200
    TxHeader.IdType              = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType         = FDCAN_DATA_FRAME;
    TxHeader.DataLength          = 8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch       = FDCAN_BRS_OFF;
    TxHeader.FDFormat            = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker       = 0;

    // 非阻塞：FIFO 满就跳过，避免废帧塞满 FIFO 后任务卡死、TEC 冻住看不清真实状态
    if (HAL_FDCAN_GetTxFifoFreeLevel(h->hcan) > 0)
    {
        HAL_FDCAN_AddMessageToTxFifoQ(h->hcan, &TxHeader, data);
    }
}

// 单 3508 初始化：启动 FDCAN2 + 初始化电机
void Test_3508_Init(void)
{
    FDCAN_Init(&hfdcan2);                     // Start FDCAN2 + 全局滤波 + 使能 RX 中断
    DJI_C620_Init(&test_3508, &hfdcan2, 1);   // id=1 => 反馈帧 0x201
}

// 单 3508 测试任务
void Test_3508_Task(void *argument)
{
    for(;;)
    {
        test_3508.target_speed = 200.0f;      // 目标转速 rpm，先小值试转
        DJI_C620_SpeedControl(&test_3508);    // 速度环 -> 目标电流
        C620_Single_Send(&test_3508);
        osDelay(1);
    }
}

void Chassis_C620_Init(void)
{
	FDCAN_Init(&hfdcan1);
	DJI_C620_Init(&wheelLF, &hfdcan1, 1);
	DJI_C620_Init(&wheelRF, &hfdcan1, 2);
	DJI_C620_Init(&wheelRB, &hfdcan1, 3);
	DJI_C620_Init(&wheelLB, &hfdcan1, 4);
}

#define SBUS_AXIS_MIN       200
#define SBUS_AXIS_MAX       1800
#define SBUS_AXIS_CENTER    1000
#define SBUS_AXIS_DEADBAND  200
#define CHASSIS_INPUT_MAX   150.0f
#define CHASSIS_SPEED_SCALE 5.65f

static float SBUS_Axis(uint16_t value)
{
	int32_t offset = (int32_t)value - SBUS_AXIS_CENTER;
	if (offset > -SBUS_AXIS_DEADBAND && offset < SBUS_AXIS_DEADBAND)
		return 0.0f;
	if (value < SBUS_AXIS_MIN)
		value = SBUS_AXIS_MIN;
	if (value > SBUS_AXIS_MAX)
		value = SBUS_AXIS_MAX;
	return ((float)((int32_t)value - SBUS_AXIS_CENTER) / 800.0f) * CHASSIS_INPUT_MAX;
}

void App_SBUS_Chassis(void *argument)
{
	Chassis_Data sbus_chassis = {stop};
	Chassis_Motor motor = {0};

	Chassis_Init(&sbus_chassis, 0.075f, 0.15f);
	Chassis_Control(&sbus_chassis, mid_speed, 0);

	for (;;)
	{
		if (sbus_failsafe_status != 0u || (HAL_GetTick() - sbus_last_tick) > 100u)
		{
			sbus_chassis.input_world_velocity.Vx = 0.0f;
			sbus_chassis.input_world_velocity.Vy = 0.0f;
			sbus_chassis.input_world_velocity.Vw = 0.0f;
			sbus_chassis.planning_velocity.Vx = 0.0f;
			sbus_chassis.planning_velocity.Vy = 0.0f;
			sbus_chassis.stop_flag = 1;
		}
		else
		{
			sbus_chassis.input_world_velocity.Vx = SBUS_Axis(CH[0]);
			sbus_chassis.input_world_velocity.Vy = SBUS_Axis(CH[1]);
			sbus_chassis.input_world_velocity.Vw = SBUS_Axis(CH[3]);
			sbus_chassis.stop_flag = 0;
		}

		Velocity_Planning(&sbus_chassis);
		Chassis_Control(&sbus_chassis, mid_speed, 0);
		for (uint8_t i = 0; i < 4; i++)
		{
			motor.wheel_speed[i] = sbus_chassis.wheel_speed[i] * CHASSIS_SPEED_SCALE;
			motor.steer_angle[i] = sbus_chassis.steer_angle[i];
		}

		(void)osMessageQueuePut(Chassis_To_CanHandle, &motor, 0, 0);
		osDelay(2);
	}
}

//void App_FDCAN(void *argument)
//{
//	static Chassis_Motor s_chassis_motor = {0};
//	for(;;)
//	{
//		xQueueReceive(Chassis_To_CanHandle,&s_chassis_motor,1);


//		//舵向电机 在电机回零完成之后在进行发指令
//		if(steer_homing_state == eHOMING_DONE)
//		{
//			DJI_PositionMode(&LF_steer, s_chassis_motor.steer_angle[0], 50.0f, 0.0f);
//			osDelay(1);
//			DJI_PositionMode(&RF_steer, s_chassis_motor.steer_angle[1], 50.0f, 0.0f);
//			osDelay(1);
//			DJI_PositionMode(&RB_steer, s_chassis_motor.steer_angle[2], 50.0f, 0.0f);
//			osDelay(1);
//			DJI_PositionMode(&LB_steer, s_chassis_motor.steer_angle[3], 50.0f, 0.0f);

//			Vesc_Speed_mode(&wheelLF, s_chassis_motor.wheel_speed[0]);              //轮向电机
//			osDelay(1);
//			Vesc_Speed_mode(&wheelRF, s_chassis_motor.wheel_speed[1]);              //轮向电机
//			osDelay(1);
//			Vesc_Speed_mode(&wheelRB, s_chassis_motor.wheel_speed[2]);              //轮向电机
//			osDelay(1);
//			Vesc_Speed_mode(&wheelLB, s_chassis_motor.wheel_speed[3]);              //轮向电机
//			osDelay(1);
//		}



//		/*fdcan重启*/
//        while ( (FDCAN1->PSR & FDCAN_PSR_BO) ) 
//        {
//            HAL_FDCAN_Stop(&hfdcan1);
//            HAL_FDCAN_DeInit(&hfdcan1);
//            HAL_FDCAN_Stop(&hfdcan2);
//            HAL_FDCAN_DeInit(&hfdcan2);
//            HAL_FDCAN_Stop(&hfdcan3);
//            HAL_FDCAN_DeInit(&hfdcan3);
//            __HAL_RCC_FDCAN_CLK_DISABLE();
//            __HAL_RCC_FDCAN_FORCE_RESET();
//            __HAL_RCC_FDCAN_RELEASE_RESET();
//            __HAL_RCC_FDCAN_CLK_ENABLE();
//            
//            osDelay(10);        
//            HAL_FDCAN_Init(&hfdcan1);
//            HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
//            HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
//            HAL_FDCAN_Start(&hfdcan1);
//            
//            HAL_FDCAN_Init(&hfdcan2);
//            HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
//            HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
//            HAL_FDCAN_Start(&hfdcan2);
//            
//            HAL_FDCAN_Init(&hfdcan3);
//            HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
//            HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
//            HAL_FDCAN_Start(&hfdcan3);
//            osDelay(10);	
//        }		
//		
//		osDelay(1);
//	}
//}


