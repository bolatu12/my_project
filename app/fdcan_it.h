#ifndef FDCAN_H
#define FDCAN_H


#include "main.h"
#include "DJI_Transform_master.h"

//舵轮电机回零
typedef enum
{
	eHOMING_IDLE,
	eHOMING_READY,
    eHOMING_MOVING,
    eHOMING_DONE	
}Steer_Homing_State;

typedef struct
{
	DJI_HandleTypeDef *steer;
	GPIO_TypeDef *port;           // 光电门所在的 GPIO 组 (如 GPIOC)
    uint16_t pin;                 // 光电门对应的引脚号 (如 GPIO_PIN_1)
    uint8_t is_homed;             // 该单轮是否已完成回零	
}Steer_Homing_TypeDef;


//void my_HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

void Chassis_C620_Init(void);
void App_SBUS_Chassis(void *argument);

#endif
