#include "FreeRTOS_Text.h"
#include "FreeRTOS.h"
#include "task.h"

#include "Chassis.h"
#include "BMI088_Driver.h"
#include "BMI088.h"

extern BMI088_DATA imu_data;
extern AttitudeEstimator estimator;
extern Kalman_Data kalman_data;

TaskHandle_t Chassis_Task_Handle;
TaskHandle_t BMI088_Task_Handle;


/**
* @brief    Chassis_Task(void *pvParameters)
* @details  底盘控制任务
**/
void Chassis_Task(void *pvParameters)
{
    while(1)
    {
        Chassis_movement();
        vTaskDelay(Chassis_Task_Delay);
    }
}


/**
* @brief    BMI088_Task(void *pvParameters)
* @details  BMI088姿态解算，严格延时BMI088_Task_Delay
**/
void BMI088_Task(void *pvParameters)
{
    TickType_t last_time = xTaskGetTickCount();
    while(1)
    {
        BMI088_read_(&imu_data);
		Kalman_Attitude_Estimator_Update(&imu_data);
        TempCtrl_BMI088();
        vTaskDelayUntil(&last_time, BMI088_Task_Delay);
    }
}


/**
* @brief    FreeRTOS_Text()
* @details  任务集合
**/
void FreeRTOS_Text()
{
    Chassis_Init();
    BMI088_init();
    xTaskCreate(BMI088_Task, "BMI088_Task", 128, NULL, 5, &BMI088_Task_Handle);
    xTaskCreate(Chassis_Task, "Chassis_Task", 128, NULL, 5, &Chassis_Task_Handle);
}


