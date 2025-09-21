#ifndef	__IMU_DATA_H__
#define	__IMU_DATA_H__

#include "sys.h"
#include "User_Define.h"

// Beta�����������˲������������ʣ��Ƽ�0.1-0.5��
#define BETA 			0.2f
#define RAD_TO_DEG 		57.29577951308232f


typedef struct 
{
    float q0;
    float q1;
    float q2;
    float q3;
}Quaternion;


typedef struct 
{
    float roll;   // ��ת�� (��) - �ȶ�
    float pitch;  // ������ (��) - �ȶ�
    float yaw;    // ƫ���� (��) - ��Ư��
}EulerAngles;


void IMU_Update_Data(IMU_DATA* imu, float delta_time);
EulerAngles Quaternion_to_Euler(const Quaternion* q);

#endif
