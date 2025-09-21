#include "Algorithm.h"
#include "PID.h"

extern uint16_t motor_maxcurrent;

PIDController pid_motor1, pid_motor2, pid_motor3, pid_motor4;


void Algorithm_Init()
{
	pid_motor1 = Creat_PID();
	pid_motor1.max_data = motor_maxcurrent;
	
	pid_motor2 = Creat_PID();
	pid_motor2.max_data = motor_maxcurrent;
	
	pid_motor3 = Creat_PID();
	pid_motor3.max_data = motor_maxcurrent;
	
	pid_motor4 = Creat_PID();
	pid_motor4.max_data = motor_maxcurrent;
	
	Modify_PID_K(&pid_motor1, 1.85, 0.057, 0.0015);
	Modify_PID_K(&pid_motor2, 1.85, 0.057, 0.0015);
	Modify_PID_K(&pid_motor3, 1.85, 0.057, 0.0015);
	Modify_PID_K(&pid_motor4, 1.85, 0.057, 0.0015);
}
