#ifndef __CHASSISTASK_H
#define __CHASSISTASK_H

#include "CanReceiveDecom.h"
#include "pid.h"
#include "gpio.h"

//60W����0.64 
#define FallowAngle -158
#define RotingBaseSpeed 0.64
typedef enum
{
	NOFORCE,
	STOP,
	FALLOW,
	ROTING
}ChassisMode_e;

//Ŀǰ��ʱ����ֻ�Ǽ򵥵ȼ��仯�ƶ�
//typedef enum
//{
//	LOW,
//	NORMAL,
//	HEIGHT,
//}Roting_Speed_e;

typedef struct
{
	float frame_period;
	float input;
	float out;
	float error;
	float buffer;
}BufferFunction_t;

typedef struct
{
	const motor_measure_t *Motor[4];
	const motor_measure_t *Yaw;
	ChassisMode_e Mode;
	
	float vx;
	float vy;
	float wz;
	pid_type_def XYPid[4];
	pid_type_def WZPid;
	float Current[4];
	float WheelSpeed[4];
	int BukPowerEn;
}ChassisCtrl_t;

extern ChassisCtrl_t ChassisCtrl;
void ChassisControlLoop();
#endif