#include "ChassisTask.h"
#include "cmsis_os.h"
#include "CMSInterface.h"
#include "BspMotor.h"
#include "arm_math.h"
#include "ChassisPowerBehaviour.h"
#include "math.h"
#include "user_lib.h"

ChassisCtrl_t ChassisCtrl;
float XYPid[4][3]={{20000,0,0},
					{20000,0,0},
					{20000,0,0},
					{20000,0,0}};
float WZPid[3] = {0.015,0.00001,0};

BufferFunction_t BufferFunctionX;
BufferFunction_t BufferFunctionY;
BufferFunction_t BufferFunctionWZ;

void BufferFunctionInit(BufferFunction_t *BufferFunction,fp32 frame_period);
void BufferFunctionCalc(BufferFunction_t *BufferFunction,fp32 input);

void ChassisInit();
void ChassisSetmode();
void ChassisContolSet();
void ChassisControlLoop();

void ChassisTask(void const * argument)
{
	ChassisInit();
	
	while(1)
	{
		ChassisSetmode();
		ChassisContolSet();
		ChassisControlLoop();
		
		APP_BatteryCombineBuckBoost2();
		ChassisPowerControl();
		ChassisCMD(ChassisCtrl.Current[0], ChassisCtrl.Current[1], ChassisCtrl.Current[2], ChassisCtrl.Current[3]);

		osDelay(1);
	}
}

void ChassisInit()
{
	for(int i=0;i<4;i++) 
	{
		ChassisCtrl.Current[i] = 0;
		ChassisCtrl.WheelSpeed[i] = 0;
		ChassisCtrl.Motor[i] = GetChassisMeasure(i);
		PID_init(&ChassisCtrl.XYPid[i], PID_POSITION, XYPid[i], 16300, 5000);
	}

	ChassisCtrl.Yaw = GetYawMeasure();
	PID_init(&ChassisCtrl.WZPid,PID_ANGLE,WZPid,1,0);
	ChassisCtrl.Mode = NOFORCE;
	BufferFunctionInit(&BufferFunctionX,100);
	BufferFunctionInit(&BufferFunctionY,100);
	BufferFunctionInit(&BufferFunctionWZ,500);
	ChassisCtrl.BukPowerEn = 1;
}
void ChassisSetmode()
{
	switch(PTZ.ChassisStatueRequest)
	{
		case 0x01:
			ChassisCtrl.Mode = NOFORCE; break;
		case 0x12:
			ChassisCtrl.Mode = ROTING; break;
		case 0x0A:
			ChassisCtrl.Mode = FALLOW; break;
		case 0x06:
			ChassisCtrl.Mode = STOP; break;
		default:
			break;
	}
}
fp32 rote_powkp = 3;
fp32 roting_speed = RotingBaseSpeed;
fp32 Erro_angle = 0;
extern fp32 robot_level;
extern fp32 real_power_limit;
void ChassisContolSet()
{
	float del = 0;
	
	if(power_limit == 60 && ChassisCtrl.Mode == ROTING)
	{
		Erro_angle = -5;
	}
	else if(power_limit == 80 && ChassisCtrl.Mode == ROTING)
	{
		Erro_angle = -24;
	}
	else if(power_limit >= 100 && ChassisCtrl.Mode == ROTING || real_power_limit==150)
	{
		Erro_angle = -25;
	}
	else if(ChassisCtrl.Mode == ROTING)
	{
		Erro_angle = -4;
	}
	else
	{
		Erro_angle = 0;
	}
	
	
	del = FallowAngle - ChassisCtrl.Yaw->angle + Erro_angle;
	

	BufferFunctionCalc(&BufferFunctionX,PTZ.FBSpeed/32767.f);
	BufferFunctionCalc(&BufferFunctionY,-PTZ.LRSpeed/32767.f);
	BufferFunctionX.out = PTZ.FBSpeed/32767.f;
	BufferFunctionY.out = -PTZ.LRSpeed/32767.f;
	
	ChassisCtrl.vx = -BufferFunctionX.out * arm_cos_f32(del/180*PI) + BufferFunctionY.out * arm_sin_f32(del/180*PI);
	ChassisCtrl.vy = BufferFunctionX.out * arm_sin_f32(del/180*PI) + BufferFunctionY.out * arm_cos_f32(del/180*PI);
	
	if(ChassisCtrl.Mode == ROTING)
	{
		//设置速度等级/旋转等级
		if(power_limit == 60)
		{
			rote_powkp = 1.1;
			if(PTZ.FBSpeed !=0 || PTZ.LRSpeed !=0)
			{
				roting_speed = 0.35;
			}
			else
			{
				roting_speed = 0.54;
			}
		}
		
		else if(power_limit == 80)
		{
			rote_powkp = 2;
			if(PTZ.FBSpeed !=0 || PTZ.LRSpeed !=0)
			{
				roting_speed = 0.60;
			}
			else
			{
				roting_speed = 0.70;
			}
		}
		else if(power_limit >= 100 || real_power_limit == 150)
		{
			rote_powkp = 3;
			if(PTZ.FBSpeed !=0 || PTZ.LRSpeed !=0)
			{
				roting_speed = 0.82;
			}
			else
			{
				roting_speed = 0.77;
			}
		}
		else
		{
			rote_powkp = 1;
			if(PTZ.FBSpeed !=0 || PTZ.LRSpeed !=0)
			{
				roting_speed = 0.25;
			}
			else
			{
				roting_speed = 0.45;
			}
		}
		ChassisCtrl.wz = roting_speed;
	}
	else if(ChassisCtrl.Mode == FALLOW||ChassisCtrl.Mode == STOP)
	{
		//设置速度等级
		if(power_limit == 60)
		{
			rote_powkp = 1.8;
		}
		
		else if(power_limit == 80)
		{
			rote_powkp = 2.2;
		}
		else if(power_limit >= 100 || real_power_limit == 150)
		{
			rote_powkp = 2.6;
		}
		else
		{
			rote_powkp = 1.5;
		}
		ChassisCtrl.wz =  PID_calc(&ChassisCtrl.WZPid,ChassisCtrl.Yaw->angle,FallowAngle);
	}
	
	//慢速模式
	if((PTZ.ChassisStatueRequest & 0x02 )&&(PTZ.ChassisStatueRequest & 0x40 )) rote_powkp=0.2;
	ChassisCtrl.vx *= rote_powkp;
	ChassisCtrl.vy *= rote_powkp;
	ChassisCtrl.wz *=3;
}


void ChassisControlLoop()
{
	if(ChassisCtrl.Mode == NOFORCE)
	{
		for(int i = 0;i<4;i++)
			ChassisCtrl.Current[i] = 0;
		return;
	}
	
	else if(ChassisCtrl.Mode == STOP)
	{
		memset(ChassisCtrl.WheelSpeed,0,sizeof(ChassisCtrl.WheelSpeed));
		for(int i =0;i<4;i++)
		{
			ChassisCtrl.Current[i] =  PID_calc(&ChassisCtrl.XYPid[i], ChassisCtrl.Motor[i]->speed_rpm * 0.000415809748903494517209f, ChassisCtrl.WheelSpeed[i]);
		}
	}
	else if(ChassisCtrl.Mode == FALLOW || ChassisCtrl.Mode == ROTING)
	{
		ChassisCtrl.WheelSpeed[0] = -ChassisCtrl.vx - ChassisCtrl.vy + ChassisCtrl.wz;
		ChassisCtrl.WheelSpeed[1] = ChassisCtrl.vx - ChassisCtrl.vy +  ChassisCtrl.wz;
		ChassisCtrl.WheelSpeed[2] = ChassisCtrl.vx + ChassisCtrl.vy + 	ChassisCtrl.wz;
		ChassisCtrl.WheelSpeed[3] = -ChassisCtrl.vx + ChassisCtrl.vy + ChassisCtrl.wz;
		 
		for(int i =0;i<4;i++)
		{
			ChassisCtrl.Current[i] =  PID_calc(&ChassisCtrl.XYPid[i], ChassisCtrl.Motor[i]->speed_rpm * 0.000415809748903494517209f, ChassisCtrl.WheelSpeed[i]);
		}
	}
}


void BufferFunctionInit(BufferFunction_t *BufferFunction,fp32 frame_period)
{
	BufferFunction->frame_period = frame_period;
	BufferFunction->input = 0;
	BufferFunction->out = 0;
	BufferFunction->error = 0;
	BufferFunction->buffer = 0;
}

void BufferFunctionCalc(BufferFunction_t *BufferFunction,fp32 input)
{
	BufferFunction->input = input;
	BufferFunction->error = BufferFunction->input - BufferFunction->out;
	if(BufferFunction->error >= 0)
	{
		BufferFunction->buffer = BufferFunction->error / (exp (0.7*BufferFunction->error));//0.7越小越往上飞
		BufferFunction->out += BufferFunction->buffer / BufferFunction->frame_period;
	}
	else
	{
		BufferFunction->buffer = fabs(BufferFunction->error) / (exp (0.7*fabs(BufferFunction->error)));//0.7越小越往上飞
		BufferFunction->out -= BufferFunction->buffer / BufferFunction->frame_period;
	}
}
