/*
* 
*/

#include "ChassisPowerBehaviour.h"
#include "RefereeBehaviour.h"
#include "CMSInterface.h"
#include "math.h"

float BufferEnergy = 60; //底盘缓冲能量

float SupKp = 1;

//计算得到
float remain_current;				//当前剩余给超级电容的电流
float available_current;			//当前可用电流


//裁判系统得到数值
fp32 chassis_volt = 0;   //单位是V
float power_limit = 0;
fp32 chassis_power = 0.0f;
fp32 chassis_power_buffer = 0.0f;
fp32 robot_level = 1;


//功率计算
fp32 real_buffer_limit = 20;//可调整
fp32 real_power_limit = 0;	//不可调整
fp32 total_power = 0.0f;
fp32 total_powerKP = 2.80099994e-06;

extern ChassisCtrl_t ChassisCtrl;


void ChassisPowerloop();
void ChassisReduceRate();

//刷新功率数据
void ChassisPowerloop()
{
	chassis_power = power_heat_data_t.chassis_power;
	chassis_power_buffer = power_heat_data_t.chassis_power_buffer;
	power_limit = robot_state.chassis_power_limit;
	robot_level = robot_state.robot_level;
	
	//超级电容充电电流
	chassis_volt = (fp32)power_heat_data_t.chassis_volt / 1000;
	available_current = power_limit/ chassis_volt;
	remain_current = available_current - (fp32)power_heat_data_t.chassis_current / 1000;
	
	if(remain_current<0)
	{
		remain_current = 0;
	}
	
	CMS_Hub.LimitCurrent = remain_current*1000;
}
fp32 watch_pre_current;
fp32 watch_new_current;
extern uint32_t CMSCounter;
void ChassisReduceRate()
{
	//选择电路
	if((PTZ.ChassisStatueRequest & 0x80) && CapChageVoltage>14 && CMSCounter >100)
	{
		CMS_Hub.power_routin = CMS_PR_BuckBoost;
		robot_level = 3;
	}
	else
	{
		CMS_Hub.power_routin = CMS_PR_BattDirect;
	}
	
	//计算最终限制功率
	if(chassis_power_buffer < real_buffer_limit)
		real_power_limit = (chassis_power_buffer/real_buffer_limit)*real_buffer_limit/2.0f;
	else
	{
		real_power_limit = power_limit;
	}
	
	//计算拟合功率
	total_power = 0;
	watch_pre_current = 0;
	for(uint32_t i=0;i<4;i++)
	{
		watch_pre_current += fabs(ChassisCtrl.Current[i]);
		total_power += fabs(ChassisCtrl.Current[i] * ChassisCtrl.Motor[i]->speed_rpm)*total_powerKP;
	}
	
	//计算减幅系数
	if(total_power>real_power_limit)
	{
		SupKp = real_power_limit/total_power;
		if(ChassisCtrl.Mode == ROTING && chassis_power_buffer == 60)
		{
			for(uint32_t i=0;i<4;i++)
			{
				ChassisCtrl.Current[i]*= pow(SupKp,0.5);
			}
		}
		else
		{
			for(uint32_t i=0;i<4;i++)
			{
				ChassisCtrl.Current[i]*=SupKp;
			}
		}

	}
	
	watch_new_current = 0;
	for(uint32_t i=0;i<4;i++)
	{
		watch_new_current += fabs(ChassisCtrl.Current[i]);
	}
}

void ChassisPowerControl()
{
	ChassisPowerloop();
	ChassisReduceRate();
}
