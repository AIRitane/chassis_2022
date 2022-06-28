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
	if(power_limit == 0) power_limit=50;
	robot_level = robot_state.robot_level;
	
	//超级电容充电电流
	chassis_volt = (fp32)power_heat_data_t.chassis_volt / 1000;
	available_current = power_limit/ chassis_volt;
	remain_current = available_current - (fp32)power_heat_data_t.chassis_current / 1000;
	
	if(remain_current<0 || CapChageVoltage>19.5)
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
	//计算最终限制功率
//	robot_level = 3;
	if(chassis_power_buffer < real_buffer_limit)
		real_power_limit = (chassis_power_buffer/real_buffer_limit)*real_buffer_limit;
	else
	{
		if(robot_level  == 1)
		{
			real_power_limit = 60;
		}
		else if(robot_level  == 2)
		{
			real_power_limit = 80;
		}
		else if(robot_level  == 3)
		{
			real_power_limit = 100;
		}
		else
		{
			real_power_limit = 60;
		}
	}
	
	//计算拟合功率
	total_power = 0;
	watch_pre_current = 0;
	for(uint32_t i=0;i<4;i++)
	{
		watch_pre_current += fabs(ChassisCtrl.Current[i]);
		total_power += fabs(ChassisCtrl.Current[i] * ChassisCtrl.Motor[i]->speed_rpm)*total_powerKP;
	}
	
		//选择电路
	if((PTZ.ChassisStatueRequest & 0x80) && CapChageVoltage>14 && CMSCounter >100)
	{
		CMS_Hub.power_routin = CMS_PR_BuckBoost;
		real_power_limit = 150;
	}
	else
	{
		CMS_Hub.power_routin = CMS_PR_BattDirect;
	}
	
	//计算减幅系数
	if(total_power>real_power_limit)
	{
		SupKp = real_power_limit/total_power;
		
		if(ChassisCtrl.Mode == ROTING && (PTZ.FBSpeed == 0 && PTZ.LRSpeed == 0)) 
		{
			if(power_limit < 90 && power_limit > 65) SupKp*=1.2;
			else if(power_limit > 90) SupKp*=1.5;
		}
		else if(ChassisCtrl.Mode == ROTING)
		{
			if(power_limit < 90 && power_limit > 58) SupKp*=1.3;
			else if(power_limit > 90) SupKp*=1.7;
		}
		else if(ChassisCtrl.Mode == FALLOW)
		{
			if(power_limit < 90 && power_limit > 58) SupKp*=1.2;
			else if(power_limit > 90) SupKp*=1.5;
		}
		
		for(uint32_t i=0;i<4;i++)
		{
			ChassisCtrl.Current[i]*=SupKp;
		}
	}
}

void ChassisPowerControl()
{
	ChassisPowerloop();
	ChassisReduceRate();
}
