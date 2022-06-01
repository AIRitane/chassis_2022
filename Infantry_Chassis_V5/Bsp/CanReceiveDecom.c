/*
* 本代码主要负责底盘电机数据的接收以及云台下发数据的接收
* 分为数据接收初始化、电机数据接收、云台数据下发接收
*	已实现：本部分亮灯检测只支持CAN1总线的通信失败，CAN2总线通信检测未执行
*	未实现：后期考虑使用定时器定频检测数据的变化
*/
#include "can.h"
#include "gpio.h"
#include "string.h"
#include "CanReceiveDecom.h"

#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
        (ptr)->angle = (ptr)->ecd / 8191.0f *360 - 180;          		\
		(ptr)->speed = (ptr)->speed_rpm * 360.0f;						\
    }
	
motor_measure_t ChassisMotor[4];
motor_measure_t YawMotor;
Aim_t Aim;
PTZ_t PTZ;
	
/*---------------------------------------数据接收初始化-------------------------------------*/
void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/*---------------------------------------CAN的接收回调函数-------------------------------------*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	/*-------------------------------------电机数据的接收-------------------------------------*/
	//底盘电机
	if(hcan->Instance == CAN2)
	{
		switch(rx_header.StdId)
		{
			case CanMotor1Id:
			case CanMotor2Id:
			case CanMotor3Id:
			case CanMotor4Id:
			{
				static uint8_t i = 0;
				i = rx_header.StdId - CanMotor1Id;
				get_motor_measure(&ChassisMotor[i], rx_data);
				
				break;
			}
			default:
			{
				//CAN1线上只会挂载四个底盘电机，因此一旦进入CNA1但是并未收到数据，判为错
				ComLedError();
				break;
			}
		}
	}
	
	else if(hcan->Instance == CAN1)
	{
		switch(rx_header.StdId)
		{
			//云台YAW电机
			case YawMotorId:
			{
				get_motor_measure(&YawMotor, rx_data);
				
				break;
			}
			/*-------------------------------------云台数据下发接收-------------------------------------*/
//			case DefaultAimStatusAndTargetId:
//			{
//				memcpy(&Aim,rx_data,sizeof(Aim_t));
//			}
//			case SentryAimStatusAndTargetId:
//			{
//				//哨兵相关，暂时不管
//			}
			case DefaulPTZRequestAndStatusId:
			{
				memcpy(&PTZ,rx_data,sizeof(PTZ_t));
			}
			case SentryPTZRequestAndStatusId:
			{
				//哨兵相关，不管
			}
			default:
			{
				//考虑到CAN2总线上还有其他设备，不进行亮灯检测
				break;
			}
		}
	}
}

const motor_measure_t *GetChassisMeasure(uint8_t i)
{
	return &ChassisMotor[i];
}
const motor_measure_t *GetYawMeasure(void)
{
	return &YawMotor;
}
