#include "RefereeTask.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "CRC8_CRC16.h"
#include "fifo.h"
#include "bsp_referee.h"
#include "RefereeBehaviour.h"
#include "RM_Cilent_UI.h"
#include "CanReceiveDecom.h"

Graph_Data deng1,kuang,deng2,deng3,deng4,fu,imagex,imagex1,imagex2,imagex3,imagey,imagey1,imagey2,imagey3,imagey4;
String_Data bullet,bullet3,DAFU,Abuff,Pbuff,Cbuff,state,capacity,ZIMIAO,dafustate,zimiaostate,dafustate1,zimiaostate1,state4;
Float_Data capacityD,Min,Sec;

 int Time=600,M=10,S=0;
 void UI(void const * argument)
 {
	 while(1)
    {
				Rectangle_Draw(&kuang,"kuang",UI_Graph_ADD,0,UI_Color_White,1,6750,380,7460,700);
				UI_ReFresh(1,kuang);
			//准星
				Line_Draw(&imagex,"xck",UI_Graph_ADD,0,UI_Color_Green,2,7010,540,7200,540);
				Line_Draw(&imagex1,"xck1",UI_Graph_ADD,1,UI_Color_Green,2,7030,500,7180,500);
				Line_Draw(&imagex2,"xck2",UI_Graph_ADD,2,UI_Color_Green,2,7050,460,7160,460);
				Line_Draw(&imagex3,"xck3",UI_Graph_ADD,3,UI_Color_Green,2,7070,420,7140,420);
        Line_Draw(&imagey3,"yck3",UI_Graph_ADD,4,UI_Color_Green,2,7045,536,7045,544);
        Line_Draw(&imagey1,"yck1",UI_Graph_ADD,5,UI_Color_Green,2,7080,536,7080,544);
				Line_Draw(&imagey2,"yck2",UI_Graph_ADD,6,UI_Color_Green,2,7130,536,7130,544);
				Line_Draw(&imagey4,"yck4",UI_Graph_ADD,7,UI_Color_Green,2,7165,536,7165,544);
				Line_Draw(&imagey,"yck",UI_Graph_ADD,0,UI_Color_Green,2,7105,600,7105,400);
			
				UI_ReFresh(7,imagex,imagex1,imagex2,imagex3,imagey,imagey3,imagey4);
			
				UI_ReFresh(2,imagey1,imagey2);
			//字符
				Char_Draw(&capacity,"capacity",UI_Graph_ADD,4,UI_Color_Pink,27,8,2,6870,300,"capacity");
				Char_ReFresh(capacity);
				Float_Draw(&capacityD,"capacityD",UI_Graph_ADD,3,UI_Color_Cyan,27,2,5,7120,300,1000);
				UI_ReFresh(1,capacityD);
				Char_Draw(&DAFU,"DAFU",UI_Graph_ADD,4,UI_Color_Pink,27,4,2,6950,240,"DAFU");
				Char_ReFresh(DAFU);
        Char_Draw(&ZIMIAO,"ZIMIAO",UI_Graph_ADD,4,UI_Color_Pink,27,6,2,6930,180,"aimbot");
				Char_ReFresh(ZIMIAO);
                 
                

				if((PTZ.PTZStatusInformation & 0x40) == 0x40)               //弹舱盖开
				{
					Circle_Draw(&deng3,"deng3",UI_Graph_ADD,7,UI_Color_Green,2,6800,150,40);
					Char_Draw(&bullet3,"bullet3",UI_Graph_ADD,5,UI_Color_Green,10,5,2,6780,153,"open");
					UI_ReFresh(1,deng3);
					Char_ReFresh(bullet3);
				}
        else
                {
					Circle_Draw(&deng3,"deng3",UI_Graph_Del,7,UI_Color_Green,2,6800,150,40);
					Char_Draw(&bullet3,"bullet3",UI_Graph_Del,5,UI_Color_Green,10,5,2,6780,153,"open");
					UI_ReFresh(1,deng3);
					Char_ReFresh(bullet3);
                }
         
				if((PTZ.PTZStatusInformation & 0x10) == 0x10)          //打大幅
				{
          Char_Draw(&dafustate1,"dafustate1",UI_Graph_Del,1,UI_Color_Pink,27,4,2,7120,240,"NONE");
					Char_ReFresh(dafustate1);
					Char_Draw(&dafustate,"dafustate",UI_Graph_ADD,2,UI_Color_Cyan,27,2,2,7120,240,"OK");
					Char_ReFresh(dafustate);
				}
				else
				{
          Char_Draw(&dafustate,"dafustate",UI_Graph_Del,1,UI_Color_Cyan,27,2,2,7120,240,"OK");
					Char_ReFresh(dafustate);
					Char_Draw(&dafustate1,"dafustate1",UI_Graph_ADD,2,UI_Color_Pink,27,4,2,7120,240,"NONE");
					Char_ReFresh(dafustate1);
				}
				if((PTZ.PTZStatusInformation & 0x08) == 0x08)          //自瞄
				{
          Char_Draw(&zimiaostate1,"zimiaostate1",UI_Graph_Del,1,UI_Color_Pink,27,4,2,7120,180,"NONE");
					Char_Draw(&zimiaostate,"zimiaostate",UI_Graph_ADD,3,UI_Color_Cyan,27,4,2,7120,180,"OK");
					Char_ReFresh(zimiaostate);		
					Char_ReFresh(zimiaostate1);					
				}
				else
				{
          Char_Draw(&zimiaostate,"zimiaostate",UI_Graph_Del,1,UI_Color_Cyan,27,4,2,7120,180,"OK");
					Char_Draw(&zimiaostate1,"zimiaostate1",UI_Graph_ADD,3,UI_Color_Pink,27,4,2,7120,180,"NONE");
					Char_ReFresh(zimiaostate1);
					Char_ReFresh(zimiaostate);
				}
                
                
				if((PTZ.ChassisStatueRequest & 0x10) == 0x10)					//底盘运动
				{
          //Circle_Draw(&deng4,"deng4",UI_Graph_Del,9,UI_Color_White,2,7400,150,40);
					Char_Draw(&state4,"state4",UI_Graph_Del,7,UI_Color_Yellow,10,6,2,7376,153,"follow");
					Circle_Draw(&deng2,"deng2",UI_Graph_ADD,8,UI_Color_Green,2,7400,150,40);
					Char_Draw(&state,"state",UI_Graph_ADD,5,UI_Color_Green,10,6,2,7376,153,"rotate");
					UI_ReFresh(2,deng2);
					Char_ReFresh(state4);
					Char_ReFresh(state);
				}
				if((PTZ.ChassisStatueRequest & 0x08) == 0x08)
				{
          Circle_Draw(&deng2,"deng2",UI_Graph_Del,8,UI_Color_Green,2,7400,150,40);
					Char_Draw(&state,"state",UI_Graph_Del,5,UI_Color_Green,10,6,2,7376,153,"rotate");
					
					Char_ReFresh(state);
					//Circle_Draw(&deng4,"deng4",UI_Graph_ADD,9,UI_Color_White,2,7400,150,40);
					Char_Draw(&state4,"state4",UI_Graph_ADD,7,UI_Color_Yellow,10,6,2,7376,153,"follow");
					UI_ReFresh(2,deng2);
					Char_ReFresh(state4);
				}
               
				if((buff_musk_t.power_rune_buff & 0x02) == 0x02)                //增益
				{
					Char_Draw(&Cbuff,"Cbuff",UI_Graph_ADD,5,UI_Color_Pink,20,13,2,7680,380,"COOLDOWN BUFF");
          Char_ReFresh(Cbuff);
				}
				else
				{
						Char_Draw(&Cbuff,"Cbuff",UI_Graph_Del,5,UI_Color_Pink,20,13,2,7680,380,"COOLDOWN BUFF");
						Char_ReFresh(Cbuff);
				}
				if((buff_musk_t.power_rune_buff & 0x04) == 0x04)
				{
					Char_Draw(&Pbuff,"Pbuff",UI_Graph_ADD,5,UI_Color_Pink,20,12,2,7700,410,"Protect BUFF");
          Char_ReFresh(Pbuff);
				}
				else
				{
						Char_Draw(&Pbuff,"Pbuff",UI_Graph_Del,5,UI_Color_Pink,20,12,2,7700,410,"Protect BUFF");
						Char_ReFresh(Pbuff);
				}
				if((buff_musk_t.power_rune_buff & 0x08) == 0x08)
				{
					Char_Draw(&Abuff,"Abuff",UI_Graph_ADD,5,UI_Color_Pink,20,11,2,7720,440,"Attack BUFF");
          Char_ReFresh(Abuff);
				}
				else
				{
						Char_Draw(&Abuff,"Abuff",UI_Graph_Del,5,UI_Color_Pink,20,11,2,7720,440,"Attack BUFF");
						Char_ReFresh(Abuff);
				}
                               
				if((field_event.event_type & 0x00000020) == 0x00000020)             //能量机关间隔时间
                {
                    Time = game_state.stage_remain_time - 75;
                    M = Time/60;
                    S = Time%60;
                }
                if(game_state.stage_remain_time>Time)
                {
                    Float_Draw(&Min,"Min",UI_Graph_ADD,3,UI_Color_Yellow,27,2,5,7700,800,M*1000);
                    //UI_ReFresh(1,Min);
                    Float_Draw(&Sec,"Sec",UI_Graph_ADD,3,UI_Color_Yellow,27,2,5,7720,800,S*1000);
                    UI_ReFresh(2,Min,Sec);
									//UI_ReFresh(1,Sec);
                }
                else
                {
                    Float_Draw(&Min,"Min",UI_Graph_Del,3,UI_Color_Yellow,27,2,5,7700,800,M*1000);
                    Float_Draw(&Sec,"Sec",UI_Graph_Del,3,UI_Color_Yellow,27,2,5,7720,800,S*1000);
										UI_ReFresh(2,Min,Sec);
                    //UI_ReFresh(1,Sec);
                }
				
        osDelay(1);			//5ms的解算频率
    }

 }