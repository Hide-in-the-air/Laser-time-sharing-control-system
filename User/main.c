#include "bsp.h"			/* 底层硬件驱动 */
#include <stdbool.h>

/*
//Nimma PORT
#define NIMMA_PORT_E  			GPIOE
//Nimma Flash + Q-Switch
#define N_FLASH_PIN13		GPIO_PIN_13
#define N_QSWITCH_PIN15	GPIO_PIN_15

//nimma600 PORT
#define nimma600_PORT_B  	GPIOB

//nimma600 Flash + Q-Switch
#define B_FLASH_PIN		GPIO_PIN_13
#define B_QSWITCH_PIN	GPIO_PIN_15
*/

//Q-Switch PORT
#define Q_PORT_E  		GPIOE
//Q-Switch
#define N9_Q_PIN13			GPIO_PIN_13	//For Nimma900 Laser
#define N6_Q_PIN15			GPIO_PIN_15	//For nimma600 Laser

//Flash PORT
#define F_PORT_B  		GPIOB

//Flash  
#define N9_F_PIN11			GPIO_PIN_11	//For Nimma900 Laser
#define N6_F_PIN13			GPIO_PIN_13	//For nimma600 Laser

typedef struct __COM_LASER__
{
	char freq : 4;
	char bshot : 1;
  char id : 3;
}com_laser;

typedef union __COM_U_LASER__
{
    com_laser cmd;
    char data;
}u_com_laser;

typedef struct __COM_LASERS__
{
		char head;
    u_com_laser cmd[2];
    char tail;
}com_lasers;


unsigned char com_pc_buff[20];
unsigned char com_nimma900_buff[20];
unsigned char com_nimma600_buff[20];


static unsigned char rev_pc_num;
static unsigned char rev_nimma900_num;
static unsigned char rev_nimma600_num;
static int32_t timer_count;

u_com_laser nimma900_cmd;
u_com_laser nimma600_cmd;

static int32_t nimma900_prd;
static int32_t nimma600_prd;

static void PrintfLogo(void);
void bsp_InitLaserPort(void);

void rev_from_pc(uint8_t _byte);
void clear_pc_com_buff(void);

void rev_from_nimma900(uint8_t _byte);
void clear_nimma900_com_buff(void);

void rev_from_nimma600(uint8_t _byte);
void clear_nimma600_com_buff(void);

void cmd_exec(uint8_t _laser1, uint8_t _laser2);
void lasers_timer_stick(void);


/*
*********************************************************************************************************
*	函 数 名: main
*	功能说明: c程序入口
*	形    参: 无
*	返 回 值: 错误代码(无需处理)
*********************************************************************************************************
*/
int main(void)
{

	bsp_Init();		/* 硬件初始化 */
	
	//初始化触发引脚
	bsp_InitLaserPort();
	
	PrintfLogo();	/* 打印例程名称和版本等信息 */
	clear_pc_com_buff();
	clear_nimma900_com_buff();

	/* 先做个LED1的亮灭显示 */
	bsp_LedOn(1);
	bsp_DelayMS(100);
	bsp_LedOff(1);
	bsp_DelayMS(100);
	
	bsp_StartAutoTimer(0, 100); /* 启动1个100ms的自动重装的定时器 */
	bsp_StartAutoTimer(1, 500);	/* 启动1个500ms的自动重装的定时器 */
	
	/* 进入主程序循环体 */
	while (1)
	{
		bsp_Idle();		/* 这个函数在bsp.c文件。用户可以修改这个函数实现CPU休眠和喂狗 */

		/* 判断定时器超时时间 */
		if (bsp_CheckTimer(0))	
		{
			/* 每隔100ms 进来一次 */  
			bsp_LedToggle(1);			
			//dbg comSendChar(COM1, 0x55);
		}
		
		/* 判断定时器超时时间 */
		if (bsp_CheckTimer(1))	
		{
			/* 每隔500ms 进来一次 */ 
			bsp_LedToggle(2);			
		}
		
		//测试触发引脚输出
		{
			/*
			for(int i = 0; i < 100000; i++);
			F_PORT_B->BSRR = (uint32_t)N_F_PIN11;
			for(int i = 0; i < 2002; i++)__NOP; //60us
			F_PORT_B->BSRR = (uint32_t)N_F_PIN11 << 16U;
			
			for(int i = 0; i < 5000; i++)__NOP;
			Q_PORT_E->BSRR = (uint32_t)N_Q_PIN13;
			for(int i = 0; i < 2002; i++)__NOP; //60us
			Q_PORT_E->BSRR = (uint32_t)N_Q_PIN13 << 16U;
			
			//for(int i = 0; i < 100000; i++);
			
			
			//Q_PORT_E->BSRR = (uint32_t)B_Q_PIN15 << 16U;
			//for(int i = 0; i < 1000; i++)
			//Q_PORT_E->BSRR = (uint32_t)B_Q_PIN15;
			//for(int i = 0; i < 1000; i++)
			//Q_PORT_E->BSRR = (uint32_t)B_Q_PIN15 << 16U;
			*/
		}
		
	}
}


/*
*********************************************************************************************************
*	函 数 名: bsp_InitLaserPort
*	功能说明: 配置Laser相关的GPIO,  该函数被直接调用。
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitLaserPort(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	/* 打开GPIO时钟 */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	
	/*
		配置所有的LED指示灯GPIO为推挽输出模式
		由于将GPIO设置为输出时，GPIO输出寄存器的值缺省是0，因此会驱动LED点亮.
		这是我不希望的，因此在改变GPIO为输出前，先关闭LED指示灯
	*/
	//配置所有输出为低
	Q_PORT_E->BSRR = (uint32_t)N9_Q_PIN13 << 16U;
	Q_PORT_E->BSRR = (uint32_t)N6_Q_PIN15 << 16U;
	
	F_PORT_B->BSRR = (uint32_t)N9_F_PIN11 << 16U;
	F_PORT_B->BSRR = (uint32_t)N6_F_PIN13 << 16U;

	/* 配置LED */
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   		/* 设置推挽输出 */
	GPIO_InitStruct.Pull = GPIO_NOPULL;               /* 上下拉电阻不使能 */
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  		/* GPIO速度等级 */

	GPIO_InitStruct.Pin = N9_Q_PIN13;
	HAL_GPIO_Init(Q_PORT_E, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = N6_Q_PIN15;
	HAL_GPIO_Init(Q_PORT_E, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = N9_F_PIN11;
	HAL_GPIO_Init(F_PORT_B, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = N6_F_PIN13;
	HAL_GPIO_Init(F_PORT_B, &GPIO_InitStruct);
	
}

/*
*********************************************************************************************************
*	函 数 名: clear_pc_com_buff
*	功能说明: 清理通信缓冲区
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void clear_pc_com_buff(void)
{
	for(int i = 0; i<20; i++)
	{
		com_pc_buff[i] = 0;
	}
	
	rev_pc_num = 0;
}

/*
*********************************************************************************************************
*	函 数 名: clear_nimma900_com_buff
*	功能说明: 清理nimma900通信缓冲区
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void clear_nimma900_com_buff(void)
{
	
	for(int i = 0; i<20; i++)
	{
		com_nimma900_buff[i] = 0;
	}
	
	rev_nimma900_num = 0;
}

/*
*********************************************************************************************************
*	函 数 名: clear_nimma600_com_buff
*	功能说明: 清理nimma600通信缓冲区
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void clear_nimma600_com_buff(void)
{
	nimma900_cmd.data = 0;
	nimma600_cmd.data = 0;
	
	for(int i = 0; i<20; i++)
	{
		com_nimma600_buff[i] = 0;
	}
	
	rev_nimma600_num = 0;
}

/*
*********************************************************************************************************
*	函 数 名: PrintfLogo
*	功能说明: 打印例程名称和例程发布日期, 接上串口线后，打开PC机的超级终端软件可以观察结果
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void PrintfLogo(void)
{
	
	printf("\r\n*************************************************************\r\n");
	//printf("\r\n*\r\n");
	
	//comSendChar(COM1, 0xaa);
	//comSendChar(COM1, 0xaa);
	
	/* 检测CPU ID 
	{
		uint32_t CPU_Sn0, CPU_Sn1, CPU_Sn2;
		
		CPU_Sn0 = *(__IO uint32_t*)(0x1FFF7A10);
		CPU_Sn1 = *(__IO uint32_t*)(0x1FFF7A10 + 4);
		CPU_Sn2 = *(__IO uint32_t*)(0x1FFF7A10 + 8);

		printf("CPU : STM32F407IGT6, LQFP176, 主频: %dMHz\r\n", SystemCoreClock / 1000000);
		printf("UID = %08X %08X %08X\n\r", CPU_Sn2, CPU_Sn1, CPU_Sn0);
	}
		
	printf("*************************************************************\n\r");
	*/
}

/*
*********************************************************************************************************
*	函 数 名: rev_from_pc
*	功能说明: 接收到由PC发出的新数据
*	形    参: _byte 接收到的新数据
*	返 回 值: 无
*********************************************************************************************************
*/
void rev_from_pc(uint8_t _byte)
{
	//printf("\r\n %s", g_tUart1.pRxBuf);
	rev_pc_num++;
	com_pc_buff[rev_pc_num-1] = _byte;
	
	unsigned char temp_buff[20];
	
	//dbg
	//printf("\r\n %d", rev_pc_num);
	//return the received
	//comSendChar(COM2, _byte);
	
	if(0xF0 == _byte)//全部停止
	{
		//停止频率输出
		
		//清数据
	}
	if(0xE0 == (_byte&0xF0))//全部启动
	{
		//启动频率输出
		
		//清数据
	}
	
	if(0x3C == _byte)//判断是Nimma的数据
	{
		//效验前几位数据，确认数据包
		if( 0x01 == com_pc_buff[1] && 0xc3 == com_pc_buff[rev_pc_num-2])
		{
			for(int i = 0; i < rev_pc_num-1; i++)
			{
				temp_buff[i] = com_pc_buff[i+1];
			}
				
			//从第0字节看是哪个激光器
			if( 0x90 == com_pc_buff[0]) //Nimma900
			{
				//若是则发至Nimma900 COM1
				comSendBuf(COM1, temp_buff, rev_pc_num-1);	
			}
		
			if( 0x60 == com_pc_buff[0])	//Nimma600
			{
				//若是则发至Nimma600 COM6
				comSendBuf(COM6, temp_buff, rev_pc_num-1);	
			}

		}
		
		//清理接收到的数据
		clear_pc_com_buff();		
	}
		
	if(76 == _byte)//判断是控制指令的数据 L
	{
		//效验前1位数据，确认数据包 C
		if(67 == com_pc_buff[0]) 
		{
			//若是则由指令处理器进行处理
			cmd_exec(com_pc_buff[1], com_pc_buff[2]);			
		}
		
		//清理接收到的数据
		clear_pc_com_buff();	
	}
	
}


/*
*********************************************************************************************************
*	函 数 名: rev_from_nimma900
*	功能说明: 接收到由nimma900发出的新数据
*	形    参: _byte 接收到的新数据
*	返 回 值: 无
*********************************************************************************************************
*/
void rev_from_nimma900(uint8_t _byte)
{
	//printf("\r\n %s", g_tUart1.pRxBuf);
	rev_nimma900_num++;
	com_nimma900_buff[rev_nimma900_num-1] = _byte;
	
	//dbg
	//printf("\r\n %d", rev_pc_num);
	//return the received
	//comSendChar(COM2, _byte);
	
	if(0x3C == _byte)//判断是Nimma的数据
	{
		//效验前几位数据，确认数据包
		if( 0x01 == com_nimma900_buff[0] && 0xc3 == com_nimma900_buff[rev_nimma900_num-2])
		{
			//加入尾部标识
			com_nimma900_buff[rev_nimma900_num] = 0x90;
			comSendBuf(COM2, com_nimma900_buff, rev_nimma900_num+1);		
			//dbg@0521
			//comSendBuf(COM4, com_nimma900_buff, rev_nimma900_num);			
			
		}
		
		//清理接收到的数据
		clear_nimma900_com_buff();		
	}
}

/*
*********************************************************************************************************
*	函 数 名: rev_from_nimma600
*	功能说明: 接收到由nimma600发出的新数据
*	形    参: _byte 接收到的新数据
*	返 回 值: 无
*********************************************************************************************************
*/
void rev_from_nimma600(uint8_t _byte)
{
	//printf("\r\n %s", g_tUart1.pRxBuf);
	rev_nimma600_num++;
	com_nimma600_buff[rev_nimma600_num-1] = _byte;
	
	//dbg
	//printf("\r\n %d", rev_pc_num);
	//return the received
	
	if(0x3C == _byte)//判断是Nimma的数据
	{
		//效验前几位数据，确认数据包
		if( 0x01 == com_nimma600_buff[0] && 0xc3 == com_nimma600_buff[rev_nimma600_num-2])
		{
			//加入尾部标识
			com_nimma600_buff[rev_nimma600_num] = 0x60;
			comSendBuf(COM2, com_nimma600_buff, rev_nimma600_num+1);		
			//dbg@0521
			//comSendBuf(COM4, com_nimma900_buff, rev_nimma900_num);			
			
		}
		
		//清理接收到的数据
		clear_nimma600_com_buff();		
	}
}

/*
*********************************************************************************************************
*	函 数 名: cmd_exec
*	功能说明: 处理指令
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void cmd_exec(uint8_t _laser1, uint8_t _laser2)
{
	nimma900_cmd.data = _laser1;
	nimma600_cmd.data = _laser2;
	
	if( 1 == nimma900_cmd.cmd.bshot || 1 == nimma600_cmd.cmd.bshot)
	{
		timer_count = 0;
		
		nimma900_prd = MINI_TIMER_USPRD/nimma900_cmd.cmd.freq;
		nimma600_prd = MINI_TIMER_USPRD/nimma600_cmd.cmd.freq;
		
		//启动timer 1000 = 1ms; 100000 = 100ms; 0则采用最小中断周期
		bsp_StartHardTimer(2, 0, lasers_timer_stick);
	}
	else
	{
		//停止timer 1000 = 1ms; 100000 = 100ms
		bsp_StopTimer(2);		
	}

}

void lasers_timer_stick(void)
{
	int j = 0;
	//同时出光
	if( (1 == nimma900_cmd.cmd.bshot) && (1 == nimma600_cmd.cmd.bshot) && (nimma900_prd == nimma600_prd))
	{
		//出光
		if(timer_count%nimma900_prd == 0)
		{
			//测试触发引脚输出
			{
				//for(int i = 0; i < 100000; i++);
				F_PORT_B->BSRR = (uint32_t)(N9_F_PIN11|N6_F_PIN13);
				for(int i = 0; i < 2002; i++)__NOP; //60us
				F_PORT_B->BSRR = (uint32_t)((N9_F_PIN11 << 16U)|(N6_F_PIN13 << 16U));
				
				//for(int i = 0; i < 5000; i++)__NOP;	//210us
				for(int i = 0; i < 4802; i++)j++;	//200us
				Q_PORT_E->BSRR = (uint32_t)(N9_Q_PIN13|N6_Q_PIN15);
				for(int i = 0; i < 2002; i++)__NOP; //60us
				Q_PORT_E->BSRR = (uint32_t)((N9_Q_PIN13 << 16U) | (N6_Q_PIN15 << 16U));
				
				/*
				//for(int i = 0; i < 100000; i++);
				F_PORT_B->BSRR = (uint32_t)N6_F_PIN13;
				for(int i = 0; i < 2002; i++)__NOP; //60us
				F_PORT_B->BSRR = (uint32_t)N6_F_PIN13 << 16U;
				
				//for(int i = 0; i < 5000; i++)__NOP;	//210us
				for(int i = 0; i < 4002; i++)j++;	//200us
				Q_PORT_E->BSRR = (uint32_t)N6_Q_PIN15;
				for(int i = 0; i < 2002; i++)__NOP; //60us
				Q_PORT_E->BSRR = (uint32_t)N6_Q_PIN15 << 16U;
				*/
			}
		}
	}
		
	timer_count++;
}