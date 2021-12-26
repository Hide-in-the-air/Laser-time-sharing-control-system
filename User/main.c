#include "bsp.h"			/* �ײ�Ӳ������ */
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
*	�� �� ��: main
*	����˵��: c�������
*	��    ��: ��
*	�� �� ֵ: �������(���账��)
*********************************************************************************************************
*/
int main(void)
{

	bsp_Init();		/* Ӳ����ʼ�� */
	
	//��ʼ����������
	bsp_InitLaserPort();
	
	PrintfLogo();	/* ��ӡ�������ƺͰ汾����Ϣ */
	clear_pc_com_buff();
	clear_nimma900_com_buff();

	/* ������LED1��������ʾ */
	bsp_LedOn(1);
	bsp_DelayMS(100);
	bsp_LedOff(1);
	bsp_DelayMS(100);
	
	bsp_StartAutoTimer(0, 100); /* ����1��100ms���Զ���װ�Ķ�ʱ�� */
	bsp_StartAutoTimer(1, 500);	/* ����1��500ms���Զ���װ�Ķ�ʱ�� */
	
	/* ����������ѭ���� */
	while (1)
	{
		bsp_Idle();		/* ���������bsp.c�ļ����û������޸��������ʵ��CPU���ߺ�ι�� */

		/* �ж϶�ʱ����ʱʱ�� */
		if (bsp_CheckTimer(0))	
		{
			/* ÿ��100ms ����һ�� */  
			bsp_LedToggle(1);			
			//dbg comSendChar(COM1, 0x55);
		}
		
		/* �ж϶�ʱ����ʱʱ�� */
		if (bsp_CheckTimer(1))	
		{
			/* ÿ��500ms ����һ�� */ 
			bsp_LedToggle(2);			
		}
		
		//���Դ����������
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
*	�� �� ��: bsp_InitLaserPort
*	����˵��: ����Laser��ص�GPIO,  �ú�����ֱ�ӵ��á�
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitLaserPort(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	/* ��GPIOʱ�� */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	
	/*
		�������е�LEDָʾ��GPIOΪ�������ģʽ
		���ڽ�GPIO����Ϊ���ʱ��GPIO����Ĵ�����ֵȱʡ��0����˻�����LED����.
		�����Ҳ�ϣ���ģ�����ڸı�GPIOΪ���ǰ���ȹر�LEDָʾ��
	*/
	//�����������Ϊ��
	Q_PORT_E->BSRR = (uint32_t)N9_Q_PIN13 << 16U;
	Q_PORT_E->BSRR = (uint32_t)N6_Q_PIN15 << 16U;
	
	F_PORT_B->BSRR = (uint32_t)N9_F_PIN11 << 16U;
	F_PORT_B->BSRR = (uint32_t)N6_F_PIN13 << 16U;

	/* ����LED */
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   		/* ����������� */
	GPIO_InitStruct.Pull = GPIO_NOPULL;               /* ���������費ʹ�� */
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  		/* GPIO�ٶȵȼ� */

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
*	�� �� ��: clear_pc_com_buff
*	����˵��: ����ͨ�Ż�����
*	��    ��: ��
*	�� �� ֵ: ��
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
*	�� �� ��: clear_nimma900_com_buff
*	����˵��: ����nimma900ͨ�Ż�����
*	��    ��: ��
*	�� �� ֵ: ��
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
*	�� �� ��: clear_nimma600_com_buff
*	����˵��: ����nimma600ͨ�Ż�����
*	��    ��: ��
*	�� �� ֵ: ��
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
*	�� �� ��: PrintfLogo
*	����˵��: ��ӡ�������ƺ����̷�������, ���ϴ����ߺ󣬴�PC���ĳ����ն��������Թ۲���
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void PrintfLogo(void)
{
	
	printf("\r\n*************************************************************\r\n");
	//printf("\r\n*\r\n");
	
	//comSendChar(COM1, 0xaa);
	//comSendChar(COM1, 0xaa);
	
	/* ���CPU ID 
	{
		uint32_t CPU_Sn0, CPU_Sn1, CPU_Sn2;
		
		CPU_Sn0 = *(__IO uint32_t*)(0x1FFF7A10);
		CPU_Sn1 = *(__IO uint32_t*)(0x1FFF7A10 + 4);
		CPU_Sn2 = *(__IO uint32_t*)(0x1FFF7A10 + 8);

		printf("CPU : STM32F407IGT6, LQFP176, ��Ƶ: %dMHz\r\n", SystemCoreClock / 1000000);
		printf("UID = %08X %08X %08X\n\r", CPU_Sn2, CPU_Sn1, CPU_Sn0);
	}
		
	printf("*************************************************************\n\r");
	*/
}

/*
*********************************************************************************************************
*	�� �� ��: rev_from_pc
*	����˵��: ���յ���PC������������
*	��    ��: _byte ���յ���������
*	�� �� ֵ: ��
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
	
	if(0xF0 == _byte)//ȫ��ֹͣ
	{
		//ֹͣƵ�����
		
		//������
	}
	if(0xE0 == (_byte&0xF0))//ȫ������
	{
		//����Ƶ�����
		
		//������
	}
	
	if(0x3C == _byte)//�ж���Nimma������
	{
		//Ч��ǰ��λ���ݣ�ȷ�����ݰ�
		if( 0x01 == com_pc_buff[1] && 0xc3 == com_pc_buff[rev_pc_num-2])
		{
			for(int i = 0; i < rev_pc_num-1; i++)
			{
				temp_buff[i] = com_pc_buff[i+1];
			}
				
			//�ӵ�0�ֽڿ����ĸ�������
			if( 0x90 == com_pc_buff[0]) //Nimma900
			{
				//��������Nimma900 COM1
				comSendBuf(COM1, temp_buff, rev_pc_num-1);	
			}
		
			if( 0x60 == com_pc_buff[0])	//Nimma600
			{
				//��������Nimma600 COM6
				comSendBuf(COM6, temp_buff, rev_pc_num-1);	
			}

		}
		
		//�������յ�������
		clear_pc_com_buff();		
	}
		
	if(76 == _byte)//�ж��ǿ���ָ������� L
	{
		//Ч��ǰ1λ���ݣ�ȷ�����ݰ� C
		if(67 == com_pc_buff[0]) 
		{
			//��������ָ��������д���
			cmd_exec(com_pc_buff[1], com_pc_buff[2]);			
		}
		
		//�������յ�������
		clear_pc_com_buff();	
	}
	
}


/*
*********************************************************************************************************
*	�� �� ��: rev_from_nimma900
*	����˵��: ���յ���nimma900������������
*	��    ��: _byte ���յ���������
*	�� �� ֵ: ��
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
	
	if(0x3C == _byte)//�ж���Nimma������
	{
		//Ч��ǰ��λ���ݣ�ȷ�����ݰ�
		if( 0x01 == com_nimma900_buff[0] && 0xc3 == com_nimma900_buff[rev_nimma900_num-2])
		{
			//����β����ʶ
			com_nimma900_buff[rev_nimma900_num] = 0x90;
			comSendBuf(COM2, com_nimma900_buff, rev_nimma900_num+1);		
			//dbg@0521
			//comSendBuf(COM4, com_nimma900_buff, rev_nimma900_num);			
			
		}
		
		//�������յ�������
		clear_nimma900_com_buff();		
	}
}

/*
*********************************************************************************************************
*	�� �� ��: rev_from_nimma600
*	����˵��: ���յ���nimma600������������
*	��    ��: _byte ���յ���������
*	�� �� ֵ: ��
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
	
	if(0x3C == _byte)//�ж���Nimma������
	{
		//Ч��ǰ��λ���ݣ�ȷ�����ݰ�
		if( 0x01 == com_nimma600_buff[0] && 0xc3 == com_nimma600_buff[rev_nimma600_num-2])
		{
			//����β����ʶ
			com_nimma600_buff[rev_nimma600_num] = 0x60;
			comSendBuf(COM2, com_nimma600_buff, rev_nimma600_num+1);		
			//dbg@0521
			//comSendBuf(COM4, com_nimma900_buff, rev_nimma900_num);			
			
		}
		
		//�������յ�������
		clear_nimma600_com_buff();		
	}
}

/*
*********************************************************************************************************
*	�� �� ��: cmd_exec
*	����˵��: ����ָ��
*	��    ��: ��
*	�� �� ֵ: ��
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
		
		//����timer 1000 = 1ms; 100000 = 100ms; 0�������С�ж�����
		bsp_StartHardTimer(2, 0, lasers_timer_stick);
	}
	else
	{
		//ֹͣtimer 1000 = 1ms; 100000 = 100ms
		bsp_StopTimer(2);		
	}

}

void lasers_timer_stick(void)
{
	int j = 0;
	//ͬʱ����
	if( (1 == nimma900_cmd.cmd.bshot) && (1 == nimma600_cmd.cmd.bshot) && (nimma900_prd == nimma600_prd))
	{
		//����
		if(timer_count%nimma900_prd == 0)
		{
			//���Դ����������
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