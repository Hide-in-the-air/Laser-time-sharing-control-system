/*
*********************************************************************************************************
*
*	ģ������ : �����ж�+FIFO����ģ��
*	�ļ����� : bsp_uart_fifo.h
*	˵    �� : ͷ�ļ�
*
*	Copyright (C), 2015-2020, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#ifndef _BSP_USART_FIFO_H_
#define _BSP_USART_FIFO_H_


/*
	�����Ҫ���Ĵ��ڶ�Ӧ�Ĺܽţ��������޸� bsp_uart_fifo.c�ļ��е� static void InitHardUart(void)����
*/

/* ����ʹ�ܵĴ���, 0 ��ʾ��ʹ�ܣ������Ӵ����С���� 1��ʾʹ�� */
/*
	������STM32-V5 ���ڷ��䣺
	������1�� RS232 оƬ��1·��
		PA10/USART1_RX
		PA9/USART1_TX

	������2�� PA2 �ܽ�������̫���� RX�ܽ����ڽ���GPS�ź�
		PA2/USART2_TX/ETH_MDIO (������̫�����������ڷ�����)
		PA3/USART2_RX	;��GPSģ�����

	������3�� RS485 ͨ�� - TTL ���� �� ����
		PB10/USART3_TX
		PB11/USART3_RX
		PB2-BOOT1/RS485_TXEN

	������4�� --- ���������á�����SD��
	������5�� --- ���������á�����SD��
		PC12/UART5_TX
		PD2/UART5_RX

	������6��--- GPRSģ�� ��Ӳ�����أ�
		PG14/USART6_TX
		PC7/USART6_RX
		PG8/USART6_RTS
		PG15/USART6_CTS
*/

#define	UART1_FIFO_EN	1
#define	UART2_FIFO_EN	1
#define	UART3_FIFO_EN	0
//dbg@0521
#define	UART4_FIFO_EN	0
#define	UART5_FIFO_EN	0
#define	UART6_FIFO_EN	1

/* PB2 ����RS485оƬ�ķ���ʹ�� */
#define RS485_TXEN_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define RS485_TXEN_GPIO_PORT              GPIOB
#define RS485_TXEN_PIN                    GPIO_PIN_2

#define RS485_RX_EN()	RS485_TXEN_GPIO_PORT->BSRR = (uint32_t)RS485_TXEN_PIN << 16U
#define RS485_TX_EN()	RS485_TXEN_GPIO_PORT->BSRR = RS485_TXEN_PIN

/* ����˿ں� */
typedef enum
{
	COM1 = 0,	/* USART1 */	//PA9 + PA10 Nimma 900
	COM2 = 1,	/* USART2 */	//PA2 + PA3 PC
	COM3 = 2,	/* USART3 */
	COM4 = 3,	/* UART4 */
	COM5 = 4,	/* UART5 */
	COM6 = 5,	/* USART6 */	//PC6 + PC7 Nimma 600
}COM_PORT_E;

/* ���崮�ڲ����ʺ�FIFO��������С����Ϊ���ͻ������ͽ��ջ�����, ֧��ȫ˫�� */
#if UART1_FIFO_EN == 1   	//PA9 + PA10 Nimma900
	#define UART1_BAUD			19200
	#define UART1_TX_BUF_SIZE	1*64
	#define UART1_RX_BUF_SIZE	1*64
#endif

#if UART2_FIFO_EN == 1		//PA2 + PA3 PC
	#define UART2_BAUD			9600
	#define UART2_TX_BUF_SIZE	1*64
	#define UART2_RX_BUF_SIZE	1*64
#endif

#if UART3_FIFO_EN == 1
	#define UART3_BAUD			9600
	#define UART3_TX_BUF_SIZE	1*64
	#define UART3_RX_BUF_SIZE	1*64
#endif

#if UART4_FIFO_EN == 1
	#define UART4_BAUD			9600
	#define UART4_TX_BUF_SIZE	1*1024
	#define UART4_RX_BUF_SIZE	1*1024
#endif

#if UART5_FIFO_EN == 1
	#define UART5_BAUD			9600
	#define UART5_TX_BUF_SIZE	1*1024
	#define UART5_RX_BUF_SIZE	1*1024
#endif

#if UART6_FIFO_EN == 1		//PC6 + PC7 Nimma600
	#define UART6_BAUD			9600
	#define UART6_TX_BUF_SIZE	1*64
	#define UART6_RX_BUF_SIZE	1*64
#endif


/* �����豸�ṹ�� */
typedef struct
{
	USART_TypeDef *uart;		/* STM32�ڲ������豸ָ�� */
	uint8_t *pTxBuf;			/* ���ͻ����� */
	uint8_t *pRxBuf;			/* ���ջ����� */
	uint16_t usTxBufSize;		/* ���ͻ�������С */
	uint16_t usRxBufSize;		/* ���ջ�������С */
	__IO uint16_t usTxWrite;	/* ���ͻ�����дָ�� */
	__IO uint16_t usTxRead;		/* ���ͻ�������ָ�� */
	__IO uint16_t usTxCount;	/* �ȴ����͵����ݸ��� */

	__IO uint16_t usRxWrite;	/* ���ջ�����дָ�� */
	__IO uint16_t usRxRead;		/* ���ջ�������ָ�� */
	__IO uint16_t usRxCount;	/* ��δ��ȡ�������ݸ��� */

	void (*SendBefor)(void); 	/* ��ʼ����֮ǰ�Ļص�����ָ�루��Ҫ����RS485�л�������ģʽ�� */
	void (*SendOver)(void); 	/* ������ϵĻص�����ָ�루��Ҫ����RS485������ģʽ�л�Ϊ����ģʽ�� */
	void (*ReciveNew)(uint8_t _byte);	/* �����յ����ݵĻص�����ָ�� */
	uint8_t Sending;			/* ���ڷ����� */
}UART_T;


/* ����ÿ�����ڽṹ����� */
#if UART1_FIFO_EN == 1
	//static UART_T g_tUart1;
	static UART_T g_tUart1;
	static uint8_t g_TxBuf1[UART1_TX_BUF_SIZE];		/* ���ͻ����� */
	static uint8_t g_RxBuf1[UART1_RX_BUF_SIZE];		/* ���ջ����� */
#endif

#if UART2_FIFO_EN == 1
	static UART_T g_tUart2;
	static uint8_t g_TxBuf2[UART2_TX_BUF_SIZE];		/* ���ͻ����� */
	static uint8_t g_RxBuf2[UART2_RX_BUF_SIZE];		/* ���ջ����� */
#endif

#if UART3_FIFO_EN == 1
	static UART_T g_tUart3;
	static uint8_t g_TxBuf3[UART3_TX_BUF_SIZE];		/* ���ͻ����� */
	static uint8_t g_RxBuf3[UART3_RX_BUF_SIZE];		/* ���ջ����� */
#endif

#if UART4_FIFO_EN == 1
	static UART_T g_tUart4;
	static uint8_t g_TxBuf4[UART4_TX_BUF_SIZE];		/* ���ͻ����� */
	static uint8_t g_RxBuf4[UART4_RX_BUF_SIZE];		/* ���ջ����� */
#endif

#if UART5_FIFO_EN == 1
	static UART_T g_tUart5;
	static uint8_t g_TxBuf5[UART5_TX_BUF_SIZE];		/* ���ͻ����� */
	static uint8_t g_RxBuf5[UART5_RX_BUF_SIZE];		/* ���ջ����� */
#endif

#if UART6_FIFO_EN == 1
	static UART_T g_tUart6;
	static uint8_t g_TxBuf6[UART6_TX_BUF_SIZE];		/* ���ͻ����� */
	static uint8_t g_RxBuf6[UART6_RX_BUF_SIZE];		/* ���ջ����� */
#endif


void bsp_InitUart(void);
void comSendBuf(COM_PORT_E _ucPort, uint8_t *_ucaBuf, uint16_t _usLen);
void comSendChar(COM_PORT_E _ucPort, uint8_t _ucByte);
uint8_t comGetChar(COM_PORT_E _ucPort, uint8_t *_pByte);
void comClearTxFifo(COM_PORT_E _ucPort);
void comClearRxFifo(COM_PORT_E _ucPort);
void comSetBaud(COM_PORT_E _ucPort, uint32_t _BaudRate);

void USART_SetBaudRate(USART_TypeDef* USARTx, uint32_t BaudRate);
void bsp_SetUartParam(USART_TypeDef *Instance,  uint32_t BaudRate, uint32_t Parity, uint32_t Mode);

void RS485_SendBuf(uint8_t *_ucaBuf, uint16_t _usLen);
void RS485_SendStr(char *_pBuf);
void RS485_SetBaud(uint32_t _baud);
uint8_t UartTxEmpty(COM_PORT_E _ucPort);

#endif

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/