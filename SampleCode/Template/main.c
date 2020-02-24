/*************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M480 MCU.
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define LED_R							(PH0)
#define LED_Y							(PH1)
#define LED_G							(PH2)

#define QSPI_MASTER_TX_DMA_CH 			(0)
#define QSPI_MASTER_RX_DMA_CH 		(1)
#define QSPI_MASTER_OPENED_CH   		((1 << QSPI_MASTER_TX_DMA_CH) | (1 << QSPI_MASTER_RX_DMA_CH))

#define QSPI_SLAVE_TX_DMA_CH  			(2)
#define QSPI_SLAVE_RX_DMA_CH  			(3)
#define QSPI_SLAVE_OPENED_CH   			((1 << QSPI_SLAVE_TX_DMA_CH) | (1 << QSPI_SLAVE_RX_DMA_CH))

#define QSPI_TARGET_FREQ				(100000ul)	//(48000000ul)

#define DATA_NUM						(16)

#define DATA_HEAD_MASTER				(0x30)
#define DATA_TAIL_MASTER				(0x36)

#define DATA_HEAD_SLAVE					(0x55)
#define DATA_TAIL_SLAVE					(0x66)

#define INDEX_HEAD						(0)
#define INDEX_TAIL						(DATA_NUM-1)

uint8_t g_au8MasterToSlaveTestPattern[DATA_NUM]={0};
uint8_t g_au8SlaveToMasterTestPattern[DATA_NUM]={0};
uint8_t g_au8MasterRxBuffer[DATA_NUM]={0};
uint8_t g_au8SlaveRxBuffer[DATA_NUM]={0};

enum
{
	QSPI_TX = 0,
	QSPI_RX = 1,		
};

void QSPI_Slave_PDMA_ClrBuffer(uint8_t TxRx)
{
    uint16_t i = 0;

	if (TxRx == QSPI_TX)
	{
	    for (i = 0; i < DATA_NUM; i++)
	    {
	        g_au8SlaveToMasterTestPattern[i] = 0xFF;
	    }
	}
	else
	{
	    for (i = 0; i < DATA_NUM; i++)
	    {
	        g_au8SlaveRxBuffer[i] = 0xFF;
	    }
	}
}

void QSPI_Slave_PDMA_PreInit(void)
{
	QSPI_Slave_PDMA_ClrBuffer(QSPI_TX);
	QSPI_Slave_PDMA_ClrBuffer(QSPI_RX);
	
    PDMA_Open(PDMA, QSPI_SLAVE_OPENED_CH);

	//RX
    PDMA_SetTransferCnt(PDMA,QSPI_SLAVE_RX_DMA_CH, PDMA_WIDTH_8, DATA_NUM);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA,QSPI_SLAVE_RX_DMA_CH, (uint32_t)&QSPI0->RX, PDMA_SAR_FIX, (uint32_t)g_au8SlaveRxBuffer, PDMA_DAR_INC);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA,QSPI_SLAVE_RX_DMA_CH, PDMA_QSPI0_RX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA,QSPI_SLAVE_RX_DMA_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    PDMA->DSCT[QSPI_SLAVE_RX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;

	//TX
    PDMA_SetTransferCnt(PDMA,QSPI_SLAVE_TX_DMA_CH, PDMA_WIDTH_8, DATA_NUM);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA,QSPI_SLAVE_TX_DMA_CH, (uint32_t)g_au8SlaveToMasterTestPattern, PDMA_SAR_INC, (uint32_t)&QSPI0->TX, PDMA_DAR_FIX);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA,QSPI_SLAVE_TX_DMA_CH, PDMA_QSPI0_TX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA,QSPI_SLAVE_TX_DMA_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    PDMA->DSCT[QSPI_SLAVE_TX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;

    QSPI_TRIGGER_RX_PDMA(QSPI0);
    QSPI_TRIGGER_TX_PDMA(QSPI0);

    PDMA_EnableInt(PDMA, QSPI_SLAVE_TX_DMA_CH, PDMA_INT_TRANS_DONE);
    PDMA_EnableInt(PDMA, QSPI_SLAVE_RX_DMA_CH, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA_IRQn);	
	
}

void QSPI_Slave_PDMA_Enable(uint8_t TxRx)
{
    uint16_t i = 0;

	if (TxRx == QSPI_TX)
	{		
		QSPI_Slave_PDMA_ClrBuffer(QSPI_TX);		

		//prepare slave TX data
		if ((g_au8SlaveRxBuffer[INDEX_HEAD] == DATA_HEAD_MASTER)&& 
			(g_au8SlaveRxBuffer[INDEX_TAIL] == DATA_TAIL_MASTER))
		{

			g_au8SlaveToMasterTestPattern[INDEX_HEAD] = DATA_HEAD_SLAVE;
			g_au8SlaveToMasterTestPattern[INDEX_TAIL] = DATA_TAIL_SLAVE;

		    for (i = (INDEX_HEAD+1); i < INDEX_TAIL; i++)
		    {
		        g_au8SlaveToMasterTestPattern[INDEX_TAIL-i] = g_au8SlaveRxBuffer[i];
		    }
			
//		    for (i = (INDEX_HEAD+1); i < INDEX_TAIL; i++)
//		    {
//		        g_au8SlaveToMasterTestPattern[i] = i;
//		    }
		}

		#if 1	//TX debug
		printf("\r\ng_au8SlaveToMasterTestPattern : \r\n");
		for (i = 0 ; i < DATA_NUM ; i++)
		{
			printf("0x%2X," , g_au8SlaveToMasterTestPattern[i]);
            if ((i+1)%8 ==0)
            {
                printf("\r\n");
            }			
		}
		#endif			
	
		//TX
	    PDMA_SetTransferCnt(PDMA,QSPI_SLAVE_TX_DMA_CH, PDMA_WIDTH_8, DATA_NUM);
   	 	PDMA_SetTransferAddr(PDMA,QSPI_SLAVE_TX_DMA_CH, (uint32_t)g_au8SlaveToMasterTestPattern, PDMA_SAR_INC, (uint32_t)&QSPI0->TX, PDMA_DAR_FIX);		
	    /* Set request source; set basic mode. */
	    PDMA_SetTransferMode(PDMA,QSPI_SLAVE_TX_DMA_CH, PDMA_QSPI0_TX, FALSE, 0);
	    QSPI_TRIGGER_TX_PDMA(QSPI0);		

	}
	else
	{		
		//RX
	    PDMA_SetTransferCnt(PDMA,QSPI_SLAVE_RX_DMA_CH, PDMA_WIDTH_8, DATA_NUM);
	    /* Set request source; set basic mode. */
	    PDMA_SetTransferMode(PDMA,QSPI_SLAVE_RX_DMA_CH, PDMA_QSPI0_RX, FALSE, 0);
	    QSPI_TRIGGER_RX_PDMA(QSPI0);	

		#if 1	//RX debug
		printf("\r\ng_au8SlaveRxBuffer : \r\n");
		for (i = 0 ; i < DATA_NUM ; i++)
		{
			printf("0x%2X," , g_au8SlaveRxBuffer[i]);
            if ((i+1)%8 ==0)
            {
                printf("\r\n");
            }			
		}
		#endif

		QSPI_Slave_PDMA_ClrBuffer(QSPI_RX);	
	}
}

void QSPI_Slave_Init(void)
{
    QSPI_Open(QSPI0, QSPI_SLAVE, QSPI_MODE_0, 8, (uint32_t)NULL);
}


void QSPI_Master_PDMA_ClrBuffer(uint8_t TxRx)
{
    uint16_t i = 0;

	if (TxRx == QSPI_TX)
	{
	    for (i = 0; i < DATA_NUM; i++)
	    {
	        g_au8MasterToSlaveTestPattern[i] = 0xFF;
	    }
	}
	else
	{
	    for (i = 0; i < DATA_NUM; i++)
	    {
	        g_au8MasterRxBuffer[i] = 0xFF;
	    }
	}
}

void QSPI_Master_PDMA_PreInit(void)
{
    uint16_t i = 0;
	
	//prepare data
    for (i=0; i < DATA_NUM; i++)
    {
        g_au8MasterToSlaveTestPattern[i] = i;
        g_au8MasterRxBuffer[i] = 0xFF;
    }

    PDMA_Open(PDMA, QSPI_MASTER_OPENED_CH);

	//TX
    PDMA_SetTransferCnt(PDMA,QSPI_MASTER_TX_DMA_CH, PDMA_WIDTH_8, DATA_NUM);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA,QSPI_MASTER_TX_DMA_CH, (uint32_t)g_au8MasterToSlaveTestPattern, PDMA_SAR_INC, (uint32_t)&QSPI0->TX, PDMA_DAR_FIX);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA,QSPI_MASTER_TX_DMA_CH, PDMA_QSPI0_TX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA,QSPI_MASTER_TX_DMA_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    PDMA->DSCT[QSPI_MASTER_TX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;

	//RX	
    PDMA_SetTransferCnt(PDMA,QSPI_MASTER_RX_DMA_CH, PDMA_WIDTH_8, DATA_NUM);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA,QSPI_MASTER_RX_DMA_CH, (uint32_t)&QSPI0->RX, PDMA_SAR_FIX, (uint32_t)g_au8MasterRxBuffer, PDMA_DAR_INC);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA,QSPI_MASTER_RX_DMA_CH, PDMA_QSPI0_RX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA,QSPI_MASTER_RX_DMA_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    PDMA->DSCT[QSPI_MASTER_RX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;

    QSPI_TRIGGER_RX_PDMA(QSPI0);
    QSPI_TRIGGER_TX_PDMA(QSPI0);

    PDMA_EnableInt(PDMA, QSPI_MASTER_TX_DMA_CH, PDMA_INT_TRANS_DONE);
    PDMA_EnableInt(PDMA, QSPI_MASTER_RX_DMA_CH, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA_IRQn);
	
}

void QSPI_Master_PDMA_Enable(uint8_t TxRx)
{
    uint16_t i = 0;
    static uint8_t k = 0x10;
	
	if (TxRx == QSPI_TX)
	{
		QSPI_Master_PDMA_ClrBuffer(QSPI_TX);
	
		//prepare master TX data
		g_au8MasterToSlaveTestPattern[INDEX_HEAD] = DATA_HEAD_MASTER;
		g_au8MasterToSlaveTestPattern[INDEX_TAIL] = DATA_TAIL_MASTER;

	    for (i = (INDEX_HEAD+1); i < INDEX_TAIL ; i++)
	    {
	        g_au8MasterToSlaveTestPattern[i] = (i + k);
	    }

		k += 0x10;
		if (k >= 0xFF)
		{
			k = 0x10;
		}

		#if 1	//TX debug
		printf("\r\ng_au8MasterToSlaveTestPattern : \r\n");
		for (i = 0 ; i < DATA_NUM ; i++)
		{
			printf("0x%2X," , g_au8MasterToSlaveTestPattern[i]);
            if ((i+1)%8 ==0)
            {
                printf("\r\n");
            }			
		}
		#endif
	
		//TX
	    PDMA_SetTransferCnt(PDMA,QSPI_MASTER_TX_DMA_CH, PDMA_WIDTH_8, DATA_NUM);
		PDMA_SetTransferAddr(PDMA,QSPI_MASTER_TX_DMA_CH, (uint32_t)g_au8MasterToSlaveTestPattern, PDMA_SAR_INC, (uint32_t)&QSPI0->TX, PDMA_DAR_FIX);		
	    /* Set request source; set basic mode. */
	    PDMA_SetTransferMode(PDMA,QSPI_MASTER_TX_DMA_CH, PDMA_QSPI0_TX, FALSE, 0);
	    QSPI_TRIGGER_TX_PDMA(QSPI0);	
	}
	else
	{
		//RX	
		PDMA_SetTransferCnt(PDMA,QSPI_MASTER_RX_DMA_CH, PDMA_WIDTH_8, DATA_NUM);
		/* Set request source; set basic mode. */
		PDMA_SetTransferMode(PDMA,QSPI_MASTER_RX_DMA_CH, PDMA_QSPI0_RX, FALSE, 0);
		QSPI_TRIGGER_RX_PDMA(QSPI0);

		#if 1	//RX debug
		printf("\r\ng_au8MasterRxBuffer : \r\n");
		for (i = 0 ; i < DATA_NUM ; i++)
		{
			printf("0x%2X," , g_au8MasterRxBuffer[i]);
            if ((i+1)%8 ==0)
            {
                printf("\r\n");
            }			
		}
		#endif

		QSPI_Master_PDMA_ClrBuffer(QSPI_RX);			
	}
}

/*
	PA0 : QSPI0_MOSI0 , PA1 : QSPI0_MISO0 , 
	PA2 : QSPI0_CLK , 
	PA3 : QSPI0_SS , 
	PA4 : QSPI0_MOSI1 , PA5 : QSPI0_MISO1 , 

*/
void QSPI_Master_Init(void)
{
    QSPI_Open(QSPI0, QSPI_MASTER, QSPI_MODE_0, 8, QSPI_TARGET_FREQ);

    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    QSPI_EnableAutoSS(QSPI0, QSPI_SS, QSPI_SS_ACTIVE_LOW);
}


void PDMA_IRQHandler(void)
{
	#if defined (ENABLE_QSPI_MASTER)	
    uint32_t status = PDMA_GET_INT_STATUS(PDMA);
	
    if (status & PDMA_INTSTS_ABTIF_Msk)   /* abort */
    {
		#if 1
        PDMA_CLR_ABORT_FLAG(PDMA, PDMA_GET_ABORT_STS(PDMA));
		#else
        if (PDMA_GET_ABORT_STS(PDMA) & (1 << QSPI_MASTER_TX_DMA_CH))
        {

        }
        PDMA_CLR_ABORT_FLAG(PDMA, (1 << QSPI_MASTER_TX_DMA_CH));

        if (PDMA_GET_ABORT_STS(PDMA) & (1 << QSPI_MASTER_RX_DMA_CH))
        {

        }
        PDMA_CLR_ABORT_FLAG(PDMA, (1 << QSPI_MASTER_RX_DMA_CH));
		#endif
    }
    else if (status & PDMA_INTSTS_TDIF_Msk)     /* done */
    {
        if((PDMA_GET_TD_STS(PDMA) & QSPI_MASTER_OPENED_CH) == QSPI_MASTER_OPENED_CH)
        {
            /* Clear PDMA transfer done interrupt flag */
            PDMA_CLR_TD_FLAG(PDMA, QSPI_MASTER_OPENED_CH);

			//insert process
			QSPI_DISABLE_TX_PDMA(QSPI0);
			QSPI_DISABLE_RX_PDMA(QSPI0);
			LED_Y ^= 1;
			QSPI_Master_PDMA_Enable(QSPI_TX);
			QSPI_Master_PDMA_Enable(QSPI_RX);
        }        		
    }
    else if (status & (PDMA_INTSTS_REQTOF0_Msk | PDMA_INTSTS_REQTOF1_Msk))     /* Check the DMA time-out interrupt flag */
    {
		LED_G ^= 1;
        PDMA_CLR_TMOUT_FLAG(PDMA,QSPI_MASTER_TX_DMA_CH);
        PDMA_CLR_TMOUT_FLAG(PDMA,QSPI_MASTER_RX_DMA_CH);
    }
    else
    {

    }

	#elif defined (ENABLE_QSPI_SLAVE)
    uint32_t status = PDMA_GET_INT_STATUS(PDMA);
	
    if (status & PDMA_INTSTS_ABTIF_Msk)   /* abort */
    {
		#if 1
        PDMA_CLR_ABORT_FLAG(PDMA, PDMA_GET_ABORT_STS(PDMA));
		#else
        if (PDMA_GET_ABORT_STS(PDMA) & (1 << QSPI_SLAVE_TX_DMA_CH))
        {

        }
        PDMA_CLR_ABORT_FLAG(PDMA, (1 << QSPI_SLAVE_TX_DMA_CH));

        if (PDMA_GET_ABORT_STS(PDMA) & (1 << QSPI_SLAVE_RX_DMA_CH))
        {

        }
        PDMA_CLR_ABORT_FLAG(PDMA, (1 << QSPI_SLAVE_RX_DMA_CH));
		#endif
    }
    else if (status & PDMA_INTSTS_TDIF_Msk)     /* done */
    {
        if((PDMA_GET_TD_STS(PDMA) & QSPI_SLAVE_OPENED_CH) == QSPI_SLAVE_OPENED_CH)
        {
            /* Clear PDMA transfer done interrupt flag */
            PDMA_CLR_TD_FLAG(PDMA, QSPI_SLAVE_OPENED_CH);

			//insert process
			QSPI_DISABLE_TX_PDMA(QSPI0);
			QSPI_DISABLE_RX_PDMA(QSPI0);			
			LED_Y ^= 1;
			QSPI_Slave_PDMA_Enable(QSPI_TX);
			QSPI_Slave_PDMA_Enable(QSPI_RX);			
        }        		
    }
    else if (status & (PDMA_INTSTS_REQTOF0_Msk | PDMA_INTSTS_REQTOF1_Msk))     /* Check the DMA time-out interrupt flag */
    {
		LED_G ^= 1;
        PDMA_CLR_TMOUT_FLAG(PDMA,QSPI_SLAVE_TX_DMA_CH);
        PDMA_CLR_TMOUT_FLAG(PDMA,QSPI_SLAVE_RX_DMA_CH);
    }
    else
    {

    }

	#endif
	
}

void TMR1_IRQHandler(void)
{
	static uint16_t CNT = 0;	
	static uint32_t log = 0;	
	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
	
		if (CNT++ > 1000)
		{		
			CNT = 0;
			printf("%s : %2d\r\n" , __FUNCTION__ , log++);
			LED_R ^= 1;
		}
    }
}

void TIMER1_HW_Init(void)
{
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);
}

void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void TIMER0_HW_Init(void)
{
	CLK_EnableModuleClock(TMR0_MODULE);
	CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
}

void TIMER0_Polling(uint32_t u32Usec)
{
	TIMER_Delay(TIMER0, u32Usec);
}

void LED_Init(void)
{
	GPIO_SetMode(PH,BIT0,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT1,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT2,GPIO_MODE_OUTPUT);
	
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_192MHZ);
    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    CLK_SetModuleClock(QSPI0_MODULE, CLK_CLKSEL2_QSPI0SEL_PCLK0, MODULE_NoMsk);
    CLK_EnableModuleClock(QSPI0_MODULE);

    CLK_EnableModuleClock(PDMA_MODULE);

//	TIMER0_HW_Init();
	TIMER1_HW_Init();
	
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Setup QSPI0 multi-function pins */
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA0MFP_QSPI0_MOSI0 | SYS_GPA_MFPL_PA1MFP_QSPI0_MISO0 | SYS_GPA_MFPL_PA2MFP_QSPI0_CLK | SYS_GPA_MFPL_PA3MFP_QSPI0_SS |
                     SYS_GPA_MFPL_PA4MFP_QSPI0_MOSI1 | SYS_GPA_MFPL_PA5MFP_QSPI0_MISO1;

    /* Enable QSPI0 clock pin (PA2) schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;

    /* Enable QSPI0 I/O high slew rate */
    GPIO_SetSlewCtl(PA, 0x3F, GPIO_SLEWCTL_HIGH);
	
    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M480 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
	
    SYS_Init();
    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());

	LED_Init();
	TIMER1_Init();

	#if defined (ENABLE_QSPI_MASTER)
	
	QSPI_Master_Init();
	QSPI_Master_PDMA_PreInit();
	
	#elif defined (ENABLE_QSPI_SLAVE)
	
	QSPI_Slave_Init();
	QSPI_Slave_PDMA_PreInit();

	#endif

    /* Got no where to go, just loop forever */
    while(1)
    {
//		TIMER0_Polling(100);
		

    }

}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
