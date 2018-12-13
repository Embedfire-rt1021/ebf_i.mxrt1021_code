/**
  ******************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2018-xx-xx
  * @brief   ��V2.3.1�汾�⽨�Ĺ���ģ��
  ******************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ��  i.MXRT1052������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************
  */
#include "fsl_debug_console.h"

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "ili9341.h"
#include "football.h"

/*******************************************************************
 * Prototypes
 *******************************************************************/


/*******************************************************************
 * Code
 *******************************************************************/

/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void)
{
		double fps;
		uint16_t width, height;
		uint16_t * pic;
	
    /* ��ʼ���ڴ汣����Ԫ */
    BOARD_ConfigMPU();
    /* ��ʼ������������ */
    BOARD_InitPins();
    /* ��ʼ��������ʱ�� */
    BOARD_BootClockRUN();
    /* ��ʼ�����Դ��� */
    BOARD_InitDebugConsole();
    /* ��ӡϵͳʱ�� */
    PRINTF("\r\n");
    PRINTF("*****��ӭʹ�� Ұ��i.MX RT1021 ������*****\r\n");
    PRINTF("CPU:             %d Hz\r\n", CLOCK_GetFreq(kCLOCK_CpuClk));
    PRINTF("AHB:             %d Hz\r\n", CLOCK_GetFreq(kCLOCK_AhbClk));
    PRINTF("SEMC:            %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SemcClk));
    PRINTF("SYSPLL:          %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllClk));
    PRINTF("SYSPLLPFD0:      %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllPfd0Clk));
    PRINTF("SYSPLLPFD1:      %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllPfd1Clk));
    PRINTF("SYSPLLPFD2:      %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllPfd2Clk));
    PRINTF("SYSPLLPFD3:      %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllPfd3Clk));  
		PRINTF("SEMC 8080 LCD ����\r\n");	
		
		ConfigSemcDbi();
		
		ILI9341_Init();

    ConfigDma();

		width = ILI9341_FRAME_H_PIXELS;
		height = ILI9341_FRAME_V_PIXELS;
		pic = (uint16_t *)gImage_football;
		
		/* ��ʼ��ʱ��������� */
		CPU_TS_TmrInit();
		while (1)
    {
			//ILI9341_FillSolid(0, 0, width-1, height-1, GREEN);                      /* Fill color with CPU */			
			FillPicDma(0, 0, width-1, height-1, pic);                               /* Fill picture with DMA */
			fps++;
			if(CPU_TS_TmrRd()> 1000000000)
			{
				PRINTF("֡��=%0.2f\r\n",fps/2);
				fps =0;
				ILI9341_FillSolid(0, 0, width-1, height-1, BLUE);
				CPU_TS_Tmr_Delay_US(500*1000);
				CPU_TS_TmrInit();
			}
    }    

}
/****************************END OF FILE**********************/
