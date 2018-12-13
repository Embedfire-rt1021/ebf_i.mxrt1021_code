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
//#include "football.h"
#include "flexio_ov5640.h"

/*******************************************************************
 * Prototypes
 *******************************************************************/
extern bool newFrame;
extern pFrameBuffer_t pLCDFrameBuffer;
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
    BOARD_InitBootPins();
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
		OV5640_Init(NULL,NULL);
		/* Camera initialization. */
    FLEXIO_Ov7670Init();
#ifndef SEMC_SDRAM_USED   /* 160x120 */
		width = 240;//ILI9341_FRAME_H_PIXELS / 2;
		height = 240;//ILI9341_FRAME_V_PIXELS / 2;
		pic = (uint16_t *)gImage_bycle;
#else   /* 320x240 */
		width = ILI9341_FRAME_H_PIXELS;
		height = ILI9341_FRAME_V_PIXELS;
//		pic = (uint16_t *)gImage_football;
#endif		
		/* ��ʼ��ʱ��������� */
		CPU_TS_TmrInit();

		while (1)
    {
			if(newFrame)
			{
					/* Display the captured frame. */
					//ILI9341_FillPic(0, 0, OV7670_FRAME_WIDTH-1u, OV7670_FRAME_HEIGHT-1u, (uint16_t *)(*pLCDFrameBuffer));
					FillPicDma(0, 0, OV7670_FRAME_WIDTH-1u, OV7670_FRAME_HEIGHT-1u, (uint16_t *)(*pLCDFrameBuffer));	
					newFrame = false;
					fps++;
			}					
			if(CPU_TS_TmrRd()> 1000000000)
			{
				PRINTF("֡��=%0.2f\r\n",fps/2);
				fps =0;
				CPU_TS_TmrInit();
			}
    }    

}
/****************************END OF FILE**********************/
