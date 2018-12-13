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
#include "bsp_ili9341_lcd.h"
#include "./touch/bsp_xpt2046_lcd.h"
#include "football.h"
#include "./touch/palette.h"
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
		uint16_t display_y_axis=0;
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
#ifndef SEMC_SDRAM_USED   /* 160x120 */
		width = 240;//ILI9341_FRAME_H_PIXELS / 2;
		height = 240;//ILI9341_FRAME_V_PIXELS / 2;
		pic = (uint16_t *)gImage_bycle;
#else   /* 320x240 */
		width = ILI9341_FRAME_H_PIXELS;
		height = ILI9341_FRAME_V_PIXELS;
		pic = (uint16_t *)gImage_football;
#endif		

	//����0��3��5��6 ģʽ�ʺϴ���������ʾ���֣�
	//���Ƽ�ʹ������ģʽ��ʾ����	����ģʽ��ʾ���ֻ��о���Ч��			
	//���� 6 ģʽΪ�󲿷�Һ�����̵�Ĭ����ʾ����  
  ILI9341_GramScan ( 3 );	
	//��������ʼ��
	XPT2046_Init();
	//��FLASH���ȡУ����������FLASH�޲�������ʹ��ģʽ3����У��
	Calibrate_or_Get_TouchParaWithFlash(3,0);	
	//���ƴ����������
	Palette_Init(LCD_SCAN_MODE);
	
		LCD_SetColors(CL_BLUE,CL_WHITE);
		ILI9341_DispString_EN_CH( 150,display_y_axis,(uint8_t*)"SDRAM����...");	
		if(SEMC_SDRAMReadWriteTest() && SDRAM_FullChipTest())
		{
			LCD_SetColors(CL_BLUE,CL_WHITE);
			ILI9341_DispString_EN_CH( 150,display_y_axis,(uint8_t*)"SDRAM���Գɹ�");	 
		}
		else
		{
			LCD_SetColors(CL_RED,CL_WHITE);
			ILI9341_DispString_EN_CH( 150,display_y_axis,(uint8_t*)"SDRAM����ʧ�� ");			
		}		
		/* ��ʼ��ʱ��������� */
		CPU_TS_TmrInit();
		while (1)
    {
			//������⺯��������������10ms����һ��
			XPT2046_TouchEvenHandler();
			//ILI9341_FillSolid(0, 0, width-1, height-1, GREEN);                      /* Fill color with CPU */			
//			FillPicDma(0, 0, width-1, height-1, pic);                               /* Fill picture with DMA */
//			fps++;
//			if(CPU_TS_TmrRd()> 1000000000)
//			{
//				PRINTF("֡��=%0.2f\r\n",fps/2);
//				fps =0;
//				ILI9341_FillSolid(0, 0, width-1, height-1, BLUE);
//				CPU_TS_Tmr_Delay_US(500*1000);
//				CPU_TS_TmrInit();
//			}
    }    

}
/****************************END OF FILE**********************/
