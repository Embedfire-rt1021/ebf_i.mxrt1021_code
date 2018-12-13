/**
  ******************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2018-xx-xx
  * @brief   用V2.3.1版本库建的工程模板
  ******************************************************************
  * @attention
  *
  * 实验平台:野火  i.MXRT1052开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
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
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{
		double fps;
		uint16_t width, height;
		uint16_t * pic;
		uint16_t display_y_axis=0;
    /* 初始化内存保护单元 */
    BOARD_ConfigMPU();
    /* 初始化开发板引脚 */
    BOARD_InitPins();
    /* 初始化开发板时钟 */
    BOARD_BootClockRUN();
    /* 初始化调试串口 */
    BOARD_InitDebugConsole();
    /* 打印系统时钟 */
    PRINTF("\r\n");
    PRINTF("*****欢迎使用 野火i.MX RT1021 开发板*****\r\n");
    PRINTF("CPU:             %d Hz\r\n", CLOCK_GetFreq(kCLOCK_CpuClk));
    PRINTF("AHB:             %d Hz\r\n", CLOCK_GetFreq(kCLOCK_AhbClk));
    PRINTF("SEMC:            %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SemcClk));
    PRINTF("SYSPLL:          %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllClk));
    PRINTF("SYSPLLPFD0:      %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllPfd0Clk));
    PRINTF("SYSPLLPFD1:      %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllPfd1Clk));
    PRINTF("SYSPLLPFD2:      %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllPfd2Clk));
    PRINTF("SYSPLLPFD3:      %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllPfd3Clk));  
		PRINTF("SEMC 8080 LCD 例程\r\n");	
		
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

	//其中0、3、5、6 模式适合从左至右显示文字，
	//不推荐使用其它模式显示文字	其它模式显示文字会有镜像效果			
	//其中 6 模式为大部分液晶例程的默认显示方向  
  ILI9341_GramScan ( 3 );	
	//触摸屏初始化
	XPT2046_Init();
	//从FLASH里获取校正参数，若FLASH无参数，则使用模式3进行校正
	Calibrate_or_Get_TouchParaWithFlash(3,0);	
	//绘制触摸画板界面
	Palette_Init(LCD_SCAN_MODE);
	
		LCD_SetColors(CL_BLUE,CL_WHITE);
		ILI9341_DispString_EN_CH( 150,display_y_axis,(uint8_t*)"SDRAM测试...");	
		if(SEMC_SDRAMReadWriteTest() && SDRAM_FullChipTest())
		{
			LCD_SetColors(CL_BLUE,CL_WHITE);
			ILI9341_DispString_EN_CH( 150,display_y_axis,(uint8_t*)"SDRAM测试成功");	 
		}
		else
		{
			LCD_SetColors(CL_RED,CL_WHITE);
			ILI9341_DispString_EN_CH( 150,display_y_axis,(uint8_t*)"SDRAM测试失败 ");			
		}		
		/* 初始化时间戳并清零 */
		CPU_TS_TmrInit();
		while (1)
    {
			//触摸检测函数，本函数至少10ms调用一次
			XPT2046_TouchEvenHandler();
			//ILI9341_FillSolid(0, 0, width-1, height-1, GREEN);                      /* Fill color with CPU */			
//			FillPicDma(0, 0, width-1, height-1, pic);                               /* Fill picture with DMA */
//			fps++;
//			if(CPU_TS_TmrRd()> 1000000000)
//			{
//				PRINTF("帧率=%0.2f\r\n",fps/2);
//				fps =0;
//				ILI9341_FillSolid(0, 0, width-1, height-1, BLUE);
//				CPU_TS_Tmr_Delay_US(500*1000);
//				CPU_TS_TmrInit();
//			}
    }    

}
/****************************END OF FILE**********************/
