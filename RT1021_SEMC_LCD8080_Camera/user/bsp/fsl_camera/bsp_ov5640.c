/**
  ******************************************************************************
  * @file    bsp_sdram.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   OV5640摄像头驱动
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火  STM32 F429 开发板  
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "bsp_ov5640.h"
#include "ov5640_AF.h"
#include "board.h"

void BOARD_PullCameraResetPin(bool pullUp)
{
    if (pullUp)
    {
        GPIO_PinWrite(BOARD_CAMERA_RST_GPIO_BASE, BOARD_CAMERA_RST_PIN_INDEX, 1);
    }
    else
    {
        GPIO_PinWrite(BOARD_CAMERA_RST_GPIO_BASE, BOARD_CAMERA_RST_PIN_INDEX,0);
    }
}

void BOARD_PullCameraPowerDownPin(bool pullUp)
{
    if (pullUp)
    {
        GPIO_PinWrite(BOARD_CAMERA_PDWN_GPIO_BASE, BOARD_CAMERA_PDWN_PIN_INDEX, 1);
    }
    else
    {
				GPIO_PinWrite(BOARD_CAMERA_PDWN_GPIO_BASE, BOARD_CAMERA_PDWN_PIN_INDEX, 0);
    }
}

static ov5640_resource_t ov5640Resource = {
    .sccbI2C = OV5640_I2C,
    .pullResetPin = BOARD_PullCameraResetPin,
    .pullPowerDownPin = BOARD_PullCameraPowerDownPin,
    .inputClockFreq_Hz = 24000000,
};

camera_device_handle_t cameraDevice = {
    .resource = &ov5640Resource, .ops = &ov5640_ops,
};

void BOARD_InitCameraResource(void)
{
    lpi2c_master_config_t masterConfig;
    uint32_t sourceClock;

    LPI2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Hz = 100000;
    masterConfig.debugEnable = true;
    masterConfig.ignoreAck = true;

    /*Clock setting for LPI2C*/
    /*
     * LPI2C clock source:
     *  0: pll3_60m
     *  1: OSC clock
     */
    CLOCK_SetMux(kCLOCK_Lpi2cMux, 1);
    /*
     * LPI2C divider.
     *  0b000000: Divide by 1
     *  0b111111: Divide by 2^6
     */
    CLOCK_SetDiv(kCLOCK_Lpi2cDiv, 0);

    /* LPI2C clock is OSC clock. */
    sourceClock = CLOCK_GetOscFreq();

    LPI2C_MasterInit(OV5640_I2C, &masterConfig, sourceClock);


    /* Set the pins for CSI reset and power down. */
    gpio_pin_config_t pinConfig = {
        kGPIO_DigitalOutput, 1,
    };

    GPIO_PinInit(GPIO1, 3, &pinConfig);
		GPIO_PinInit(GPIO1, 24, &pinConfig);
}
/**
  * @brief  配置OV5640
  * @param  None
  * @retval None
  */
void Camera_Init(void) 
{
	BOARD_InitCameraResource();
}
