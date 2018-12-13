/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "board.h"
#include "pin_mux.h"
#include "fsl_flexio_camera.h"

#include "ov5640_driver.h"
#include "fsl_sccb_master_driver.h"
#include "flexio_ov5640.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define FXIO_SHFT_COUNT         4u          /* 4 shifters */
#define DMA_TRSF_SIZE           8u          /* 8 bytes */
#define DMA_MINOR_LOOP_SIZE     16u         /* 16 bytes */
#define DMA_MAJOR_LOOP_SIZE     (OV7670_FRAME_BYTES / DMA_MINOR_LOOP_SIZE)

/*******************************************************************************
 * Variables
 ******************************************************************************/
__attribute__((section("NonCacheable"))) static FrameBuffer_t g_FlexioCameraFrameBuffer[OV7670_FRAME_BUFFER_CNT];
__attribute__((section("NonCacheable"))) static pFrameBuffer_t pFlexioCameraFrameBuffer = g_FlexioCameraFrameBuffer;
__attribute__((section("NonCacheable"))) pFrameBuffer_t pLCDFrameBuffer;

/* OV7670 configuration structures. */
static ov7670_handler_t s_Ov7670CameraHandler = {
    .i2cBase = BOARD_CAMERA_I2C_INST,
    .i2cDeviceAddr = OV7670_I2C_ADDR
};

static ov7670_advanced_config_t s_Ov7670CameraAdvancedConfig =
{
    .filter = (ov7670_filter_config_t *)&OV7670_FILTER_DISABLED,
    .nightMode = (ov7670_night_mode_config_t *)&OV7670_NIGHT_MODE_AUTO_FR_DIVBY2,
    .whiteBalance = (ov7670_white_balance_config_t *)&OV7670_WHITE_BALANCE_DEFAULT,
    .lightMode = (ov7670_light_mode_config_t *)&OV7670_LIGHT_MODE_AUTO,
    .colorSaturation = (ov7670_color_saturation_config_t *)&OV7670_COLOR_SATURATION_DEFAULT,
    .specialEffect = (ov7670_special_effect_config_t *)&OV7670_SPECIAL_EFFECT_DISABLED,
    .gammaCurveSlope = (ov7670_gamma_curve_slope_config_t *)&OV7670_GAMMA_CURVE_SLOPE_DEFAULT,
};

static ov7670_config_t s_Ov7670CameraConfig =
{
    .outputFormat = (ov7670_output_format_config_t *)&OV7670_FORMAT_RGB565,
    .resolution =
    {
        .width = OV7670_FRAME_WIDTH,
        .height = OV7670_FRAME_HEIGHT,
    },
    .frameRate = (ov7670_frame_rate_config_t *)&OV7670_15FPS_24MHZ_XCLK,
    .contrast = 0x30,
    .brightness = 0x80,
    .advancedConfig = (ov7670_advanced_config_t *)&s_Ov7670CameraAdvancedConfig,
};

static FLEXIO_CAMERA_Type s_FlexioCameraDevice = {
    .flexioBase = BOARD_CAMERA_FLEXIO_INST,
    .datPinStartIdx = BOARD_CAMERA_FLEXIO_DATA_PIN_START_INDEX,
    .pclkPinIdx = BOARD_CAMERA_FLEXIO_PCLK_PIN_INDEX,
    .hrefPinIdx = BOARD_CAMERA_FLEXIO_HREF_PIN_INDEX,
    .shifterStartIdx = 0U,
    .shifterCount = FXIO_SHFT_COUNT,
    .timerIdx = 0U,
};

static flexio_camera_config_t s_FlexioCameraConfig;

/*******************************************************************************
 * Code
 ******************************************************************************/

static void configFlexIO(void)
{
    /* Configure FlexIO. */
    FLEXIO_Reset(BOARD_CAMERA_FLEXIO_INST);
    /*Init the flexio to the camera mode */
    FLEXIO_CAMERA_GetDefaultConfig(&s_FlexioCameraConfig);
    FLEXIO_CAMERA_Init(&s_FlexioCameraDevice, &s_FlexioCameraConfig);
    /* Clear all the flag. */
    FLEXIO_CAMERA_ClearStatusFlags(&s_FlexioCameraDevice, kFLEXIO_CAMERA_RxDataRegFullFlag | kFLEXIO_CAMERA_RxErrorFlag);
    /* Enable FlexIO. */
    FLEXIO_CAMERA_Enable(&s_FlexioCameraDevice, true);
}

static void configCamera(void)
{
    ov7670_status_t ov7670Status;

    BOARD_Camera_RstPinInit();
    BOARD_Camera_PdwnPinInit();
		BOARD_Camera_pullPdwnPin(true);
	  for(volatile uint32_t i=0; i<100000; i++)				/* PWRDN. */
    {}
    BOARD_Camera_pullRstPin(false);                 /* Reset. */
    for(volatile uint32_t i=0; i<100000; i++)
    {}
		BOARD_Camera_pullPdwnPin(false);
	  for(volatile uint32_t i=0; i<100000; i++)				/* PWR ON. */
    {}			
    BOARD_Camera_pullRstPin(true);                  /* Normal mode. */
    for(volatile uint32_t i=0; i<100000; i++)
    {}

    /* Init the I2C to communicate with the SCCB in OV7670 */
#if BOARD_I2C_EMU_WITH_GPIO
    BOARD_Init_I2C_GPIO_Pins();
#else
    BOARD_Init_I2C_Pins();
    FLEXIO_Ov7670SccbInit(BOARD_CAMERA_I2C_INST);
#endif
    /* Configure the OV7670 camera */
    do
    {
        ov7670Status = OV7670_Init(&s_Ov7670CameraHandler, (ov7670_config_t *)&s_Ov7670CameraConfig);
    } while (ov7670Status != kStatus_OV7670_Success);
}

static void configDMA(void)
{
    uint32_t soff, smod = 0u, size=0u;

    while(1u << size < DMA_TRSF_SIZE) /* size = log2(DMA_TRSF_SIZE) */
    {
        size++;
    }

    if(DMA_TRSF_SIZE == DMA_MINOR_LOOP_SIZE)
    {
        soff = 0u;
    }
    else
    {
        soff = DMA_TRSF_SIZE;
        while(1u << smod < DMA_MINOR_LOOP_SIZE) /* smod = log2(DMA_MINOR_LOOP_SIZE) */
        {
            smod++;
        }
    }
    
    /* Configure DMA TCD */
    DMA0->TCD[FLEXIO_CAMERA_DMA_CHN].SADDR = FLEXIO_CAMERA_GetRxBufferAddress(&s_FlexioCameraDevice);
    DMA0->TCD[FLEXIO_CAMERA_DMA_CHN].SOFF = soff;
    DMA0->TCD[FLEXIO_CAMERA_DMA_CHN].ATTR = DMA_ATTR_SMOD(smod) |
                                            DMA_ATTR_SSIZE(size) |
                                            DMA_ATTR_DMOD(0u) |
                                            DMA_ATTR_DSIZE(size);
    DMA0->TCD[FLEXIO_CAMERA_DMA_CHN].NBYTES_MLNO = DMA_MINOR_LOOP_SIZE;
    DMA0->TCD[FLEXIO_CAMERA_DMA_CHN].SLAST = 0u;
    DMA0->TCD[FLEXIO_CAMERA_DMA_CHN].DADDR = (uint32_t)(*pFlexioCameraFrameBuffer);
    DMA0->TCD[FLEXIO_CAMERA_DMA_CHN].DOFF = DMA_TRSF_SIZE;
    DMA0->TCD[FLEXIO_CAMERA_DMA_CHN].CITER_ELINKNO = DMA_MAJOR_LOOP_SIZE;
    DMA0->TCD[FLEXIO_CAMERA_DMA_CHN].DLAST_SGA = -OV7670_FRAME_BYTES;
    DMA0->TCD[FLEXIO_CAMERA_DMA_CHN].CSR = 0u;
    DMA0->TCD[FLEXIO_CAMERA_DMA_CHN].CSR |= DMA_CSR_DREQ_MASK;
    DMA0->TCD[FLEXIO_CAMERA_DMA_CHN].BITER_ELINKNO = DMA_MAJOR_LOOP_SIZE;

    /* Configure DMA MUX Source */
    DMAMUX->CHCFG[FLEXIO_CAMERA_DMA_CHN] = DMAMUX->CHCFG[FLEXIO_CAMERA_DMA_CHN] &
                                            (~DMAMUX_CHCFG_SOURCE_MASK) | 
                                            DMAMUX_CHCFG_SOURCE(FLEXIO_CAMERA_DMA_MUX_SRC);
    /* Enable DMA channel. */
    DMAMUX->CHCFG[FLEXIO_CAMERA_DMA_CHN] |= DMAMUX_CHCFG_ENBL_MASK;
}

void FLEXIO_Ov7670CaptureStart(void)
{
    if(pFlexioCameraFrameBuffer == &g_FlexioCameraFrameBuffer[OV7670_FRAME_BUFFER_CNT-1])
    {
        pFlexioCameraFrameBuffer = &g_FlexioCameraFrameBuffer[0];
    }
    else
    {
        pFlexioCameraFrameBuffer++;
    }

    DMA0->TCD[FLEXIO_CAMERA_DMA_CHN].DADDR = (uint32_t)pFlexioCameraFrameBuffer;

    /* Enable DMA channel request. */
    DMA0->SERQ = DMA_SERQ_SERQ(FLEXIO_CAMERA_DMA_CHN);
}

void FLEXIO_Ov7670CaptureDone(void)
{
    FLEXIO_CAMERA_ClearStatusFlags(&s_FlexioCameraDevice,
                                   kFLEXIO_CAMERA_RxDataRegFullFlag | kFLEXIO_CAMERA_RxErrorFlag);

    pLCDFrameBuffer = pFlexioCameraFrameBuffer;
}

void FLEXIO_Ov7670Init(void)
{
    configFlexIO();

//    configCamera();

    configDMA();

    /* Enable FlexIO DMA request. */
    FLEXIO_CAMERA_EnableRxDMA(&s_FlexioCameraDevice, true);
    /* Init VSYNC pin, and enable the GPIO interrupt. */
    BOARD_Camera_VsynPinInit();
}

