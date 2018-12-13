/*
 * The Clear BSD License
 * Copyright 2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
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
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
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

#ifndef _FSL_ILI9341_H_
#define _FSL_ILI9341_H_

#include "fsl_common.h"
#include "semc_8080_lcd.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define ILI9341_FRAME_H_PIXELS      320
#define ILI9341_FRAME_V_PIXELS      240

#define ILI9341_CMD_COLUMN_ADDR_SET 0x2A
#define ILI9341_CMD_PAGE_ADDR_SET   0x2B
#define ILI9341_CMD_MEMORY_WRITE    0x2C

#define RED     0xF800
#define GREEN   0x07E0
#define BLUE    0x001F

/*******************************************************************************
 * API Functions
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

static inline void ILI9341_Write_Cmd(uint16_t value)
{
    DBI_CMD16 = value;
}

static inline void ILI9341_Write_Data(uint16_t value)
{
    DBI_DATA16 = value;
}

static inline uint16_t ILI9341_Read_Data(void)
{
    return DBI_DATA16;
}

void ILI9341_Init(void);

void ILI9341_SetWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

void ILI9341_FillSolid(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);

void ILI9341_FillPic(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t *buf);

#if defined(__cplusplus)
}
#endif
#endif /* _FSL_ILI9341_H_ */
