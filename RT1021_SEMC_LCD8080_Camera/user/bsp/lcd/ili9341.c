/*
 * The Clear BSD License
 * Copyright 2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted(subject to the limitations in the disclaimer below) provided
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
 *(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include "ili9341.h"
#include "board.h"

/*******************************************************************************
 * Functions
 ******************************************************************************/
static void ILI9341_Delay(uint32_t t)
{
    for(; t>0; t--)
    {
        for(volatile uint32_t i=10000; i>0; i--)
        {
        }
    }
}

static void ILI9341_ConfigReg(void)
{
    ILI9341_Write_Cmd(0xCB);
    ILI9341_Write_Data(0x39);
    ILI9341_Write_Data(0x2C);
    ILI9341_Write_Data(0x00);
    ILI9341_Write_Data(0x34);
    ILI9341_Write_Data(0x02);

    ILI9341_Write_Cmd(0xCF);
    ILI9341_Write_Data(0x00);
    ILI9341_Write_Data(0XC1);
    ILI9341_Write_Data(0X30);

    ILI9341_Write_Cmd(0xE8);
    ILI9341_Write_Data(0x85);
    ILI9341_Write_Data(0x00);
    ILI9341_Write_Data(0x78);

    ILI9341_Write_Cmd(0xEA);
    ILI9341_Write_Data(0x00);
    ILI9341_Write_Data(0x00);

    ILI9341_Write_Cmd(0xED);
    ILI9341_Write_Data(0x64);
    ILI9341_Write_Data(0x03);
    ILI9341_Write_Data(0X12);
    ILI9341_Write_Data(0X81);

    ILI9341_Write_Cmd(0xF7);
    ILI9341_Write_Data(0x20);

    ILI9341_Write_Cmd(0xC0);
    ILI9341_Write_Data(0x23);

    ILI9341_Write_Cmd(0xC1);
    ILI9341_Write_Data(0x10);

    ILI9341_Write_Cmd(0xC5);
    ILI9341_Write_Data(0x3e);
    ILI9341_Write_Data(0x28);

    ILI9341_Write_Cmd(0xC7);
    ILI9341_Write_Data(0x86);

    ILI9341_Write_Cmd(0x36);
    ILI9341_Write_Data(0x28);

    ILI9341_Write_Cmd(0x3A);
    ILI9341_Write_Data(0x55);

    ILI9341_Write_Cmd(0xB1);
    ILI9341_Write_Data(0x00);
    ILI9341_Write_Data(0x18);

    ILI9341_Write_Cmd(0xB6);
    ILI9341_Write_Data(0x08);
    ILI9341_Write_Data(0x82);
    ILI9341_Write_Data(0x27);

    ILI9341_Write_Cmd(0xF2);
    ILI9341_Write_Data(0x00);

    ILI9341_Write_Cmd(0x26);
    ILI9341_Write_Data(0x01);

    ILI9341_Write_Cmd(0xE0);
    ILI9341_Write_Data(0x0F);
    ILI9341_Write_Data(0x31);
    ILI9341_Write_Data(0x2B);
    ILI9341_Write_Data(0x0C);
    ILI9341_Write_Data(0x0E);
    ILI9341_Write_Data(0x08);
    ILI9341_Write_Data(0x4E);
    ILI9341_Write_Data(0xF1);
    ILI9341_Write_Data(0x37);
    ILI9341_Write_Data(0x07);
    ILI9341_Write_Data(0x10);
    ILI9341_Write_Data(0x03);
    ILI9341_Write_Data(0x0E);
    ILI9341_Write_Data(0x09);
    ILI9341_Write_Data(0x00);

    ILI9341_Write_Cmd(0XE1);
    ILI9341_Write_Data(0x00);
    ILI9341_Write_Data(0x0E);
    ILI9341_Write_Data(0x14);
    ILI9341_Write_Data(0x03);
    ILI9341_Write_Data(0x11);
    ILI9341_Write_Data(0x07);
    ILI9341_Write_Data(0x31);
    ILI9341_Write_Data(0xC1);
    ILI9341_Write_Data(0x48);
    ILI9341_Write_Data(0x08);
    ILI9341_Write_Data(0x0F);
    ILI9341_Write_Data(0x0C);
    ILI9341_Write_Data(0x31);
    ILI9341_Write_Data(0x36);
    ILI9341_Write_Data(0x0F);

    ILI9341_Write_Cmd(0x11);
    ILI9341_Delay(120);

    ILI9341_Write_Cmd(0x29);
    ILI9341_Delay(200);
}

void ILI9341_SetWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    ILI9341_Write_Cmd(ILI9341_CMD_COLUMN_ADDR_SET);
    ILI9341_Write_Data(x1 >> 8);
    ILI9341_Write_Data(x1 & 0xFF);
    ILI9341_Write_Data(x2 >> 8);
    ILI9341_Write_Data(x2 & 0xFF);

    ILI9341_Write_Cmd(ILI9341_CMD_PAGE_ADDR_SET);
    ILI9341_Write_Data(y1 >> 8);
    ILI9341_Write_Data(y1 & 0xFF);
    ILI9341_Write_Data(y2 >> 8);
    ILI9341_Write_Data(y2 & 0xFF);
}
void ILI9341_LightPanel(bool light)
{
    if(light)
    {
        GPIO_PinWrite(ILI9341_BL_GPIO, ILI9341_BL_GPIO_PIN, 0u);
    }
    else
    {
        GPIO_PinWrite(ILI9341_BL_GPIO, ILI9341_BL_GPIO_PIN, 1u);
    }
}
void ILI9341_Init(void)
{
    gpio_pin_config_t gpio_pin_config = {kGPIO_DigitalOutput, 1, kGPIO_NoIntmode};

    GPIO_PinInit(ILI9341_RESET_GPIO, ILI9341_RESET_GPIO_PIN, &gpio_pin_config);
		GPIO_PinInit(ILI9341_BL_GPIO, ILI9341_BL_GPIO_PIN, &gpio_pin_config);

    /* Light the panle. */
    ILI9341_LightPanel(true);
    CPU_TS_TmrInit();
    /* Reset ILI9341. */
    GPIO_PinWrite(ILI9341_RESET_GPIO, ILI9341_RESET_GPIO_PIN, 0u);
    ILI9341_Delay(25);
    ILI9341_Write_Data(0x00);   /* Pull up WR, RD, and RS from the defautl state.*/
    ILI9341_Delay(25);
    GPIO_PinWrite(ILI9341_RESET_GPIO, ILI9341_RESET_GPIO_PIN, 1u);
    ILI9341_Delay(50);

    ILI9341_ConfigReg();
}

void ILI9341_FillSolid(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    uint32_t pixels = (uint32_t)(x2 - x1 + 1) * (uint32_t)(y2 - y1 + 1);

    ILI9341_SetWindow(x1, y1, x2, y2);

    ILI9341_Write_Cmd(ILI9341_CMD_MEMORY_WRITE);
    for(uint32_t i=0; i<pixels; i++)
    {
        ILI9341_Write_Data(color);
    }
}

void ILI9341_FillPic(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t *buf)
{
    uint32_t pixels = (uint32_t)(x2 - x1 + 1) * (uint32_t)(y2 - y1 + 1);

    ILI9341_SetWindow(x1, y1, x2, y2);

    ILI9341_Write_Cmd(ILI9341_CMD_MEMORY_WRITE);
    for(uint32_t i=0; i<pixels; i++)
    {
        ILI9341_Write_Data(*buf++);
    }
}
