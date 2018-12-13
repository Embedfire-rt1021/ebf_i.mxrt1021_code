/*
 * The Clear BSD License
 * Copyright 2017 NXP
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
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
 * LOSS OF USE, DATA, OR PROFITS;OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_device_registers.h"
#include "pin_mux.h"
#include "fsl_semc.h"
#include "fsl_edma.h"
#include "fsl_dmamux.h"
#include "clock_config.h"
#include "semc_8080_lcd.h"
#include "ili9341.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define FILL_PIC_DMA_CHANNEL    1

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile bool Dma_Transfer_Done = false;
edma_handle_t edmaHandle;

/*******************************************************************************
 * Code
 ******************************************************************************/
status_t ConfigSemcDbi(void)
{
#ifndef SEMC_SDRAM_USED
    semc_config_t semcConfig;
    semc_dbi_config_t dbiConfig;
    uint32_t clockFrq = CLOCK_GetFreq(kCLOCK_SemcClk);

    /* Initializes the MAC configure structure to zero. */
    memset(&semcConfig, 0, sizeof(semc_config_t));
    memset(&dbiConfig, 0, sizeof(semc_dbi_config_t));

    /* Initialize SEMC. */
    SEMC_GetDefaultConfig(&semcConfig);
    semcConfig.dqsMode = kSEMC_Loopbackdqspad; /* For more accurate timing. */
    SEMC_Init(SEMC, &semcConfig);

    /* Configure SEMC DBI(8080). */
    dbiConfig.csxPinMux = kSEMC_MUXCSX0;
    dbiConfig.address = SEMC_DBI_START_ADDRESS;
    dbiConfig.memsize_kbytes = 1024;
    dbiConfig.columnAddrBitNum = kSEMC_Dbi_Colum_12bit;
    dbiConfig.burstLen = kSEMC_Dbi_BurstLen64;
    dbiConfig.portSize = kSEMC_PortSize16Bit;
    dbiConfig.tCsxSetup_Ns = 45;
    dbiConfig.tCsxHold_Ns = 45;
    dbiConfig.tWexLow_Ns = 45;
    dbiConfig.tWexHigh_Ns = 45;
    dbiConfig.tRdxLow_Ns = 45;
    dbiConfig.tRdxHigh_Ns = 45;
    dbiConfig.tCsxInterval_Ns = 0;

    return SEMC_ConfigureDBI(SEMC, &dbiConfig, clockFrq);

#else /* Configure SEMC DBI by accessing the registers directly, instead of using the SDK drivers. */

    SEMC->IOCR = SEMC->IOCR | (7 << SEMC_IOCR_MUX_CSX0_SHIFT);                  /* Use CSX0. */

    SEMC->BR[7] = (SEMC_DBI_START_ADDRESS & SEMC_BR_BA_MASK)    |
                  SEMC_BR_MS(8)                             |
                  SEMC_BR_VLD_MASK;

    SEMC->DBICR0 = SEMC_DBICR0_PS(kSEMC_PortSize16Bit)      |
                   SEMC_DBICR0_BL(kSEMC_Dbi_BurstLen64)     |
                   SEMC_DBICR0_COL(kSEMC_Dbi_Colum_12bit);

    /* Timing setting. */
    SEMC->DBICR1 = SEMC_DBICR1_CES(8u) |
                   SEMC_DBICR1_CEH(8u) |
                   SEMC_DBICR1_WEL(8u) |
                   SEMC_DBICR1_WEH(8u) |
                   SEMC_DBICR1_REL(8u) |
                   SEMC_DBICR1_REH(8u) |
                   SEMC_DBICR1_CEITV(0u);

    return kStatus_Success;
#endif
}

void EDMA_Callback(edma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
    if (handle->channel == FILL_PIC_DMA_CHANNEL && transferDone)
    {
        Dma_Transfer_Done = true;
    }
}

void ConfigDma(void)
{
    edma_config_t edma_config;

    DMAMUX_Init(DMAMUX);

    EDMA_GetDefaultConfig(&edma_config);
    EDMA_Init(DMA0, &edma_config);

    DMAMUX_EnableAlwaysOn(DMAMUX, FILL_PIC_DMA_CHANNEL, true);
    DMAMUX_EnableChannel(DMAMUX, FILL_PIC_DMA_CHANNEL);

    EDMA_CreateHandle(&edmaHandle, DMA0, FILL_PIC_DMA_CHANNEL);
    EDMA_SetCallback(&edmaHandle, EDMA_Callback, NULL);
}

void FillPicDma(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t *buf)
{
    uint32_t bytes = (uint32_t)(x2 - x1 + 1) * (uint32_t)(y2 - y1 + 1) * 2;
    edma_transfer_config_t transferConfig;

    ILI9341_SetWindow(x1, y1, x2, y2);

    ILI9341_Write_Cmd(ILI9341_CMD_MEMORY_WRITE);

    EDMA_PrepareTransfer(&transferConfig,
                         buf, 32,
                         (void *)DBI_DATA_ADDR, 32,
                         bytes, bytes,
                         kEDMA_MemoryToPeripheral);

    EDMA_SubmitTransfer(&edmaHandle, &transferConfig);

    Dma_Transfer_Done = false;
    EDMA_StartTransfer(&edmaHandle);

    /* Wait for EDMA transfer finish */
    while (Dma_Transfer_Done != true)
    {
    }
}

