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

#ifndef __SEMC_8080_LCD__
#define __SEMC_8080_LCD__

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SEMC_DBI_START_ADDRESS (0xA0000000U)

#define DBI_DATA_ADDR   SEMC_DBI_START_ADDRESS
#define DBI_CMD_ADDR    (SEMC_DBI_START_ADDRESS + 0x10000)
#define DBI_DATA16      *((uint16_t *)SEMC_DBI_START_ADDRESS)
#define DBI_CMD16       *(uint16_t *)(SEMC_DBI_START_ADDRESS + 0x10000)
#define DBI_DATA32      *((uint32_t *)SEMC_DBI_START_ADDRESS)
#define DBI_CMD32       *(uint32_t *)(SEMC_DBI_START_ADDRESS + 0x10000)

status_t ConfigSemcDbi(void);
void ConfigDma(void);
void FillPicDma(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t *buf);

#endif /* __SEMC_8080_LCD__ */
