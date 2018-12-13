#ifndef _BOARD_H_
#define _BOARD_H_

#include "clock_config.h"
#include "fsl_common.h"
#include "fsl_gpio.h"
#include "./delay/core_delay.h" 

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief The board name */
#define BOARD_NAME                    "YH i.MX RT1021 EVK Board"

/* 调试串口定义的信息 */
#define BOARD_DEBUG_UART_TYPE         DEBUG_CONSOLE_DEVICE_TYPE_LPUART
#define BOARD_DEBUG_UART_BASEADDR     (uint32_t) LPUART1
#define BOARD_DEBUG_UART_INSTANCE     1U

#define BOARD_DEBUG_UART_CLK_FREQ     BOARD_DebugConsoleSrcFreq()

#define BOARD_UART_IRQ                LPUART1_IRQn
#define BOARD_UART_IRQ_HANDLER        LPUART1_IRQHandler

#define ILI9341_RESET_GPIO      GPIO3
#define ILI9341_RESET_GPIO_PIN  9u

#define ILI9341_BL_GPIO         GPIO3
#define ILI9341_BL_GPIO_PIN     25u

/* FlexIO Camera configurations */
#define BOARD_CAMERA_FLEXIO_INST                    FLEXIO1
#define BOARD_CAMERA_FLEXIO_DATA_PIN_START_INDEX    8
#define BOARD_CAMERA_FLEXIO_DATA_PIN_END_INDEX      15
#define BOARD_CAMERA_FLEXIO_PCLK_PIN_INDEX          6
#define BOARD_CAMERA_FLEXIO_HREF_PIN_INDEX          2
#define BOARD_CAMERA_VSYNC_GPIO_BASE                GPIO1
#define BOARD_CAMERA_VSYNC_PIN_INDEX                26u
#define BOARD_CAMERA_VSYNC_IRQn                     GPIO1_Combined_16_31_IRQn
#define BOARD_CAMERA_VSYNC_IRQHandler               GPIO1_Combined_16_31_IRQHandler
#define BOARD_CAMERA_RST_GPIO_BASE                  GPIO1
#define BOARD_CAMERA_RST_PIN_INDEX                  24u
#define BOARD_CAMERA_PDWN_GPIO_BASE                 GPIO1
#define BOARD_CAMERA_PDWN_PIN_INDEX                 3u

/* I2C for camera SCCB. */
#define BOARD_CAMERA_I2C_INST                       LPI2C1

/* I2C for camera SCCB. */
#define BOARD_I2C_SCL_GPIO_BASE         GPIO1
#define BOARD_I2C_SDA_GPIO_BASE         GPIO1
#define BOARD_I2C_SCL_PIN_INDEX         30u
#define BOARD_I2C_SDA_PIN_INDEX         31u

#ifndef BOARD_DEBUG_UART_BAUDRATE
#define BOARD_DEBUG_UART_BAUDRATE     (115200U)
#endif /* BOARD_DEBUG_UART_BAUDRATE */

/*! @brief FLASH空间大小 */
#define BOARD_FLASH_SIZE    (0x2000000U)

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
 * API
 ******************************************************************************/
uint32_t BOARD_DebugConsoleSrcFreq(void);

void BOARD_InitDebugConsole(void);

void BOARD_ConfigMPU(void);
  
void CopyAndUseRAMVectorTable(void);
void BOARD_Camera_RstPinInit(void);
void BOARD_Camera_PdwnPinInit(void);
void BOARD_Camera_VsynPinInit(void);
void BOARD_Camera_pullRstPin(bool value);
void BOARD_Camera_pullPdwnPin(bool value);
/* DMA channel assigements. */
#define FLEXIO_CAMERA_DMA_CHN           0u
#define FLEXIO_CAMERA_DMA_MUX_SRC       (kDmaRequestMuxFlexIO1Request0Request1 & 0xFF)
#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _BOARD_H_ */
