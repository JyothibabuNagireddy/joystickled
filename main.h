/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VCP_RX_Pin GPIO_PIN_11
#define VCP_RX_GPIO_Port GPIOC
#define SPI1_MOSI_DIR_Pin GPIO_PIN_12
#define SPI1_MOSI_DIR_GPIO_Port GPIOC
#define Tamper_key_Pin GPIO_PIN_13
#define Tamper_key_GPIO_Port GPIOC
#define EX_RESET_OD_Pin GPIO_PIN_0
#define EX_RESET_OD_GPIO_Port GPIOC
#define MC_PFCsync1_Pin GPIO_PIN_1
#define MC_PFCsync1_GPIO_Port GPIOC
#define JOY_SEL_Pin GPIO_PIN_0
#define JOY_SEL_GPIO_Port GPIOA
#define MC_BusVoltage_Pin GPIO_PIN_1
#define MC_BusVoltage_GPIO_Port GPIOA
#define MC_CurrentA_Pin GPIO_PIN_2
#define MC_CurrentA_GPIO_Port GPIOA
#define Audio_OUT_L_Pin GPIO_PIN_4
#define Audio_OUT_L_GPIO_Port GPIOA
#define Audio_OUT_R_Pin GPIO_PIN_5
#define Audio_OUT_R_GPIO_Port GPIOA
#define Audio_IN_Pin GPIO_PIN_6
#define Audio_IN_GPIO_Port GPIOA
#define SPI1_MOSI_Pin GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port GPIOA
#define USART1_TX_Pin GPIO_PIN_4
#define USART1_TX_GPIO_Port GPIOC
#define USART1_RX_Pin GPIO_PIN_5
#define USART1_RX_GPIO_Port GPIOC
#define MC_EnIndex_Pin GPIO_PIN_0
#define MC_EnIndex_GPIO_Port GPIOB
#define MC_PFCpwm_Pin GPIO_PIN_1
#define MC_PFCpwm_GPIO_Port GPIOB
#define Potentiometer_Pin GPIO_PIN_2
#define Potentiometer_GPIO_Port GPIOB
#define MC_CurrentC_Pin GPIO_PIN_10
#define MC_CurrentC_GPIO_Port GPIOB
#define MC_EmergencySTOP_Pin GPIO_PIN_12
#define MC_EmergencySTOP_GPIO_Port GPIOB
#define MC_DissipativeBrake_Pin GPIO_PIN_15
#define MC_DissipativeBrake_GPIO_Port GPIOB
#define MC_UH_Pin GPIO_PIN_8
#define MC_UH_GPIO_Port GPIOA
#define MC_VH_Pin GPIO_PIN_9
#define MC_VH_GPIO_Port GPIOA
#define MC_EnA_Pin GPIO_PIN_6
#define MC_EnA_GPIO_Port GPIOC
#define MC_WH_Pin GPIO_PIN_10
#define MC_WH_GPIO_Port GPIOA
#define USART1_CTS_Pin GPIO_PIN_11
#define USART1_CTS_GPIO_Port GPIOA
#define USART1_RTS_Pin GPIO_PIN_12
#define USART1_RTS_GPIO_Port GPIOA
#define SWDAT_Pin GPIO_PIN_13
#define SWDAT_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SDcard_detect_Pin GPIO_PIN_9
#define SDcard_detect_GPIO_Port GPIOC
#define MC_PFCsync2_Pin GPIO_PIN_0
#define MC_PFCsync2_GPIO_Port GPIOD
#define MicroSD_CS_OD_Pin GPIO_PIN_1
#define MicroSD_CS_OD_GPIO_Port GPIOD
#define MC_UL_Pin GPIO_PIN_2
#define MC_UL_GPIO_Port GPIOD
#define MC_VL_Pin GPIO_PIN_3
#define MC_VL_GPIO_Port GPIOD
#define MC_WL_Pin GPIO_PIN_4
#define MC_WL_GPIO_Port GPIOD
#define SPI1_SCK_Pin GPIO_PIN_3
#define SPI1_SCK_GPIO_Port GPIOB
#define SPI1_MISO_Pin GPIO_PIN_4
#define SPI1_MISO_GPIO_Port GPIOB
#define MC_EnB_Pin GPIO_PIN_5
#define MC_EnB_GPIO_Port GPIOB
#define I2C1_SCL_Pin GPIO_PIN_6
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOB
#define LCD_CS_OD_Pin GPIO_PIN_8
#define LCD_CS_OD_GPIO_Port GPIOB
#define MC_NTC_Pin GPIO_PIN_9
#define MC_NTC_GPIO_Port GPIOB
#define VCP_TX_Pin GPIO_PIN_10
#define VCP_TX_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
