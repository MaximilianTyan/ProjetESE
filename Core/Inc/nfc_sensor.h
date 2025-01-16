/*
 * nfc_sensor.h
 *
 *  Created on: Jan 13, 2025
 *      Author: xana
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_NFC_SENSOR_H_
#define INC_NFC_SENSOR_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Public Includes ------------------------------------------------------------------*/
#include "stdbool.h"
#include "stdint.h"
#include "stm32l0xx_hal.h"

/* Public types ------------------------------------------------------------*/

/* Public constants --------------------------------------------------------*/

/* Public macro ------------------------------------------------------------*/

/* Public functions prototypes ---------------------------------------------*/
#define INCLUDED_CORRECTLY
void init_nfc(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint32_t cs_pin, GPIO_TypeDef* reset_port, uint32_t reset_pin);
bool get_nfc_id(uint32_t *id);

/* Public defines -----------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* INC_NFC_SENSOR_H_ */
