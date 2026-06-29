/**
 * @file       oscillogram.h
 * @version    $(APP_VERSION)
 * @date       $(RELEASE_DATE)
 * @brief      file_brief
 * @author     jan.bartovsky
 *
 * @copyright  Logic Elements Copyright
 *
 * @defgroup gr group_name
 * @{
 * @brief group_brief
 *
 * This module contains
 *
 * @par Main features:
 *
 * @par Example
 * @code
 *
 * @endcode
 */
#ifndef OSCILLOGRAM_H_
#define OSCILLOGRAM_H_

/* Includes ------------------------------------------------------------------*/

#include "common.h"
#include "configuration.h"

/* Definitions----------------------------------------------------------------*/

/* Typedefs-------------------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/


Status_t Oscil_Init(void);

Status_t Oscil_GetPacket(uint8_t **data, uint16_t *length);

Status_t Oscil_IsReady(void);

Status_t Oscil_DataAvailable(osc_source_t source, float *data, uint16_t length);

Status_t Oscil_SampleAvailable(void);

#endif /* OSCILLOGRAM_H_ */
/** @} */
