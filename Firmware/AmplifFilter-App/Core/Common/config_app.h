/**
 * @file       config_app.h
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
#ifndef COMMON_CONFIG_APP_H_
#define COMMON_CONFIG_APP_H_

/* Includes ------------------------------------------------------------------*/

#include "common.h"

/* Definitions----------------------------------------------------------------*/

#define CONF_APP_NOT_TIME_SET           (1 << 0)
#define CONF_APP_NOT_TIME_CALIB         (1 << 1)
#define CONF_APP_NOT_TASK_CONTROL       (1 << 2)
#define CONF_APP_NOT_CALIBRATION        (1 << 3)
#define CONF_APP_NOT_AUTH_ADMIN         (1 << 4)
#define CONF_APP_NOT_AUTH_USER          (1 << 5)
#define CONF_APP_NOT_MODBUS_SET         (1 << 6)
#define CONF_APP_NOT_KP_IN_SET          (1 << 7)
#define CONF_APP_NOT_KP_OUT_SET         (1 << 8)
#define CONF_APP_NOT_DIN_SET            (1 << 9)
#define CONF_APP_NOT_BOW_SET            (1 << 10)
#define CONF_APP_NOT_EXC_REF_SET_1      (1 << 11)
#define CONF_APP_NOT_EXC_REF_SET_2      (1 << 12)
#define CONF_APP_NOT_LINEAR_SET         (1 << 13)
#define CONF_APP_NOT_CUR_SET            (1 << 14)
//#define CONF_APP_NOT_         (1 << 0)

/* Typedefs-------------------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/


Status_t Config_Callback(uint32_t id);

Status_t Config_AppInit(void);

void ConfigApp_TaskBody(void * argument);

Status_t ConfigApp_Notify(uint32_t flag);

#endif /* COMMON_CONFIG_APP_H_ */
/** @} */
