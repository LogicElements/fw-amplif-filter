/**
 * @file       config_app.c
 * @brief      Application-specific configuration callback implementation
 * @addtogroup grConfig
 * @{
 */

/* Includes ------------------------------------------------------------------*/

#include "config_app.h"
//#include "common.h"
#include "stdlib.h"
#include "configuration.h"

/* Application includes */
#include "system_msp.h"

/* Private defines -----------------------------------------------------------*/

#define CA_MSG_LEN            16

#define CA_FACT_INFO_WORDS    16

#define CA_FACT_SN_PREFIX     4

/* Private macros  -----------------------------------------------------------*/
/* Private typedefs ----------------------------------------------------------*/

typedef struct
{
  char ref1Re[CA_MSG_LEN];
  char ref1Im[CA_MSG_LEN];
  char ref2Re[CA_MSG_LEN];
  char ref2Im[CA_MSG_LEN];

  uint32_t factInfo[CA_FACT_INFO_WORDS];

}ConfigApp_Private_t;

/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static ConfigApp_Private_t ca;

/* Public variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

static uint32_t Config_BootstrapPins(void);

//Status_t Config_WriteSerialNumber(void);

/* Functions -----------------------------------------------------------------*/



Status_t Config_AppInit(void)
{
  Status_t ret = STATUS_OK;

  /* Read entire factory information block */
  memcpy(&ca.factInfo, CONF_C_BOOTLOADER_OFFSET + (uint32_t)CONF_FW_INFO_OFFSET, CA_FACT_INFO_WORDS * 4);


  return ret;
}




Status_t Config_Callback(uint32_t id)
{
  Status_t ret = STATUS_OK;

  /* Check if some value need further action to propagate */
  switch (CONF_BLOCK_ID(id))
  {
    case CONF_BLOCK_ID(CONF_SYS_UPTIME):

      break;

    default:
      break;
  }

  return ret;
}


/* Private Functions ---------------------------------------------------------*/



/** @} */
