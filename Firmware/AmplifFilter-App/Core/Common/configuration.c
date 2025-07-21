/**
 * @file       configuration.c
 * @brief      Configuration and tools for register map implementation
 * @addtogroup grConfig
 * @{
 */

/* Includes ------------------------------------------------------------------*/

#include "configuration.h"
#include "config_app.h"
#include "reg_map.h"


//#include "flash_app.h"

/* Private defines -----------------------------------------------------------*/
/* Private macros  -----------------------------------------------------------*/
/* Private typedefs ----------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

/**
 * Definition of firmware information block in the flash memory
 */
const uint32_t __attribute__((section (".sectionFwInfo"))) CONF_FIRMWARE_INFO[8] = {
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};

/**
 * Definition of empty (default) firmware information block. It is used for comparison.
 */
const uint32_t __attribute__((section (".sectionEndOfFlash"))) CONF_FIRMWARE_INFO_DEFAULT[8] = {
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};


/**
 * Vector of register lengths from ID
 */
const uint32_t CONF_LENGTH[16] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768};


/* Private variables ---------------------------------------------------------*/

/**
 * Shadow storage for synchronization of registers
 */
static uint8_t CONF_REG_SYN_LOCAL[CONF_REG_LOCAL_LENGTH];

/* Private variables ---------------------------------------------------------*/
/* Public variables ----------------------------------------------------------*/

/**
 * Vector of block size limits
 */
extern const uint32_t CONF_REG_LIMIT[CONF_REG_BLOCK_NUMBER];

/**
 * Vector of register ID to store to Flash
 */
extern const uint32_t CONF_REG_FLASH[CONF_REG_FLASH_NUMBER];

/* Extern function prototypes ------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/



/* Functions -----------------------------------------------------------------*/


Status_t Config_Init(void)
{
  Status_t ret = STATUS_OK;

  /* Register map dimensions check */
  if (CONF_DIM_CONDITION)
  {
    Error_Handler();
  }

  /* Use factory default after every power on */
  RegMap_RestoreFactoryValues();

  /* Initialize application-specific stuff */
  Config_AppInit();


  /* Check data in BSRAM */
//  Bsram_Init();

  return ret;
}



Status_t Config_ApplyConfig(uint32_t id)
{
  Status_t ret = STATUS_OK;

  Config_Callback(id);

  /* If this value should be stored in Flash, notify flash task */
  if ((id & 0x070) == 0x070)
  {
//    if (CONF_INT(CONF_SYS_TASK_CONTROL) != 1)
    {
//      FlashApp_RequestConfStorage();
    }
  }

  return ret;
}





Status_t Config_ReadStream(uint8_t *data, uint32_t length)
{
  Status_t ret = STATUS_OK;
  uint32_t reqIdx = 0;
  uint32_t id;
  uint32_t sizeReg;
  uint32_t i;
  uint32_t value;

  /* First entry must be CONF_SYS_REGMAP_VERSION with major part equal to our factory set value */
  id = *((uint32_t *) (data));
  value = *((uint32_t *) (data + 4));

  if ((id != CONF_SYS_REGMAP_VERSION) || (value & 0xFFFF0000) != (CONF_INT(CONF_SYS_REGMAP_VERSION) & 0xFFFF0000) )
  {
    ret = STATUS_ERROR;
  }
  else
  {
    /* Process the whole received stream */
    while ((reqIdx < length))
    {
      /* Take ID */
      id = *((uint32_t *) (data + reqIdx));
      reqIdx += sizeof(uint32_t);
      sizeReg = CONF_BYTE_LEN_ID(id);

      /* Check that this ID is known */
      ret = STATUS_ERROR;
      for (i = 0; i < sizeof(CONF_REG_FLASH)/sizeof(uint32_t); i++)
      {
        if (CONF_REG_FLASH[i] == id)
        {
          ret = STATUS_OK;
        }
      }

      if (ret == STATUS_OK)
      {
        /* Write new value into register storage */
        memcpy(CONF_PTR(id), data + reqIdx, sizeReg);
      }

      reqIdx += sizeReg;
    }
  }
  return ret;
}

Status_t Config_FillStream(uint8_t *data, uint32_t *length, uint32_t maxLength)
{
  Status_t ret = STATUS_OK;
  uint32_t reqIdx = 0;
  uint32_t id;
  uint32_t sizeReg;
  uint32_t i;

  /* Go through all the known config ID that need to be stored in flash */
  for (i = 0; i < sizeof(CONF_REG_FLASH)/sizeof(uint32_t); i++)
  {
    /* Take next ID from the list of known IDs */
    id = CONF_REG_FLASH[i];
    sizeReg = CONF_BYTE_LEN_ID(id);

    if (reqIdx + 4 + sizeReg <= maxLength)
    {
      /* Store ID into config stream */
      *((uint32_t *) (data + reqIdx)) = id;
      reqIdx += sizeof(uint32_t);

      /* Store config value into stream */
      memcpy(data + reqIdx, CONF_PTR(id), sizeReg);
      reqIdx += sizeReg;
    }
    else
    {
      ret = STATUS_ERROR;
    }
  }

  *length = reqIdx;

  return ret;
}

Status_t Config_NeedToSync(uint8_t *data, uint16_t *length)
{
  Status_t ret = STATUS_ERROR;
  int i;
  uint32_t packetIdx = 0;
  uint32_t tempIdx = 0;
  uint32_t sizeReg;
  uint32_t id;

  /* Go through all the known config ID that need to be synchronized with server */
  for (i = 0; i < sizeof(CONF_REG_SYNCED) / sizeof(uint32_t); i++)
  {
    /* Take current ID and Length */
    id = CONF_REG_SYNCED[i];
    sizeReg = CONF_BYTE_LEN_ID(id);

    /* Is current value different from temporary one */
    if (memcmp(CONF_PTR(id), CONF_REG_SYN_LOCAL + tempIdx, sizeReg) != 0)
    {
      /* Add this register to stream */
      memcpy(data + packetIdx, &id, sizeof(id));
      memcpy(data + packetIdx + 4, CONF_PTR(id), sizeReg);

      /* Copy current value of register into temporary storage so it is not sent next time */
      memcpy(CONF_REG_SYN_LOCAL + tempIdx, CONF_PTR(id), sizeReg);

      /* Increment packetIdx and  by size of register + size of ID */
      packetIdx += sizeReg + 4;

      /* Set return value to OK */
      ret = STATUS_OK;
    }

    /* Increment tempIdx pointer to synced storage */
    tempIdx += sizeReg;
  }

  /* Return length of data via pointer to length */
  *length = packetIdx;

  return ret;
}




Status_t Config_CheckLimits(uint32_t id)
{
  Status_t ret = STATUS_ERROR;

  if (CONF_BLOCK_ID(id) < (CONF_REG_BLOCK_NUMBER))
  {
    if (CONF_REG_LIMIT[CONF_BLOCK_ID(id)] >= (CONF_ADDR_ID(id) + CONF_BYTE_LEN_ID(id)))
    {
      ret = STATUS_OK;
    }
  }
  return ret;
}




/* Private Functions ---------------------------------------------------------*/





/** @} */
