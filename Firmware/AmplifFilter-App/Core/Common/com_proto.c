/**
 * @file       comm_proto.c
 * @brief      file_brief
 * @addtogroup gr
 * @{
 */

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "com_proto.h"

#include "system_msp.h"
#include "configuration.h"
#include "usbd_cdc_if.h"

/* Private defines -----------------------------------------------------------*/\

/**
 * Countdown of parsing loop (depends on number of registers in a single request
 */
#define PARSE_COUNTDOWN_VALUE       400

/**
 * Length of event buffer
 */
#define EVENT_BUFFER                1024

/**
 * Text message length
 */
#define MSG_LENGTH                  32

/* Private macros  -----------------------------------------------------------*/
/* Private typedefs ----------------------------------------------------------*/

typedef enum
{
  VERSION_IDLE = 0,
  VERSION_UNCHECKED,
  VERSION_CHECKED_OK,
  VERSION_CHECKED_WRONG,
}ver_check_t;

/**
 * Definition of all private variables
 */
typedef struct
{
  uint32_t sentResp;          ///< number of received and answered packets

  /* Event variables - implementation dependent */
  char buffer[EVENT_BUFFER];  ///< Event buffer
  uint32_t ptrWr;             ///< Write pointer
  uint32_t ptrRd;             ///< Read pointer
  char msg[MSG_LENGTH];       ///< Message buffer

  ver_check_t version_check;     ///< Check version of firmware regarding flash size

} ComProto_Private_t;

/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/**
 * Instance of all private variables (except HAL handles)
 */
static ComProto_Private_t com;

/* Private variables ---------------------------------------------------------*/
/* Public variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

static uint32_t ComProto_FillEvents(uint32_t *data, uint32_t *resp);

static Status_t ComProto_ReadReg(uint8_t *data, uint8_t *txResp);

static Status_t ComProto_WriteReg(uint8_t *data, uint8_t *txResp);

static Status_t ComProto_FwUpgrade(uint8_t *data, uint8_t *txResp);


/* Functions -----------------------------------------------------------------*/

Status_t ComProto_Init(void)
{
  Status_t ret = STATUS_OK;

  com.version_check = VERSION_IDLE;

  return ret;
}


Status_t ComProto_CheckProtoLength(uint8_t *data, uint16_t length)
{
  Status_t ret = STATUS_ERROR;
  uint16_t packetLen;

  /* Get packet length */
  packetLen = ComProto_GetLength(data);

  /* Check length, start byte, minimal length */
  if (length >= packetLen && length >= COM_PROTO_MIN_LENGTH)
  {
    ret = STATUS_OK;
  }

  if (data[0] != COM_PROTO_UART_START_BYTE && length >= COM_PROTO_MIN_LENGTH)
  {
    /* Possible dead-lock */
    CDC_ClearRxBuffer();
  }

  return ret;
}

uint16_t ComProto_GetLength(uint8_t *data)
{
  /* Extract data length from header */
  return data[2] + data[3] * 256;
}




Status_t ComProto_FillSeries(uint8_t *buffer, uint32_t id, uint32_t timestamp, uint32_t delta, uint32_t count)
{
  Status_t ret = STATUS_OK;
  uint32_t *buf = (uint32_t *)buffer;
  uint16_t length;

  /* Length of packet */
  length = COM_PROTO_HEADER_LEN + 16 + count * CONF_BYTE_LEN_ID(id);

  /* Header */
  buffer[0] = COM_PROTO_UART_START_BYTE;
  buffer[1] = COM_PROTO_TIME_SER;
  buffer[2] = length;
  buffer[3] = length >> 8;

  /* Series information */
  buf[1] = id;
  buf[2] = timestamp;
  buf[3] = delta;
  buf[4] = count;

  return ret;
}



/* Private Functions ---------------------------------------------------------*/

Status_t ComProto_ProcessPacket(uint8_t *data, uint8_t* txResp)
{
  Status_t ret = STATUS_OK;
  uint16_t respIdx = 0;

  /* Fill response packet header */
  txResp[0] = data[0];
  txResp[1] = COM_PROTO_READ_REG_RESP;
  txResp[2] = data[2];
  txResp[3] = data[3];

  switch (data[1])
  {
    /* Get Event packet */
    case COM_PROTO_GET_EVENT:
      respIdx = ComProto_FillEvents((uint32_t *)data, (uint32_t *)(txResp + COM_PROTO_HEADER_LEN));
      respIdx += COM_PROTO_HEADER_LEN;

      txResp[1] = COM_PROTO_EVENT_RESP;
      txResp[2] = respIdx;
      txResp[3] = respIdx / 256;

      break;

      /* Write register */
    case COM_PROTO_WRITE_REG:
      ret = ComProto_WriteReg(data, txResp);

      break;

      /* Read register */
    case COM_PROTO_READ_REG:
      ret = ComProto_ReadReg(data, txResp);

      break;

      /* Update firmware */
    case COM_PROTO_FW_UPGRADE:

      ret = ComProto_FwUpgrade(data, txResp);


      break;

    default:
      break;
  }

  if (ret == STATUS_OK)
  {
    com.sentResp++;
  }

  return ret;
}




static Status_t ComProto_WriteReg(uint8_t *data, uint8_t *txResp)
{
  Status_t ret = STATUS_OK;
  uint32_t countdown = PARSE_COUNTDOWN_VALUE;
  uint32_t reqIdx;
  uint32_t id;
  uint32_t sizeReg;
  uint32_t length;

  /* Get length */
  length = ComProto_GetLength(data);

  /* Iterate until the end of request packet (countdown guard) */
  reqIdx = COM_PROTO_HEADER_LEN;
  id = *((uint32_t *) (data + reqIdx));

  /* Write command guard according to login. When no permission, let through the login attempt */

    while (reqIdx < length && countdown != 0)
    {
      id = *((uint32_t *) (data + reqIdx));
      memcpy(txResp + reqIdx, &id, sizeof(uint32_t));
      reqIdx += sizeof(uint32_t);

      sizeReg = CONF_BYTE_LEN_ID(id);

      /* Check the correctness of ID */
      ret = Config_CheckLimits(id);

      if (ret == STATUS_OK)
      {
        /* Write new value into register storage */
        memcpy(CONF_PTR(id), data + reqIdx, sizeReg);

        /* Copy register value into response */
        memcpy(txResp + reqIdx, CONF_PTR(id), sizeReg);

        /* Apply received configuration */
        ret = Config_ApplyConfig(id);
      }
      else
      {
        /* Id outside memory boundaries, return 0xF0 values */
        memset(data + reqIdx, 0xF0, sizeReg);
      }

      reqIdx += sizeReg;
      countdown--;
    }

    if (countdown)
    {
      ret = STATUS_OK;
    }
    else
    {
      ret = STATUS_ERROR;
    }


  return ret;
}



static Status_t ComProto_ReadReg(uint8_t *data, uint8_t *txResp)
{
  Status_t ret = STATUS_OK;
  uint32_t countdown = PARSE_COUNTDOWN_VALUE;
  uint32_t respIdx;
  uint32_t reqIdx;
  uint32_t id;
  uint32_t sizeReg;
  uint32_t length;

  /* Get packet length */
  length = ComProto_GetLength(data);

  /* Iterate until the end of request packet (countdown guard) */
  reqIdx = COM_PROTO_HEADER_LEN;
  respIdx = COM_PROTO_HEADER_LEN;
  while ((reqIdx < length) && countdown != 0)
  {
    id = *((uint32_t *) (data + reqIdx));
    sizeReg = CONF_BYTE_LEN_ID(id);

    if (Config_CheckLimits(id) == STATUS_OK)
    {
      /* Copy Id to response  */
      memcpy(txResp + respIdx, &id, sizeof(uint32_t));
      respIdx += sizeof(uint32_t);

      /* Copy register value into response */
      memcpy(txResp + respIdx, CONF_PTR(id), sizeReg);

      /* Increment values */
      reqIdx += sizeof(uint32_t);
      respIdx += sizeReg;
    }
    else
    {
      /* Increment values */
      reqIdx += sizeof(uint32_t);
    }

    countdown--;
  }
  txResp[2] = respIdx;
  txResp[3] = respIdx / 256;

  if (countdown)
  {
    ret = STATUS_OK;
  }
  else
  {
    ret = STATUS_ERROR;
  }

  return ret;
}



static Status_t ComProto_FwUpgrade(uint8_t *data, uint8_t *txResp)
{
  Status_t ret = STATUS_OK;
  uint16_t respIdx;
  uint16_t dataLength;
  uint32_t offset;
  uint32_t retCode;
  uint32_t version;
  uint32_t date;
  uint32_t shift = 0;

  dataLength = *(uint16_t *) (data + 2);
  dataLength -= 8;
  offset = *(uint32_t *)(data + 4);

  /* Zero offset within sector means we need to erase sector */
  if (offset % 0x20000 == 0)
  {
    System_FlashErase((uint32_t)CONF_C_APP_BUFFER_OFFSET + offset, (uint32_t)CONF_C_APP_BUFFER_OFFSET + offset);

    /* First packet */
    if (offset == 0)
    {
      com.version_check = VERSION_UNCHECKED;
    }
  }

  if (dataLength == 0)
  {
    /* The very last packet, verify image */
    ret = System_VerifyImage((uint32_t*)CONF_C_APP_BUFFER_OFFSET);
    retCode = ret;
    if (ret == STATUS_OK)
    {
      version = *((uint32_t*)(CONF_C_APP_BUFFER_OFFSET + (uint32_t)CONF_FW_INFO_OFFSET));
      date = *((uint32_t*)(CONF_C_APP_BUFFER_OFFSET + (uint32_t)CONF_FW_INFO_OFFSET + 4));
    }
  }
  else if (dataLength % 32 != 0)
  {
    /* Wrong length of data */
    retCode = 2;
  }
  else
  {
    /* Check uint64 alignment */
    if (((uint32_t)data % 4) != 0)
    {
      shift = ((uint32_t)data % 4);
      memcpy(data + 8 - shift, data + 8, dataLength);
    }

    /* Write data to flash */
    retCode = System_FlashProgram((uint32_t)CONF_C_APP_BUFFER_OFFSET + offset, data + 8 - shift, dataLength);
  }

  /* Check version of incoming file */
  if (com.version_check == VERSION_UNCHECKED && offset > 1024)
  {
    uint16_t flash_size_kb = *(const uint16_t*)(0x1FF1E880UL);

    version = *((uint32_t*)(CONF_C_APP_BUFFER_OFFSET + (uint32_t)CONF_FW_INFO_OFFSET));

    if (version < 34 && flash_size_kb == 1024)
    {
      retCode = 2;
    }

    com.version_check = VERSION_IDLE;
  }

  respIdx = 12;

  txResp[1] = COM_PROTO_FW_UPG_ACK;
  txResp[2] = respIdx;
  txResp[3] = respIdx / 256;

  *(uint32_t *)(txResp + 4) = offset + dataLength;
  *(uint32_t *)(txResp + 8) = retCode;

  return ret;
}


static uint32_t ComProto_FillEvents(uint32_t *data, uint32_t *resp)
{
  uint32_t timestamp;
  uint32_t length = 0;

  /* Timestamp is unused so far */
  UNUSED(timestamp);

  /* Skip the packet header */

  /* Check for timestamp ID */
  if (*(data + 1) == CONF_SYS_UPTIME)
  {
    timestamp = *(data + 2);
    length = com.ptrWr - com.ptrRd;

    if (length)
    {
      /* Copy data to output packet */
      memcpy(resp, com.buffer + com.ptrRd, length);
      com.ptrRd += length;
    }

    if (com.ptrRd == com.ptrWr)
    {
      com.ptrRd = 0;
      com.ptrWr = 0;
    }
  }

  return length;
}





/** @} */
