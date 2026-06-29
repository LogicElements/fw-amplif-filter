/**
 * @file       oscillogram.c
 * @brief      file_brief
 * @addtogroup gr
 * @{
 */

/* Includes ------------------------------------------------------------------*/

#include "oscillogram.h"
#include "usbd_cdc_if.h"
#include "com_proto.h"

/* Private defines -----------------------------------------------------------*/

#define OSC_DATA_LENGTH       2048

#define OSC_HEADER_SERIES     (20 / 4)

/* Private macros  -----------------------------------------------------------*/
/* Private typedefs ----------------------------------------------------------*/

/**
 * Declaration of all private variables
 */
typedef struct
{
  uint32_t config;
  uint32_t ready;
  float buffer[OSC_HEADER_SERIES + OSC_DATA_LENGTH];      // 5 is the header size
  uint32_t length;
}Oscil_Private_t;

/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/**
 * Instance of all private variables (except HAL handles)
 */
static Oscil_Private_t osc;

/* Private variables ---------------------------------------------------------*/
/* Public variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Functions -----------------------------------------------------------------*/


Status_t Oscil_Init(void)
{
  Status_t ret = STATUS_OK;

  osc.config = conf.osc.mode + ((uint32_t)conf.osc.source << 8);

  return ret;
}

Status_t Oscil_Handle(void)
{
  Status_t ret = STATUS_OK;

  if (Oscil_IsReady() == STATUS_OK)
   {
     ComProto_FillSeries((uint8_t*)osc.buffer, ((uint32_t)conf.osc.source << 16) | 2, conf.sys.uptime, 0, osc.length);
     CDC_Transmit_HS((uint8_t*)osc.buffer, ComProto_GetLength((uint8_t*)osc.buffer));

     osc.length = 0;
   }

  return ret;
}


Status_t Oscil_GetPacket(uint8_t **data, uint16_t *length)
{
  Status_t ret = STATUS_OK;

  *data = (uint8_t *)osc.buffer;

  *length = osc.length;

  osc.ready = 0;
  osc.length = 0;

  return ret;
}


Status_t Oscil_IsReady(void)
{
  Status_t ret = STATUS_ERROR;

  /* Check change of configuration */
  if (osc.config != conf.osc.mode + ((uint32_t)conf.osc.source << 8))
  {
    /* Configuration of oscillogram has been changed */
    osc.config = conf.osc.mode + ((uint32_t)conf.osc.source << 8);
    osc.ready = 0;
  }
  else if (osc.ready != 0)
  {
    ret = STATUS_OK;
    osc.ready = 0;
  }
  else if (conf.osc.source == OSC_DUMMY && conf.osc.start != 0)
  {
    for (int i = 0; i < 1024; i++)
    {
      osc.buffer[i + OSC_HEADER_SERIES] = i % 50;
    }
    osc.length = 1024;
//    osc.ready = 1;

    /* Just once */
    conf.osc.start = 0;
    ret = STATUS_OK;
  }

  return ret;
}



Status_t Oscil_DataAvailable(osc_source_t source, float *data, uint16_t length)
{
  Status_t ret = STATUS_OK;

  if (conf.osc.mode != OSC_DISABLED)
  {
    if (conf.osc.start != 0)
    {
      /* Oscillogram enabled and requested */
      if (source == conf.osc.source)
      {
        /* It is the correct source, take the data */
        osc.length = length;
        SAT_UP(osc.length, OSC_DATA_LENGTH);
        memcpy(osc.buffer + OSC_HEADER_SERIES, data, osc.length * sizeof(float));

        osc.ready = 1;

        /* Clear one-shot start flag */
        if (conf.osc.mode == OSC_ONE_SHOT)
        {
          conf.osc.start = 0;
        }
      }
    }
  }

  return ret;
}


Status_t Oscil_SampleAvailable(void)
{
  Status_t ret = STATUS_OK;
  float newVal;

  if (conf.osc.mode != OSC_DISABLED)
  {
    if (conf.osc.start != 0)
    {
      /* Oscillogram enabled and requested */
      switch (conf.osc.source)
      {
        case OSC_RAW:
          newVal = conf.meas.raw;
          break;
        case OSC_AVERAGE:
              newVal = conf.meas.average;
              break;
        case OSC_RMS:
              newVal = conf.dsp.rms;
              break;
        case OSC_MEAN:
              newVal = conf.dsp.mean;
              break;
        case OSC_VAR:
              newVal = conf.dsp.var;
              break;
        case OSC_STD:
              newVal = conf.dsp.std;
              break;
        case OSC_FIR:
              newVal = conf.dsp.fir;
              break;
        default:
          newVal = -1;
          break;
      }

      /* Insert into buffer */
      if (osc.length < 1024)
      {
        osc.buffer[osc.length + OSC_HEADER_SERIES] = newVal;
        osc.length++;
      }
      else
      {
        osc.ready = 1;
        /* Clear one-shot start flag */
        if (conf.osc.mode == OSC_ONE_SHOT)
        {
          conf.osc.start = 0;
        }
      }

    }
  }

  return ret;
}


/* Private Functions ---------------------------------------------------------*/


/** @} */
