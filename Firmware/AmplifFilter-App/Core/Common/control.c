/**
 * @file       control.c
 * @brief      Command register and state machine implementation
 * @addtogroup grControl
 * @{
 */

/* Includes ------------------------------------------------------------------*/

#include "control.h"
#include "configuration.h"
#include "system_msp.h"
#include "main.h"
//#include "flash_app.h"


/* Private defines -----------------------------------------------------------*/

/**
 * Command keys
 */
#define CONTROL_RESET           9901
#define CONTROL_FACTORY         8801
#define CONTROL_TESTING         7701
#define CONTROL_ERROR           6600
#define CONTROL_WDG_LATCH       5501
#define CONTROL_CAL_REFRESH     4401
#define CONTROL_CAL_UPDATE      4499
#define CONTROL_MEM_BBM_STAT    3301
#define CONTROL_MEM_BBM_RUN     3399

/* Private macros  -----------------------------------------------------------*/

/* Private typedefs ----------------------------------------------------------*/

/**
 * Declaration of all private variables
 */
typedef struct
{
  uint16_t prevCmd;
  uint16_t localError;
  uint32_t prevTick;
  uint32_t count;
  uint32_t restartPend;

}Control_Private_t;

/* Private constants ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/**
 * Instance of all private variables (except HAL handles)
 */
static Control_Private_t cntrl;

/* Public variables ----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Functions -----------------------------------------------------------------*/

Status_t Control_Init(void)
{
  Status_t ret = STATUS_OK;

  return ret;
}

Status_t Control_Handle(void)
{
  Status_t ret = STATUS_OK;

  /* Handle Reset commands */
  if (conf.sys.command == CONTROL_RESET || cntrl.restartPend)
  {
    System_Delay(1000);
    System_Reset();
  }

  /* Handle factory reset command */
  else if (conf.sys.command == CONTROL_FACTORY)
  {
//    FlashApp_RequestFactorySettings();
  }
  else if (conf.sys.command == CONTROL_WDG_LATCH)
  {
    /* The delay below should cause IWDG to restart device */
    System_Delay(8000);
  }

  /* Handle set error command */
  else if (conf.sys.command > CONTROL_ERROR && conf.sys.command < (CONTROL_ERROR + 100))
  {
    cntrl.localError = conf.sys.command - CONTROL_ERROR;
    conf.sys.error |= (1 << (cntrl.localError - 1));
  }

  /* Handle clear error command */
  else if (conf.sys.command == CONTROL_ERROR)
  {
    cntrl.localError = 0;
    conf.sys.error = 0;
  }

  /* Handle refresh the calibration data */
  else if (conf.sys.command == CONTROL_CAL_REFRESH)
  {
    //Calib_Init(); ***
  }

  /* Handle update the calibration data */
  else if (conf.sys.command == CONTROL_CAL_UPDATE)
  {
    //Calib_Write(); ***
  }

  /* Testing mode disables setting of modbus address from DIP */
  if (conf.sys.command == CONTROL_TESTING)
  {
    SET_BIT(conf.sys.status, STAT_BIT_TESTING);
  }
  else
  {
    CLEAR_BIT(conf.sys.status, STAT_BIT_TESTING);
//    conf.com.mb_address = (conf.sys.io_input & 0xF) + MODBUS_BASE_ADDRESS;
  }

  /* Error flag of status register */
  if (cntrl.localError != 0 || conf.sys.error != 0)
  {
    SET_BIT(conf.sys.status, STAT_BIT_ERROR);
  }
  else
  {
    CLEAR_BIT(conf.sys.status, STAT_BIT_ERROR);
  }

  /* Increment timestamp */
  if (HAL_GetTick() > cntrl.prevTick)
  {
    cntrl.prevTick += 1000;
    conf.sys.uptime += 1;

    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
  }

  /* Store previous command */
  cntrl.prevCmd = conf.sys.command;
  conf.sys.command = 0;
  cntrl.count++;

  return ret;
}

Status_t Control_ScheduleRestart(void)
{
  Status_t ret = STATUS_OK;

  cntrl.restartPend = 1;

  return ret;
}

/* Private Functions ---------------------------------------------------------*/

/** @} */
