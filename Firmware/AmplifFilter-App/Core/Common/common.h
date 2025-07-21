/**
  * @file       common.h
  * @version    $(APP_VERSION)
  * @date       $(RELEASE_DATE)
  * @brief      Common header include
  * @author     jan.bartovsky
  *
  * @copyright  Logic Elements Copyright
  *
  * @defgroup grCommon Common definitions
  * @{
  * @brief Common definitions for all modules
  *
  * This module contains common definitions for all modules.
  *
  * @par Main features:
  * - Status and error codes, types
  * - Error status checking macros
  * - Assert parameters checking macros
  *
  * @par Example
  * @code
  * #include "common.h"
  * @endcode
  */
#ifndef APPLICATION_COMMON_H_
#define APPLICATION_COMMON_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "string.h"

#define DEBUG

/* Definitions - Compilation flags--------------------------------------------*/

/**
 * Compilation-time flag - Disable GSM completely - useful when no GSM modem is assembled on the board
 */


/* Definitions----------------------------------------------------------------*/

/**
 * Success status
 */
#define STATUS_OK       0
/**
 * Error or fail status
 */
#define STATUS_ERROR    1

/**
 * Error or fail status
 */
#define STATUS_TIMEOUT   2

/**
 * Busy status
 */
#define STATUS_BUSY     3


/**
 * Interrupt priorities
 */
/* No RTOS calls, most critical */

/* With RTOS calls, must be >= 5 */

#define PRIO_IRQ_KP_ADC             5       ///< KP analog watchdog

#define PRIO_IRQ_FADC_READY_1       7       ///< FADC1 data ready
#define PRIO_IRQ_KP_REVOLUTION      6       ///< FADC1 data ready

#define PRIO_IRQ_FADC_SPI           6       ///< Read-out of FADC

#define PRIO_IRQ_DIN_ADC_DMA        8       ///< Din adc conversion complete

#define PRIO_IRQ_USB                9      ///< USB device - middle priority

#define PRIO_IRQ_CURRENT_SPI        10      ///< Not used
#define PRIO_IRQ_KP_TIMER           11      ///< Sampling timer for KP and timeout
#define PRIO_IRQ_KP_TIMER_FAKE      12      ///< Timer for Fake KP and freerunning


#define PRIO_LED_BLINKING           14      ///< LED blinking timer

#define PRIO_IRQ_RTC_WAKEUP         14
#define PRIO_IRQ_SYSTEM_PVD         14
#define PRIO_IRQ_DIAG_ADC           14
#define PRIO_IRQ_MODBUS_TIMER       14
#define PRIO_IRQ_MODBUS_UART        14


/* RTOS-related IRQ */
#define PRIO_IRQ_PENDSV             15
#define PRIO_IRQ_SYSTICK            15


/**
 * Timers assignment
 *
 * TIM1 -
 * TIM2 -
 * TIM3 -
 * TIM4 - LED PWM
 * TIM5 - System free running timer
 * TIM6 - Din adc sampling timer
 * TIM7 - KP sampling timer for oscillogram
 * TIM8 -
 * TIM12-
 * TIM13- Modbus timeout timer
 * TIM14-
 * TIM15- KP fake and free running timer
 * TIM16- KP supervisor timer
 * TIM17- LED blinking
 *
 */


/* Macros ------------------------------------------------------------------*/

/**
 * Minimum of two arguments
 */
#ifndef MIN
#define MIN(a, b)   (((a)>(b))?(b):(a))
#endif

/**
 * Maximum of two arguments
 */
#ifndef MAX
#define MAX(a, b)   (((a)<(b))?(b):(a))
#endif

/**
 * Saturate the X to the given upper bound VAL
 */
#define SAT_UP(x, val)      ((x) = ((x)>(val))?(val):(x))

/**
 * Saturate the X to the given lower bound VAL
 */
#define SAT_DOWN(x, val)    ((x) = ((x)<(val))?(val):(x))

/*
 * Debug defines and macros follow
 */

/**
 * General printf-like definition
 */
#define PRINTF(...)
//#define PRINTF(...) printf(__VA_ARGS__)

/**
 * Parameters assertion. If the expr is false, the warning message is printed.
 */
//#define ASSERT_PARAM(expr) ((expr)?((void)0):PRINTF("ASSERT: line [%d] file [%s] \n\r",__LINE__,__FILE__))
#define ASSERT_PARAM(expr)
/**
 * Common Error message
 */
//#define ERR_PRINT(ret, code)  EventMngr_Error(code, (((uint32_t)ret) << 16) | (__LINE__ & 0x0000ffff))
#define ERR_PRINT(ret, code)  PRINTF("ERR: code [%d] line [%d] file [%s] \n\r",code,__LINE__,__FILE__)

/**
 * Error catching macro. If retValue is non-zero, the error message is printed with given errorCode.
 */
#define CATCH_ERROR(retValue, errorCode)  \
    do{\
  if ((retValue) != 0)\
  {\
    ERR_PRINT(retValue, errorCode);\
  }\
    }while(0)


#ifndef __weak
    #define __weak   __attribute__((weak))
#endif /* __weak */


#define TICK_EXPIRED(a) (HAL_GetTick() - (a) < 0x7fffffff)

#define GET_BYTE_0(a)       ( (uint8_t) ((a) & 0xff))
#define GET_BYTE_1(a)       ( (uint8_t) (((a) >> 8) & 0xff))
#define GET_BYTE_2(a)       ( (uint8_t) (((a) >> 16) & 0xff))
#define GET_BYTE_3(a)       ( (uint8_t) (((a) >> 24) & 0xff))


//#define Error_Handler() _Error_Handler(__FILE__, __LINE__)


/* Typedefs-------------------------------------------------------------------*/

/**
 * General system pointer to function type
 */
typedef  void (*System_Callback_t)(void);

/**
 * General status return type
 */
typedef int16_t Status_t;

/* Functions -----------------------------------------------------------------*/

void _Error_Handler(char *file, int line);

#endif /* APPLICATION_COMMON_H_ */
/** @} */
