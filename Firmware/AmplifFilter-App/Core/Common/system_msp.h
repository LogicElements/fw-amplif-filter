/**
  * @file       system_msp.h
  * @version    1.0
  * @date       07-12-2015
  * @brief      System MCU-specific package definitions
  *
  * @copyright  Logic Elements Copyright
  *
  * @defgroup grSystemMsp System MSP (platform dependent)
  * @{
  * @brief System MCU-specific package
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
#ifndef SYSTEM_MSP_H_
#define SYSTEM_MSP_H_

/* Includes ------------------------------------------------------------------*/

#include "common.h"

/* Definitions----------------------------------------------------------------*/


#define SYSTEM_TIMER_COUNT      11

#define SYSTEM_TIMER_KP      10

#define SYSTEM_TIMER_FREQ       10000000   // 10 MHz


/* Typedefs-------------------------------------------------------------------*/

/**
 * General system pointer to function type
 */
typedef  void (*System_Callback_t)(void);


/* Functions -----------------------------------------------------------------*/


/**
 * Configure the system clocks.
 */
void SystemClock_Config(void);

/**
 * Configure the system clocks after wake-up from Stop mode (wake-up from Sleep mode does not need clock configuration).
 */
void System_ClockConfigStopMode(void);


/**
 * Start application at specified address
 *
 * This function is used by bootloaders and reset managers to jump to
 * an application image located at the specified address.
 * Before jumping this function disables all interrupts, sets Stack Pointer,
 * sets the Reset Vector, and jumps to the application address + 4.
 * @param address Beginning address of the application to run
 */
void System_StartApplication(uint32_t address);

/**
 * Software reset.
 */
void System_Reset(void);

/**
 * Refresh independent watchdog
 * @return Status
 */
Status_t System_ReloadIwdg(void);


void System_InitTimer(void);

void System_StartTimer(uint32_t id);

uint32_t System_StopTimer(uint32_t id);

uint32_t System_RestartTimer(uint32_t id);

uint32_t System_GetFreeRunCount(void);

/**
 * Initialize Voltage detector
 * @return Status
 */
Status_t System_PvdInit(void);

/**
 * Unlock Flash interface for programming.
 */
Status_t System_FlashEnable(void);

/**
 * Lock Flash interface for programming
 */
Status_t System_FlashDisable(void);

/**
 * Program a bunch of data into Flash memory
 * @param address Address in Flash memory where the data should be stored
 * @param data Pointer to the data to write to Flash
 * @param dataLength Length of the data to write
 * @return Status
 */
int16_t System_FlashProgram(uint32_t address, uint8_t *data, uint32_t dataLength);

/**
 * Erase the selected Flash sector
 * @param[in] startAddr Start address of the memory to erase
 * @param[in] endAddr End address of the memory to erase
 * @return HAL status
 */
int16_t System_FlashErase(uint32_t startAddr, uint32_t endAddr);


/**
 * Check if the flash is empty
 * @param address Start address to check
 * @param size Size of the flash to check
 * @return Result (OK - empty, ERROR - not empty)
 */
Status_t System_IsFlashNotEmpty(uint32_t *address, uint32_t size);


Status_t System_VerifyImage(uint32_t *address);

/**
 * System delay based on system tick.
 * @param miliseconds Delay in msec.
 */
void System_Delay(uint32_t miliseconds);

/**
 * Generate a short delay by decrementing volatile variable
 * @param iterations
 */
void System_NanoDelay(uint16_t iterations);

/**
 * Get the current system tick count
 * @return System tick count
 */
uint32_t System_GetTick(void);

/**
 * Serial wire viewer print
 * @param data Pointer to data to print via SWV
 * @param length Length of data to print
 * @param port SWV port to be used
 */
void System_SwvPrint(uint8_t* data, uint16_t length, uint8_t port);

/**
 * Initialize CRC unit. Stm32F4 has no CRC settings
 */
void System_CrcInit(void);

/**
 * Clear CRC computation.
 */
void System_CrcClear(void);

/**
 * Calculate CRC computation.
 * @param data Pointer to data to calculate
 * @param length Length of the data
 * @return
 */
uint32_t System_CrcAccumulate(uint32_t *data, uint32_t length);

/**
 * Get random number using RNG
 * @return Random number
 */
uint32_t System_GetRandomNumber(void);

#endif /* SYSTEM_MSP_H_ */

/** @} */

