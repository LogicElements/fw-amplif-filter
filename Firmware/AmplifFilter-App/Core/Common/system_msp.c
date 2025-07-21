/**
 * @file       system_msp.c
 * @brief      System MCU-specific package implementation
 * @addtogroup grSystemMsp
 * @{
 */

/* Includes ------------------------------------------------------------------*/
#include "system_msp.h"
#include "configuration.h"

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/

#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08020000) /* Base @ of Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08040000) /* Base @ of Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x08060000) /* Base @ of Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08080000) /* Base @ of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x080A0000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x080C0000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x080E0000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08100000) /* Base @ of Sector 8, 128 Kbytes */

#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */


/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


static CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htimTimer;

//IWDG_HandleTypeDef hiwdg;

uint32_t timerTime[SYSTEM_TIMER_COUNT];

/* Private function prototypes -----------------------------------------------*/

static uint32_t System_GetSector(uint32_t address);

/* Public function prototypes -----------------------------------------------*/
/**
 * Weak prototype of user hook function on entering low-power Stop mode.
 */
__weak void System_EnterStopMode(void);

/**
 * Weak prototype of user hook function on exiting low-power Stop mode.
 */
__weak void System_ExitStopMode(void);


int16_t System_GetZoneFromTimestamp(uint32_t timestamp);

/* Public functions ----------------------------------------------------------*/


void SystemClock_Config2(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  SCB->VTOR = (uint32_t)CONF_C_APPLICATION_OFFSET;
  __enable_irq();

  /**Supply configuration update enable
   */
//  MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);
//
//   /**Configure the main internal regulator output voltage
//   */
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//
//  while ((PWR->D3CR & (PWR_D3CR_VOSRDY)) != PWR_D3CR_VOSRDY)
//  {
//  }
   /**Configure LSE Drive Capability
   */
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /* Enable backup regulator (meaningful only with battery) */
  __HAL_RCC_BKPRAM_CLK_DISABLE();
  HAL_PWREx_DisableBkUpReg();
  HAL_PWR_EnableBkUpAccess();

   /**Macro to configure the PLL clock source
   */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);


   /**Initializes the CPU, AHB and APB busses clocks
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_LSI
                             |RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 48;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 16;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

   /**Initializes the CPU, AHB and APB busses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                             |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                             |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART1
                             |RCC_PERIPHCLK_UART5|RCC_PERIPHCLK_RNG
                             |RCC_PERIPHCLK_SPI5|RCC_PERIPHCLK_SPI4
                             |RCC_PERIPHCLK_SPI3|RCC_PERIPHCLK_SPI1
                             |RCC_PERIPHCLK_ADC
                             |RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.PLL2.PLL2M = 16;
  PeriphClkInitStruct.PLL2.PLL2N = 200;
  PeriphClkInitStruct.PLL2.PLL2P = 8;
  PeriphClkInitStruct.PLL2.PLL2Q = 4;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.PLL3.PLL3M = 16;
  PeriphClkInitStruct.PLL3.PLL3N = 320;
  PeriphClkInitStruct.PLL3.PLL3P = 4;
  PeriphClkInitStruct.PLL3.PLL3Q = 8;
  PeriphClkInitStruct.PLL3.PLL3R = 6;   // was 4
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_0;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
//  PeriphClkInitStruct.Sai23ClockSelection = RCC_SAI23CLKSOURCE_PLL;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_PLL2;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
  PeriphClkInitStruct.RngClockSelection = RCC_RNGCLKSOURCE_HSI48;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL3;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_PLL1QCLK, RCC_MCODIV_3);

   /* Enable all GPIO ports */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  __HAL_RCC_RNG_CLK_ENABLE();

  __HAL_RCC_DMA1_CLK_ENABLE();

  /* Enable USB power regulator */
  HAL_PWREx_EnableUSBVoltageDetector();

//  hiwdg.Instance = IWDG1;
//  hiwdg.Init.Prescaler = IWDG_PRESCALER_16;
//  hiwdg.Init.Window = 4095;
//  hiwdg.Init.Reload = 4094;
//  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }
  /* Disable IWDG in debug mode */
  DBGMCU->APB4FZ1 = DBGMCU->APB4FZ1 | DBGMCU_APB4FZ1_DBG_IWDG1;

  System_ReloadIwdg();

}






Status_t System_ReloadIwdg(void)
{
  Status_t ret = STATUS_OK;

//  ret = HAL_IWDG_Refresh(&hiwdg);

  return ret;
}



Status_t System_PvdInit(void)
{
  PWR_PVDTypeDef sConfigPVD;

  HAL_NVIC_SetPriority(PVD_IRQn, PRIO_IRQ_SYSTEM_PVD, 0);
  HAL_NVIC_EnableIRQ(PVD_IRQn);

  sConfigPVD.PVDLevel = PWR_PVDLEVEL_7;
  sConfigPVD.Mode = PWR_PVD_MODE_IT_RISING_FALLING;
  HAL_PWR_ConfigPVD(&sConfigPVD);

  HAL_PWR_EnablePVD();

  return STATUS_OK;
}


void PVD_IRQHandler(void)
{
  /* Check PWR Exti flag */
  HAL_PWR_PVD_IRQHandler();
}




void System_InitTimer(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  __TIM5_CLK_ENABLE();

  htimTimer.Instance = TIM5;
  htimTimer.Init.Prescaler = 19;
  htimTimer.Init.CounterMode = TIM_COUNTERMODE_UP;
  htimTimer.Init.Period = 0xffffffff;
  htimTimer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htimTimer);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htimTimer, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htimTimer, &sMasterConfig);
}



void System_StartTimer(uint32_t id)
{
  TIM5->CR1|=(TIM_CR1_CEN);
  timerTime[id] = TIM5->CNT;
}

uint32_t System_StopTimer(uint32_t id)
{
  return TIM5->CNT - timerTime[id];
}

__inline uint32_t System_GetFreeRunCount(void)
{
  return TIM5->CNT;
}

void System_MicroWait(uint32_t us)
{
  uint32_t end = System_GetFreeRunCount() + us;
  while((System_GetFreeRunCount() - (end) > 0x7fffffff))
    ;
}

uint32_t System_RestartTimer(uint32_t id)
{
  uint32_t val = TIM5->CNT;
  uint32_t ret = val - timerTime[id];

  timerTime[id] = val;

  return ret;
}


void System_CrcInit(void)
{
  uint32_t val[] = {0x01, 0x02};
  volatile uint32_t retVal;

  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.Init.CRCLength = CRC_POLYLENGTH_32B;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_WORDS;

  __HAL_RCC_CRC_CLK_ENABLE();
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_CRC_DR_RESET(&hcrc);

  retVal = HAL_CRC_Accumulate(&hcrc, val, 1);

  retVal++;

  __HAL_CRC_DR_RESET(&hcrc);

  retVal = HAL_CRC_Accumulate(&hcrc, val, 2);

  retVal++;

  __HAL_CRC_DR_RESET(&hcrc);
}

void System_CrcClear(void)
{
  /* Reset CRC Calculation Unit */
  __HAL_CRC_DR_RESET(&hcrc);
}

uint32_t System_CrcAccumulate(uint32_t *data, uint32_t length)
{
  return HAL_CRC_Accumulate(&hcrc, data, length);
}

uint32_t System_GetRandomNumber(void)
{
  uint32_t countdown = 40;  // 40 cycles max (see RefMan)
  RNG->CR = 0x04;

  while(((RNG->SR & 0x01) == 0) && countdown != 0)
    countdown--;

  RNG->CR = 0x00;
  RNG->SR = 0x00;
  return RNG->DR;
}




void System_StartApplication(uint32_t address)
{
  uint32_t JumpAddress;
  System_Callback_t jump_to_application;

  /* Prepare Jump address */
  JumpAddress = *(uint32_t*) (address + 4);
  jump_to_application = (System_Callback_t) (JumpAddress);

  /* Disable all interrupts */
  /* + 0 is a workaround of "assignment to itself code check" */
  NVIC->ICER[0] = NVIC->ICER[0] + 0;
  NVIC->ICER[1] = NVIC->ICER[1] + 0;
  NVIC->ICER[2] = NVIC->ICER[2] + 0;

  /* Clear pending interrupts */
  /* + 0 is a workaround of "assignment to itself code check" */
  NVIC->ICPR[0] = NVIC->ICPR[0] + 0;
  NVIC->ICPR[1] = NVIC->ICPR[1] + 0;
  NVIC->ICPR[2] = NVIC->ICPR[2] + 0;

  /* Set the reset vector */
  SCB->VTOR = address;

  /* Initialize application Stack Pointer */
  __set_MSP(*(uint32_t*) address);

  /* Jump to the application finally ;) */
  jump_to_application();
}

void System_Reset(void)
{
  NVIC_SystemReset();
}

Status_t System_FlashEnable(void)
{
  Status_t ret;
  /* Unlock the Program memory */
  ret = HAL_FLASH_Unlock();

  /* Clear all FLASH flags */
  __HAL_FLASH_CLEAR_FLAG_BANK1(FLASH_FLAG_WRPERR | FLASH_FLAG_PGSERR | FLASH_FLAG_INCERR);
  __HAL_FLASH_CLEAR_FLAG_BANK2(FLASH_FLAG_WRPERR | FLASH_FLAG_PGSERR | FLASH_FLAG_INCERR);

  return ret;
}

Status_t System_FlashDisable(void)
{
  Status_t ret;

  ret = HAL_FLASH_Lock();

  return ret;
}


int16_t System_FlashProgram(uint32_t address, uint8_t *data, uint32_t dataLength)
{
  HAL_StatusTypeDef ret = 0;
  uint32_t i;
  uint32_t retries;

  /* The data length must be multiple of 32 bytes */
  if (dataLength < 32 || dataLength % 32)
  {
    ret = -1;
  }
  else
  {
    ret = System_FlashEnable();

    /* For all data to flash */
    for (i = 0; i < dataLength / 32; i++)
    {
      ret = STATUS_ERROR;
      retries = 3;
      while (retries != 0 && ret != 0)
      {
        /* Flash a single 32-bit word */
        __HAL_FLASH_CLEAR_FLAG_BANK1(FLASH_FLAG_WRPERR | FLASH_FLAG_PGSERR | FLASH_FLAG_INCERR);
        __HAL_FLASH_CLEAR_FLAG_BANK2(FLASH_FLAG_WRPERR | FLASH_FLAG_PGSERR | FLASH_FLAG_INCERR);

        ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address + i * 32, (uint64_t) ((uint32_t)data + i * 32));

        if (ret != 0)
        {
          ret++;
        }

        retries--;
      }
    }
    System_FlashDisable();

    /* Check the result of flashing */
    if (memcmp((void*)address, data, dataLength) != 0)
    {
      ret = STATUS_ERROR;
    }
  }

  return (int16_t) ret;
}

int16_t System_FlashErase(uint32_t startAddr, uint32_t endAddr)
{
  HAL_StatusTypeDef ret;
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SectorError = 0;
  uint32_t FirstSector = 0;
  uint32_t NbOfSectors = 0;

  /* Get the First sector ID to erase */
  FirstSector = System_GetSector(startAddr);

  /* Get the number of sectors to erase */
  NbOfSectors = System_GetSector(endAddr) - FirstSector + 1;

  /* Configure the sector erase structure */
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = 0;
  EraseInitStruct.Sector = FirstSector;
  EraseInitStruct.NbSectors = NbOfSectors;

  if (IS_FLASH_PROGRAM_ADDRESS_BANK1(startAddr))
  {
    EraseInitStruct.Banks = FLASH_BANK_1;
  }
  else
  {
    EraseInitStruct.Banks = FLASH_BANK_2;
  }

  /* Enable flash access, erase sector, disable flash access */
  System_FlashEnable();
  ret = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
  System_FlashDisable();

  return (int16_t) ret;
}



Status_t System_VerifyImage(uint32_t *address)
{
  Status_t ret = STATUS_OK;
  uint32_t checksum = *(address + (uint32_t)CONF_FW_INFO_OFFSET / 4 + 2);
  uint32_t calcChecksum;
  uint32_t size = *(address + (uint32_t)CONF_FW_INFO_OFFSET / 4 + 3);

  /* If size is greater than MAX value */
  if (size > (uint32_t)CONF_C_APPLICATION_MAX_SIZE)
  {
    ret = STATUS_ERROR;
  }
  else
  {
    /* If flash is not empty */
    ret = System_IsFlashNotEmpty(address, (uint32_t)size);
  }

  if (ret == STATUS_OK)
  {
    /* Calculate CRC. CRC skips the Firmware information field, uses all 0xFF */
    System_CrcClear();
    System_CrcAccumulate(address, (uint32_t)CONF_FW_INFO_OFFSET / 4);
    System_CrcAccumulate((uint32_t*)CONF_FIRMWARE_INFO_DEFAULT, 8);
    calcChecksum = System_CrcAccumulate(address + (uint32_t)CONF_FW_INFO_OFFSET / 4 + 8, (size - (uint32_t)CONF_FW_INFO_OFFSET - 32) / 4);

    /* If CRC is different */
    if (calcChecksum != checksum)
    {
      ret = STATUS_ERROR;
    }
  }

  return ret;
}



Status_t System_IsFlashNotEmpty(uint32_t *address, uint32_t size)
{
  Status_t ret = STATUS_ERROR;
  uint32_t i = 0;

  /* Go through the given flash */
  while (i < size && ret == STATUS_ERROR)
  {
    /* If there is not a reset value, exit  */
    if (address[i/4] != 0xFFFFFFFF)
    {
      ret = STATUS_OK;
    }
    /* Increment step */
    i += 0x100;
  }

  return ret;
}

void System_SwvPrint(uint8_t* data, uint16_t length, uint8_t port)
{
  int i;

  /* Check if the particular port of SWV is enabled */
  if ((ITM->TCR & ITM_TCR_ITMENA_Msk) && /* ITM enabled */
  (ITM->TER & (1 << port))) /* ITM Port #port enabled */
  {
    /* Print all the data byte by byte */
    for (i = 0; i < length; i++)
    {
      /* Wait on completion of the previous send operation */
      while (ITM->PORT[port].u32 == 0)
        ;

      /* Send the next byte */
      ITM->PORT[port].u8 = *data++;
    }
  }
}


void System_Delay(uint32_t milliseconds)
{
  HAL_Delay(milliseconds);
}


void System_NanoDelay(uint16_t iterations)
{
  volatile uint32_t i = iterations + 1;
  while(i)
  {
    i--;
  }
}

uint32_t System_GetTick(void)
{
  return HAL_GetTick();
}


/* Private functions ----------------------------------------------------------*/

/**
 * @brief  Gets the sector of a given address
 * @param  None
 * @retval The sector of a given address
 */
static uint32_t System_GetSector(uint32_t address)
{
  uint32_t sector = FLASH_SECTOR_7;

  /* Bank 2 ? */
  if (address >= ADDR_FLASH_SECTOR_8)
  {
    address -= 0x00100000;
  }

  if ((address < ADDR_FLASH_SECTOR_1) && (address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_SECTOR_0;
  }
  else if ((address < ADDR_FLASH_SECTOR_2) && (address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_SECTOR_1;
  }
  else if ((address < ADDR_FLASH_SECTOR_3) && (address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_SECTOR_2;
  }
  else if ((address < ADDR_FLASH_SECTOR_4) && (address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_SECTOR_3;
  }
  else if ((address < ADDR_FLASH_SECTOR_5) && (address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_SECTOR_4;
  }
  else if ((address < ADDR_FLASH_SECTOR_6) && (address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_SECTOR_5;
  }
  else if ((address < ADDR_FLASH_SECTOR_7) && (address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_SECTOR_6;
  }
  else if ((address < ADDR_FLASH_SECTOR_8) && (address >= ADDR_FLASH_SECTOR_7))
  {
    sector = FLASH_SECTOR_7;
  }
//  else if((address < ADDR_FLASH_SECTOR_9) && (address >= ADDR_FLASH_SECTOR_8))
//  {
//    sector = FLASH_SECTOR_8;
//  }
//  else if((address < ADDR_FLASH_SECTOR_10) && (address >= ADDR_FLASH_SECTOR_9))
//  {
//    sector = FLASH_SECTOR_9;
//  }
//  else if((address < ADDR_FLASH_SECTOR_11) && (address >= ADDR_FLASH_SECTOR_10))
//  {
//    sector = FLASH_SECTOR_10;
//  }
//  else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11) */
//  {
//    sector = FLASH_SECTOR_11;
//  }

  return sector;
}

/** @} */

