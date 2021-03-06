/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : FSMC.c
  * Description        : This file provides code for the configuration
  *                      of the FSMC peripheral.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "fsmc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SRAM_HandleTypeDef hsram1;

/* FSMC initialization function */
void MX_FSMC_Init(void)
{
    /* USER CODE BEGIN FSMC_Init 0 */

    /* USER CODE END FSMC_Init 0 */

    FSMC_NORSRAM_TimingTypeDef Timing = {0};

    /* USER CODE BEGIN FSMC_Init 1 */

    /* USER CODE END FSMC_Init 1 */

    /** Perform the SRAM1 memory initialization sequence
    */
    hsram1.Instance = FSMC_NORSRAM_DEVICE;
    hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
    /* hsram1.Init */
    hsram1.Init.NSBank = FSMC_NORSRAM_BANK4;
    hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
    hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
    hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_8;
    hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
    hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
    hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
    hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
    hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
    hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
    hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
    hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
    hsram1.Init.ContinuousClock = FSMC_CONTINUOUS_CLOCK_SYNC_ONLY;
    hsram1.Init.WriteFifo = FSMC_WRITE_FIFO_ENABLE;
    hsram1.Init.PageSize = FSMC_PAGE_SIZE_NONE;
    /* Timing */
    Timing.AddressSetupTime = 0;
    Timing.AddressHoldTime = 15;
    Timing.DataSetupTime = 2;
    Timing.BusTurnAroundDuration = 2;
    Timing.CLKDivision = 16;
    Timing.DataLatency = 17;
    Timing.AccessMode = FSMC_ACCESS_MODE_A;
    /* ExtTiming */

    if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
    {
        Error_Handler( );
    }

    /* USER CODE BEGIN FSMC_Init 2 */

    /* USER CODE END FSMC_Init 2 */
}

static uint32_t FSMC_Initialized = 0;

static void HAL_FSMC_MspInit(void) {
    /* USER CODE BEGIN FSMC_MspInit 0 */

    /* USER CODE END FSMC_MspInit 0 */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (FSMC_Initialized) {
        return;
    }
    FSMC_Initialized = 1;

    /* Peripheral clock enable */
    __HAL_RCC_FSMC_CLK_ENABLE();

    /** FSMC GPIO Configuration
    PC2   ------> FSMC_NWE
    PC3   ------> FSMC_A0
    PA2   ------> FSMC_D4
    PA3   ------> FSMC_D5
    PA4   ------> FSMC_D6
    PA5   ------> FSMC_D7
    PC4   ------> FSMC_NE4
    PC5   ------> FSMC_NOE
    PB14   ------> FSMC_D0
    PC6   ------> FSMC_D1
    PC11   ------> FSMC_D2
    PC12   ------> FSMC_D3
    */
    /* GPIO_InitStruct */
    GPIO_InitStruct.Pin = LCD_WR_Pin|LCD_RD_Pin|LCD_CS_Pin|LCD_NOE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;

    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* GPIO_InitStruct */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* GPIO_InitStruct */
    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;

    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* GPIO_InitStruct */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;

    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USER CODE BEGIN FSMC_MspInit 1 */

    /* USER CODE END FSMC_MspInit 1 */
}

void HAL_SRAM_MspInit(SRAM_HandleTypeDef* sramHandle) {
    /* USER CODE BEGIN SRAM_MspInit 0 */

    /* USER CODE END SRAM_MspInit 0 */
    HAL_FSMC_MspInit();
    /* USER CODE BEGIN SRAM_MspInit 1 */

    /* USER CODE END SRAM_MspInit 1 */
}

static uint32_t FSMC_DeInitialized = 0;

static void HAL_FSMC_MspDeInit(void) {
    /* USER CODE BEGIN FSMC_MspDeInit 0 */

    /* USER CODE END FSMC_MspDeInit 0 */
    if (FSMC_DeInitialized) {
        return;
    }
    FSMC_DeInitialized = 1;
    /* Peripheral clock enable */
    __HAL_RCC_FSMC_CLK_DISABLE();

    /** FSMC GPIO Configuration
    PC2   ------> FSMC_NWE
    PC3   ------> FSMC_A0
    PA2   ------> FSMC_D4
    PA3   ------> FSMC_D5
    PA4   ------> FSMC_D6
    PA5   ------> FSMC_D7
    PC4   ------> FSMC_NE4
    PC5   ------> FSMC_NOE
    PB14   ------> FSMC_D0
    PC6   ------> FSMC_D1
    PC11   ------> FSMC_D2
    PC12   ------> FSMC_D3
    */

    HAL_GPIO_DeInit(GPIOC, LCD_WR_Pin|LCD_RD_Pin|LCD_CS_Pin|LCD_NOE_Pin
                    |GPIO_PIN_6|GPIO_PIN_11|GPIO_PIN_12);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_14);

    /* USER CODE BEGIN FSMC_MspDeInit 1 */

    /* USER CODE END FSMC_MspDeInit 1 */
}

void HAL_SRAM_MspDeInit(SRAM_HandleTypeDef* sramHandle) {
    /* USER CODE BEGIN SRAM_MspDeInit 0 */

    /* USER CODE END SRAM_MspDeInit 0 */
    HAL_FSMC_MspDeInit();
    /* USER CODE BEGIN SRAM_MspDeInit 1 */

    /* USER CODE END SRAM_MspDeInit 1 */
}
/**
  * @}
  */

/**
  * @}
  */
