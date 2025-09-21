#include "SPI.h"

SPI_HandleTypeDef spi_handle;

void SPI2_Init()
{
    spi_handle.Instance = SPI2;
    spi_handle.Init.Mode = SPI_MODE_MASTER;
    spi_handle.Init.Direction = SPI_DIRECTION_2LINES;
    spi_handle.Init.DataSize = SPI_DATASIZE_8BIT;
    spi_handle.Init.CLKPolarity = SPI_POLARITY_HIGH;
    spi_handle.Init.CLKPhase = SPI_PHASE_2EDGE;
    spi_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    spi_handle.Init.FirstBit = SPI_FIRSTBIT_MSB;

    spi_handle.Init.NSS = SPI_NSS_SOFT;
    spi_handle.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    spi_handle.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    
    spi_handle.Init.TIMode = SPI_TIMODE_DISABLE;
    spi_handle.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;

    spi_handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spi_handle.Init.CRCPolynomial = 0x0;
    spi_handle.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    spi_handle.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    
    spi_handle.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
    spi_handle.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    spi_handle.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    spi_handle.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    spi_handle.Init.IOSwap = SPI_IO_SWAP_DISABLE;
    
    HAL_SPI_Init(&spi_handle);
}


void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef gpio_initstruct;
    RCC_PeriphCLKInitTypeDef periph_initstruct;

    if(hspi->Instance == SPI2)
    {
        periph_initstruct.PeriphClockSelection = RCC_PERIPHCLK_SPI2;
        periph_initstruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
        HAL_RCCEx_PeriphCLKConfig(&periph_initstruct);

        __HAL_RCC_SPI2_CLK_ENABLE();
        GPIOB_CLK();
        GPIOC_CLK();

        gpio_initstruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
        gpio_initstruct.Mode = GPIO_MODE_AF_PP;
        gpio_initstruct.Pull = GPIO_NOPULL;
        gpio_initstruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio_initstruct.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(GPIOC, &gpio_initstruct);

        gpio_initstruct.Pin = GPIO_PIN_13;
        gpio_initstruct.Mode = GPIO_MODE_AF_PP;
        gpio_initstruct.Pull = GPIO_NOPULL;
        gpio_initstruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio_initstruct.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(GPIOB, &gpio_initstruct);
    }

}




