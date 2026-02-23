#include "TI_SPI.h"


void configureSPI(uint32_t base)
{
    // Must put SPI into reset before configuring it
    SPI_disableModule(base);

    // SPI configuration.
    SPI_setConfig(base, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA0,
                  SPI_MODE_MASTER, SPIFSI_CLK_SPEED, 16);
    SPI_disableLoopback(base);
    SPI_setEmulationMode(base, SPI_EMULATION_STOP_MIDWAY);
                               //SPI_EMULATION_FREE_RUN);

    // FIFO disable
    SPI_disableFIFO(base);

    // Interrupt configuration
    SPI_disableInterrupt(base, SPI_INT_RX_DATA_TX_EMPTY);
    SPI_disableInterrupt(base, SPI_INT_RX_OVERRUN);

    // Configuration complete. Enable the module.
    SPI_enableModule(base);
}

void configureSPI_GPIO(void)
{
    // SPIA-SIMO
    GPIO_setMasterCore(GD_SPISIMO_GPIO, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GD_SPISIMO_GPIO_CFG);

    // SPIA-SOMI
    GPIO_setMasterCore(GD_SPISOMI_GPIO, GPIO_CORE_CPU1);
    GPIO_setQualificationMode(GD_SPISOMI_GPIO, GPIO_QUAL_3SAMPLE);
    GPIO_setPinConfig(GD_SPISOMI_GPIO_CFG);

    // SPIA-CLK
    GPIO_setMasterCore(GD_SPICLK_GPIO, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GD_SPICLK_GPIO_CFG);

    // SPIA-STE
    GPIO_setMasterCore(GD_SPISTE_GPIO, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GD_SPISTE_GPIO_CFG);

    return;
}

void GD_Init_LED_blink(uint16_t imax)
{
    uint16_t i;

    for (i=0; i<imax; i++)
    {
        GPIO_writePin(34, 1);
        DEVICE_DELAY_US(100000);
        GPIO_writePin(34, 0);
        DEVICE_DELAY_US(100000);
    }
    DEVICE_DELAY_US(10000);

    return;
}