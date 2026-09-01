#include "TI_UART.h"

void TI_UART_SCIB_GPIO_Init(void)
{
    //
    // SCIB UART pins on LAUNCHXL_F28P65X:
    // RX = GPIO55
    // TX = GPIO38
    //
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCIRXDB);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCITXDB);

    GPIO_setPadConfig(DEVICE_GPIO_PIN_SCIRXDB, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_SCITXDB, GPIO_PIN_TYPE_STD);

    GPIO_setQualificationMode(DEVICE_GPIO_PIN_SCIRXDB, GPIO_QUAL_ASYNC);
    GPIO_setQualificationMode(DEVICE_GPIO_PIN_SCITXDB, GPIO_QUAL_ASYNC);
}

void TI_UART_Init(uint32_t base, uint32_t baudrate)
{
    SCI_setConfig(base,
                  DEVICE_LSPCLK_FREQ,
                  baudrate,
                  SCI_CONFIG_WLEN_8 |
                  SCI_CONFIG_STOP_ONE |
                  SCI_CONFIG_PAR_NONE);

    SCI_resetChannels(base);

    SCI_enableFIFO(base);
    SCI_resetTxFIFO(base);
    SCI_resetRxFIFO(base);

    SCI_clearInterruptStatus(base,
                             SCI_INT_RXFF |
                             SCI_INT_TXFF |
                             SCI_INT_RXERR);

    SCI_enableModule(base);
    SCI_performSoftwareReset(base);
}

void TI_UART_Send(uint16_t frame_amount, uint16_t *data)
{
    uint16_t i;

    for(i = 0U; i < frame_amount; i++)
    {
        SCI_writeCharBlockingFIFO(TI_UART_SCIB_BASE, data[i] & 0x00FFU);
    }
}

bool TI_UART_Read(uint16_t frame_amount, uint16_t *data)
{
    uint16_t i;

    if(SCI_getRxFIFOStatus(TI_UART_SCIB_BASE) < frame_amount)
    {
        return false;
    }

    for(i = 0U; i < frame_amount; i++)
    {
        data[i] = SCI_readCharBlockingFIFO(TI_UART_SCIB_BASE) & 0x00FFU;
    }

    return true;
}