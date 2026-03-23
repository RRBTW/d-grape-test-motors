/* ============================================================
 *  stm32f4xx_it.c — Обработчики прерываний (bare metal, без FreeRTOS)
 * ============================================================ */

#include "main.h"
#include "stm32f4xx_it.h"

void NMI_Handler(void) { while (1) {} }

void HardFault_Handler(void)
{
    GPIOE->BSRR = GPIO_PIN_1;
    GPIOD->BSRR = (uint32_t)GPIO_PIN_12 << 16U;
    while (1) {}
}

void MemManage_Handler(void)
{
    GPIOE->BSRR = GPIO_PIN_1;
    GPIOD->BSRR = GPIO_PIN_12;
    while (1) {}
}

void BusFault_Handler(void)
{
    GPIOE->BSRR = GPIO_PIN_1;
    GPIOD->BSRR = GPIO_PIN_13;
    while (1) {}
}

void UsageFault_Handler(void)
{
    GPIOE->BSRR = GPIO_PIN_1;
    GPIOD->BSRR = GPIO_PIN_14;
    while (1) {}
}

void SysTick_Handler(void) { HAL_IncTick(); }

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
void OTG_FS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}
