#include "main.h"
#include "stm32f4xx_it.h"
#include "cmsis_armcc.h"

void NMI_Handler(void)
{
   while (1)
  {
  }
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {

  }
}

void SVC_Handler(void)
{

}

void DebugMon_Handler(void)
{

}

void PendSV_Handler(void)
{

}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void USART1_IRQHandler(void) {
  if (USART1->SR & USART_SR_RXNE) {
    uint8_t received_byte = (uint8_t)(USART1->DR);
    process_data(received_byte);
  }
}