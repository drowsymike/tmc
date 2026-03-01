#include "main.h"
#include "stm32f4xx_it.h"
#include <string.h>

extern UART_HandleTypeDef huart2;

uint8_t rx_byte = 0;
uint8_t rx_buffer[64] = {0};
uint8_t rx_buffer_index = 0;
volatile uint8_t command_ready = 0;
uint16_t counter = 0;
extern bool temperature_check_flag;

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
  if (counter == 1000) {
    temperature_check_flag = true;
    counter = 0;
  } else {
    counter++;
  }
}

void USART2_IRQHandler(void) {
  HAL_UART_IRQHandler(&huart2);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) 
  {
    if (rx_byte == '\n' || rx_byte == '\r') 
    {
      rx_buffer[rx_buffer_index] = '\0';
      command_ready = true;
      rx_buffer_index = 0;
    }
    else if (rx_buffer_index < 63) 
    {
      rx_buffer[rx_buffer_index++] = rx_byte;
    }
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  }
}

/*
  callback
  receiving the message

*/