#include "stm32f4xx_hal.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include "bmp180.h"

UART_HandleTypeDef huart2;
I2C_HandleTypeDef i2c;

uint8_t cmd_tmp[4] = {0};

bool set_temperature_cmd_flag = false;
bool turn_off_cmd_flag = false;
bool info_cmd_flag = false;
bool data_incoming = false;
uint16_t target_temperature = 0;

uint8_t msg[16] = {0};

extern uint8_t rx_byte;
extern uint8_t rx_buffer[64];
extern uint8_t rx_buffer_index;
extern uint8_t command_ready;

TIM_HandleTypeDef htim3;

/*--Function headers for STM32-------------------------------------------------*/

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_USART2_UART_Init(void);
void i2c_init(void);

void pwm_timer_init(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  i2c_init();
  pwm_timer_init();
  bmp180_struct_init();
  bmp180_init();
  bmp180_get_global_coefficients();
  float res = bmp180_get_temp();
  if (res == 0.0) {
    HAL_UART_Transmit(&huart2, (uint8_t*)"zero\n", 10, 10);
  } else {
    snprintf((char*)msg, sizeof(msg), "%f", res);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg), 100);
  }
  /*HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 10);
  HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 10);*/
  //HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg), 100);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);

  while (1)
  {
    if (command_ready == 1) {
      /*HAL_UART_Transmit(&huart2, rx_buffer, strlen((char*)rx_buffer), 100);
      HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 10);*/

      memcpy(cmd_tmp, rx_buffer, 3);
      cmd_tmp[3] = '\0';
      
      /*HAL_UART_Transmit(&huart2, cmd_tmp, strlen((char*)cmd_tmp), 100);
      HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 10);*/

      if (strcmp((const char *)cmd_tmp, (const char *)set_temperature_cmd) == 0) {
        //HAL_UART_Transmit(&huart2, (const uint8_t *)"temperature changing\n", strlen((char*)"temperature changing\n"), 10);
        int parsed_count = sscanf((char*)rx_buffer, "set %hu", &target_temperature);
        if (parsed_count == 1) {
          //HAL_UART_Transmit(&huart2, (const uint8_t *)"setting is done\n", strlen((char*)"setting is done\n"), 10);
          //setting, actually
          set_pwm(target_temperature);
        } else {
          HAL_UART_Transmit(&huart2, (const uint8_t *)"Unknown command\n", strlen((char*)"Unknown command\n"), 10);
        }
        //calling the function of changing the PWM filling
      } else if (strcmp((const char *)cmd_tmp, (const char *)turn_off_cmd) == 0) {
        HAL_UART_Transmit(&huart2, (const uint8_t *)"shutting down...\n", strlen((char*)"shutting down...\n"), 10);
        //calling the function of disabling the PWM
      } else {
        HAL_UART_Transmit(&huart2, (const uint8_t *)"Unknown command\n", strlen((char*)"Unknown command\n"), 10);
      }

      rx_buffer_index = 0;
      memset(rx_buffer, 0, 64);
      command_ready = 0;
    }
  }
}

/**
 * @brief function for init the TIM3 in
 * mode of PWM
 * 
 */
void pwm_timer_init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8399;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&htim3);
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);
  HAL_TIM_PWM_Init(&htim3);
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
  HAL_TIM_MspPostInit(&htim3);
}

void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

void i2c_init(void)
{
  i2c.Instance = I2C1;
  i2c.Init.ClockSpeed = 100000;
  i2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
  i2c.Init.OwnAddress1 = 0;
  i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  i2c.Init.OwnAddress2 = 0;
  i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&i2c) == HAL_OK) {
    //HAL_UART_Transmit(&huart2, (const uint8_t *)"fine\n", strlen((char *)"fine\n"), HAL_MAX_DELAY);
  }
}

void MX_GPIO_Init(void)
{

}

int set_pwm(uint8_t filling) {
  if (filling > 100) return 1;
  uint32_t compare_value = (999 * (uint32_t)filling) / 100; 
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, compare_value);
  return 0;
}

void _init(void) {

}
