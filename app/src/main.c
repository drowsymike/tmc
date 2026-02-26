#include "stm32f4xx_hal.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include "bmp180.h"
#include "lcd1602.h"

#define coil_prescaler 15999
#define coil_max_period 9999
#define coil_max_pulse 0

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

float Kp = 2.0;
float Ki = 0.5;
float Kd = 1.0;
float integral = 0;
float prev_error = 0;

TIM_HandleTypeDef htim3;

bool temperature_check_flag = false;

float compute_pid(float target, float current) {
  float error = target - current;
  
  // P
  float P_out = Kp * error;
  
  // I (с защитой от переполнения)
  integral += error;
  if (integral > 50) integral = 50;   // Чуть уменьшим лимиты для стабильности
  if (integral < -50) integral = -50; 
  float I_out = Ki * integral;
  
  // D
  float D_out = Kd * (error - prev_error);
  prev_error = error;
  
  float output = P_out + I_out + D_out;
  
  // Ограничиваем мощность 0-100%
  if (output > 100) output = 100;
  if (output < 0) output = 0;
  
  return output;
}

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
  MX_USART2_UART_Init();
  i2c_init();
  pwm_timer_init();
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  set_pwm(80);
  float current_temp = 0;

  while (1)
  {
    if (temperature_check_flag == true) {
      temperature_check_flag = false;
      
      current_temp = bmp180_get_temp();
      
      float power = compute_pid((float)target_temperature, current_temp);
      
      // ПРИМЕНЯЕМ ПИД к железу
      set_pwm((uint8_t)power);
      
      // Вывод на дисплей для контроля
      snprintf((char*)msg, sizeof(msg), "T:%2.1f P:%d%%", current_temp, (int)power);
      lcd1602_transmit_command(0b10000000);
      lcd1602_send_string((char*)msg);
    }
    
    if (command_ready == 1) {
      memcpy(cmd_tmp, rx_buffer, 3);
      cmd_tmp[3] = '\0';
      if (strcmp((const char *)cmd_tmp, (const char *)set_temperature_cmd) == 0) {
        int parsed_count = sscanf((char*)rx_buffer, "set %hu", &target_temperature);
        if (parsed_count == 1) {
          set_pwm(target_temperature);
        } else {
          HAL_UART_Transmit(&huart2, (const uint8_t *)"Unknown command\n", strlen((char*)"Unknown command\n"), 10);
        }
      } else if (strcmp((const char *)cmd_tmp, (const char *)turn_off_cmd) == 0) {
        HAL_UART_Transmit(&huart2, (const uint8_t *)"shutting down...\n", strlen((char*)"shutting down...\n"), 10);
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
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = coil_prescaler; 
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = coil_max_period; 
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&htim3);

  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = coil_max_pulse;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
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
  //period is 100%
  if (filling > 100) {
    return 1;
  }
  //uint32_t compare_value = 100 - (filling * ((float)coil_max_period * 0.01)); 
  uint32_t compare_value = (100 - filling) * ((float)coil_max_period * 0.01);
  if (compare_value > coil_max_period) compare_value = coil_max_period;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, compare_value);
  return 0;
}

void _init(void) {

}
