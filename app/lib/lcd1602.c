#include <main.h>
#include "lcd1602.h"
#include <string.h>

extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef i2c;

uint16_t lcd1604_addr = 0x27 << (uint16_t)1;

void lcd1602_init(void) {	
  HAL_Delay(50);
  lcd1602_transmit_command(0x33);
  HAL_Delay(5);
  lcd1602_transmit_command(0x32);
  HAL_Delay(1);
  lcd1602_transmit_command(0x28);
  HAL_Delay(1);
  lcd1602_transmit_command(0x08);
  HAL_Delay(1);
  lcd1602_transmit_command(0x01);
  HAL_Delay(2);
  lcd1602_transmit_command(0x06);
  HAL_Delay(1);
  lcd1602_transmit_command(0x0C);
}

void lcd1602_send_string(char *str) {
  if (sizeof(*str) == 0) {
    HAL_UART_Transmit(&huart2, (const uint8_t*)"Warning: in the send function 'lcd1602_send_string' you pass an empty string\n", \
    strlen((char *)"Warning: in the send function 'lcd1602_send_string' you pass an empty string\n"), HAL_MAX_DELAY);
  }
  while(*str) {
    lcd1602_transmit_data((uint8_t)(*str));
    str++;
  }
}

HAL_StatusTypeDef lcd1602_transmit(uint8_t data, uint8_t flags) {
	uint8_t upper_bits = data & 0xF0;
	uint8_t lower_bits = (data << 4) & 0xF0;
	uint8_t data_arr[4];                              
	data_arr[0] = upper_bits | flags | BACKLIGHT | PIN_EN; 
	data_arr[1] = upper_bits | flags | BACKLIGHT;         
	data_arr[2] = lower_bits | flags | BACKLIGHT | PIN_EN;
	data_arr[3] = lower_bits | flags | BACKLIGHT;
	if(HAL_I2C_Master_Transmit(&i2c, lcd1604_addr, data_arr, sizeof(data_arr), HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
  } else {
        return HAL_OK;
  }
}

void lcd1602_transmit_data(uint8_t data) {
	lcd1602_transmit(data, PIN_RS);
}

void lcd1602_transmit_command(uint8_t cmd) {
	lcd1602_transmit(cmd, 0);
}