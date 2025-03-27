#ifndef I2C_LCD_H
#define I2C_LCD_H

#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "unistd.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t i2c_master_init(void);
void lcd_init(void);
void lcd_send_cmd(char cmd);
void lcd_send_data(char data);
void lcd_send_string(char *str);
void lcd_put_cur(int row, int col);
void lcd_clear(void);
void lcd_clear_row(int row);
void lcd_write_string(int row, const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif