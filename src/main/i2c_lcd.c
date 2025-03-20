/** Put this in the src folder **/

#include "i2c_lcd.h"

static const char *TAG = "LCD";

esp_err_t err;

i2c_master_bus_handle_t i2c_bus_handle;
i2c_master_dev_handle_t i2c_dev_handle;

esp_err_t i2c_master_init(void)
{
    i2c_master_bus_config_t i2c_master_config =
    {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    

    err = i2c_new_master_bus(&i2c_master_config, &i2c_bus_handle);
    if (err != ESP_OK) 
    {
        ESP_LOGI(TAG, "Error configuring I2C Master: %s", esp_err_to_name(err));
        return err;
    }

    i2c_device_config_t dev_cfg =
    {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LCD_I2C_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    err = i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &i2c_dev_handle);
    if (err != ESP_OK) 
    {
        ESP_LOGI(TAG, "Error configuring I2C Master: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

void lcd_send_cmd(char cmd)
{
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (cmd & 0xf0);
    data_l = ((cmd << 4) & 0xf0);
    
    // en=1, rs=0
    data_t[0] = data_u | 0x0C;
    
    // en=0, rs=0
    data_t[1] = data_u | 0x08;
    
    // en=1, rs=0
    data_t[2] = data_l | 0x0C;
    
    // en=0, rs=0
    data_t[3] = data_l | 0x08;
    
    // Send command using new I2C API
    err = i2c_master_transmit(i2c_dev_handle, data_t, sizeof(data_t), -1);
    if (err != ESP_OK) 
    {
        ESP_LOGI(TAG, "Error in sending command: %s", esp_err_to_name(err));
    }
}

void lcd_send_data(char data)
{
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (data & 0xf0);
    data_l = ((data << 4) & 0xf0);
    
    // en=1, rs=0
    data_t[0] = data_u | 0x0D;
    
    // en=0, rs=0
    data_t[1] = data_u | 0x09;
    
    // en=1, rs=0
    data_t[2] = data_l | 0x0D;
    
    // en=0, rs=0
    data_t[3] = data_l | 0x09;
    
    err = i2c_master_transmit(i2c_dev_handle, data_t, sizeof(data_t), -1);
    if (err != ESP_OK) 
    {
        ESP_LOGI(TAG, "Error in sending command: %s", esp_err_to_name(err));
    }
}

void lcd_clear(void)
{
    lcd_send_cmd(0x01);
    usleep(5000);
}

void lcd_put_cur(int row, int col)
{
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }

    lcd_send_cmd(col);
}

void lcd_init(void)
{
    // 4 bit initialisation
    // wait for >40ms
    usleep(50000);
    lcd_send_cmd(0x30);
    
    // wait for >4.1ms
    usleep(5000);
    lcd_send_cmd(0x30);
    
    // wait for >100us
    usleep(200);
    lcd_send_cmd(0x30);
    usleep(10000);
    
    // 4bit mode
    lcd_send_cmd(0x20);
    usleep(10000);

    // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
    lcd_send_cmd(0x28);
    usleep(1000);
    
    // Display on/off control --> D=0,C=0, B=0  ---> display off
    lcd_send_cmd(0x08);
    usleep(1000);
    
    // clear display
    lcd_send_cmd(0x01);
    usleep(1000);
    usleep(1000);
    
    // Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
    lcd_send_cmd(0x06);
    usleep(1000);
    
    // Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
    lcd_send_cmd(0x0C);
    usleep(1000);
}

void lcd_send_string(char *str)
{
    while (*str) lcd_send_data(*str++);
}
