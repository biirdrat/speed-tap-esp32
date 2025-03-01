/** Put this in the src folder **/

#include "i2c_lcd.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "unistd.h"

#define I2C_MASTER_SCL_IO           GPIO_NUM_22
#define I2C_MASTER_SDA_IO           GPIO_NUM_21
#define I2C_MASTER_NUM              0
#define I2C_MASTER_FREQ_HZ          400000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

#define SLAVE_ADDRESS_LCD 0x4E>>1

esp_err_t err;

#define I2C_NUM I2C_NUM_0

static const char *TAG = "LCD";

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void lcd_send_cmd (char cmd)
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
    
    err = i2c_master_write_to_device(I2C_NUM, SLAVE_ADDRESS_LCD, data_t, 4, 1000);
    if (err != 0) ESP_LOGI(TAG, "Error in sending command");
}

void lcd_send_data (char data)
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
    
    err = i2c_master_write_to_device(I2C_NUM, SLAVE_ADDRESS_LCD, data_t, 4, 1000);
    if (err != 0) ESP_LOGI(TAG, "Error in sending data");
}

void lcd_clear (void)
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

void lcd_init (void)
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

void lcd_send_string (char *str)
{
    while (*str) lcd_send_data(*str++);
}
