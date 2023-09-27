#ifndef OLED_I2C_H
#define OLED_I2C_H

/**
  Section: Included Files
*/

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif

// I2C アドレス
#define LCD_dev_addr 0x78	// OLEDアドレス

int OLED_i2c_init(void);
int OLED_i2c_clear(void);
int LCD_i2C_cmd(unsigned char cmd);
int LCD_i2C_data(char *str);
int LCD_clear_pos(unsigned char cmd);


#endif	//OLED_I2C_H
/**
 End of File
*/

