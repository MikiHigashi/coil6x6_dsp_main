#include <xc.h>
#include <stdio.h>
#include <string.h>

#define FCY 69784687UL
#include <libpic30.h>
#include "soft_i2c.h"
#include "oled_i2c.h"
#include "mcc_generated_files/mcc.h"


// 表示クリアしDDRAMアドレス設定
// 1:タイムアウト 0:正常
int LCD_clear_pos(unsigned char cmd) {
    if (LCD_i2C_cmd(0x01)) return 1; // クリアディスプレイ
    __delay_ms(1);
    if (cmd == 0x80) return 1;
    return LCD_i2C_cmd(cmd);
}


// 文字列 str を表示する
// 1:タイムアウト 0:正常
int LCD_i2C_data(char *str) {
//	unsigned char c;
//	char l;
//	char i;

//	return 0;
}


// ==================== I2C接続LCDにコマンド1つ送信 ===========================
// 1:タイムアウト 0:正常
int LCD_i2C_cmd(unsigned char cmd) {
	I2C_start();
	I2C_send(LCD_dev_addr);	// スレーブアドレス
	if (I2C_ackchk() == 2) return 1;
	I2C_send(0x80);
	if (I2C_ackchk() == 2) return 1;
	I2C_send(cmd);
	if (I2C_ackchk() == 2) return 1;
	I2C_stop();
    return 0;
}


// ==================== I2C接続OLEDの初期化 ===========================
int OLED_i2c_init(void) {
	I2C_start();
	I2C_send(LCD_dev_addr);	// スレーブアドレス
	if (I2C_ackchk() == 2) return 1;
	I2C_send(0);
	if (I2C_ackchk() == 2) return 1;
	I2C_send(0x8D); // Set charge pump
	if (I2C_ackchk() == 2) return 1;
	I2C_send(0x14); // Enable charge pump
	if (I2C_ackchk() == 2) return 1;
	I2C_send(0xAF); // Display ON
	if (I2C_ackchk() == 2) return 1;
	I2C_stop();
    return 0;
}


// ==================== I2C接続OLEDのクリア ===========================
int OLED_i2c_clear(void) {
    uint16_t i;

	I2C_start();
	I2C_send(LCD_dev_addr);	// スレーブアドレス
	if (I2C_ackchk() == 2) return 1;
	I2C_send(0);
	if (I2C_ackchk() == 2) return 1;
	I2C_send(0x20); // Set memory addressing mode
	if (I2C_ackchk() == 2) return 1;
	I2C_send(0x00); // Horizontal addressing mode
	if (I2C_ackchk() == 2) return 1;
	I2C_send(0x21); // Set column address
	if (I2C_ackchk() == 2) return 1;
	I2C_send(0x00); // Column start address 0
	if (I2C_ackchk() == 2) return 1;
	I2C_send(0x7F); // Column end address 127d
	if (I2C_ackchk() == 2) return 1;
	I2C_send(0x22); // Set page address
	if (I2C_ackchk() == 2) return 1;
	I2C_send(0x00); // Page start address 0
	if (I2C_ackchk() == 2) return 1;
	I2C_send(0x07); // Page end address 7d
	if (I2C_ackchk() == 2) return 1;
	I2C_stop();

	I2C_start();
	I2C_send(LCD_dev_addr);	// スレーブアドレス
	if (I2C_ackchk() == 2) return 1;
	I2C_send(0x40); // Control byte Co=0, D/C#=1 (The following data bytes are stored at the GDDRAM)
    for(i=0; i<1024; i++) { // 128culomn * 8page
    	I2C_send(0x88); // filled with 0 (OLED clear)
        if (I2C_ackchk() == 2) return 1;
    }    
	I2C_stop();
    return 0;
}
