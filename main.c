// 受信機メインdsPIC
#define FCY 69784687UL
#include <libpic30.h>
#include "mcc_generated_files/mcc.h"
#include <stdio.h>
#include <string.h>
#include "soft_i2c.h"
#include "lcd_i2c.h"
#include "twe_lite.h"


typedef union tagHL16 {
    signed short SHL;
    uint16_t HL;
    struct {
        uint8_t L;
        uint8_t H;
    };
    struct {
        unsigned :8;
        unsigned :7;
        unsigned T:1;
    };
} HL16;


typedef union tagHL32 {
    unsigned long HL;
    struct {
        uint8_t L;
        uint16_t M;
        uint8_t H;
    };
} HL32;


// PWM 送信パッケージ
typedef union tagPWM4 {
    uint16_t pwm[4];
    uint8_t buf[8];
} PWM4;


// AD変換値 DMAセットされる
uint16_t temp1 = 0;
uint16_t temp2 = 0;
uint16_t battery = 0;

#define TIMEOUT 1000 /* 受信タイムアウト */
uint8_t sid[] = {0x82, 0x02, 0x2e, 0x90}; // 送信機の TWE LITE シリアル番号
#define RSV_BYTES 8 /* 電波受信すべきデーターのバイト数 送信機 send_main に応じた値でなければならない */ 
#define SPI_BYTES 8 /* SPI送受信するデーターのバイト数 */
#define MAX_CNT_ERR 5 /* 連続エラーがこれだけ続くと強制停止 */
#define LOADING_COUNT 20 /* 装填動作待ち時間（片道・30ミリ秒単位）*/

//#define SPI_BYTES2 8 /* SPI送信するデーターのバイト数 */
PWM4 data1, data2, data3; // SPI送信するデーター
 // data3 は、data3.pwm[0] だけにセットすること 
PWM4 data; // SPI受信格納先
 // MSB15 1:充電完了 0:充電未完
 // bit14 1:IGBT破損の可能性あり 0:正常
 // bit13 1:弾切れ 0:装填されている
 // bit12 予備
 // bit11-0 充電現在電圧
uint16_t cnt_err = 0; // ERROR 連続回数
//HL16 data_ok; // 正常に受信できた最後のデーター 0:強制停止
uint8_t charge = 0; // 1:充電ON受信 0:充電OFF受信
uint8_t charge0 = 0; // 直前の charge


uint8_t fired; // 射撃弾数

uint8_t rsvt[32]; // 受信バッファー
#define RSVA_BYTES 24 
uint8_t rsv[RSVA_BYTES]; // 正常に受信できたデーターの転送先
char buf[32];


uint16_t countdown = 0; // カウントダウンタイマー
 // TMR2 割り込みごとにカウントダウンする。１カウント30ミリ秒



// SPI送信
void spi_send(void) {
    uint16_t portb; // = step_val;
    data3.pwm[3] = data3.pwm[2] = data3.pwm[1] = data3.pwm[0];
    uint8_t idx, b, d, m, *dp = (uint8_t *)(data.buf);
    uint8_t d1, *dp1 = data1.buf;
    uint8_t d2, *dp2 = data2.buf;
    uint8_t d3, *dp3 = data3.buf;
    // パケット先頭 STRB=1 で相手に伝える
    // クロックを1にするまで15μ秒以上空けるのを仕様とする
    SPI_STRB_SetHigh();
    __delay_us(14);

    SPI_STRB_SetLow();
    for (idx=0; idx<SPI_BYTES; idx++) {
        d = 0;
        d1 = (*(dp1++));
        d2 = (*(dp2++));
        d3 = (*(dp3++));
        m = 0x80;
        for (b=0; b<8; b++) {
            portb = 0x4000; // SPI_CLOCK_SetHigh();
            if (d1 & m) {
                portb |= 0x2000; // SPI_DATA_SetHigh();
            }
            else {
                portb &= 0xdfff; // SPI_DATA_SetLow();
            }
            if (d2 & m) {
                portb |= 0x1000; // SPI2_DATA_SetHigh();
            }
            else {
                portb &= 0xefff; // SPI2_DATA_SetLow();
            }
            if (d3 & m) {
                portb |= 0x0800; // SPI3_OUT_SetHigh();
            }
            else {
                portb &= 0xf7ff; // SPI3_OUT_SetLow();
            }
            LATB = portb;
            __delay_us(3);
            m >>= 1;
            d <<= 1;
            portb &= 0xbfff; // SPI_CLOCK_SetLow();
            LATB = portb;
            if (SPI3_IN_GetValue()) {
                d++;
            }
            __delay_us(3);
        }        
        (*(dp++)) = d;
    }
    SPI_DATA_SetLow();
    SPI2_DATA_SetLow();
    SPI3_OUT_SetLow();
}



// 受信データー確認
// 受信あれば1 なければ0 を返す
char check_rsv(void) {
    uint8_t i, n = get_rsv_size();
    if (n <= 15) {
        return 0; // 受信データーが少な過ぎる
    }
    // 送信機のシリアルIDを確認
    for (i=0; i<4; i++) {
        if (rsvt[i+3] != sid[i]) {
            return 0; // 送信機のシリアルIDと違う
        }
    }
    if (rsvt[13] != RSV_BYTES) {
        return 0; // データー長が想定と違う
    }

    for (i=0; i<RSVA_BYTES; i++) {
        rsv[i] = rsvt[i];
    }
    return 1;
}


void int_timer(void) {
    if (countdown) {
        countdown --;
    }
    spi_send();
}


// 装填サーボ開
void servo_open(void) {
    data1.pwm[0] = 12000 - 2200;
    data1.pwm[3] = 12000 + 2200;
}

// 装填サーボ閉
void servo_close(void) {
    data1.pwm[0] = 12000 + 2800;
    data1.pwm[3] = 12000 - 2800;
}


uint8_t mode_charger = 0; // コンデンサー充電モード
 // 0: 何もせず待機
 // 1: パチンコ玉装填動作・往路
 // 2: パチンコ玉装填動作・復路
 // 3: 装填状態再確認
 // 4: 弾切れ
 // 5: コンデンサー充電中
 // 6: 射撃後
 

int main(void)
{
    // initialize the device
    SYSTEM_Initialize();
    UART1_SetRxInterruptHandler(TWE_rsv_int);
    set_rsv_buf(rsvt, 32);
    TMR2_SetInterruptHandler(int_timer);
    
    DMA_ChannelEnable(DMA_CHANNEL_1);
    DMA_PeripheralAddressSet(DMA_CHANNEL_1, (volatile unsigned int) &ADC1BUF0);
    DMA_StartAddressASet(DMA_CHANNEL_1, (uint16_t)(&temp1));        
    //TMR2_SetInterruptHandler(int_timer);

    __delay_ms(100); // I2C バス安定化待ち    
    LCD_i2c_init(8);
    //LCD_i2C_cmd(0x80);
    //LCD_i2C_data("ABCDE");

    fired = 0;
            
    uint8_t id = 0; // 応答ID
    uint8_t WiFi = 0; // 受信感度
    uint8_t i;
    uint16_t t = 0;

    for (i=0; i<RSVA_BYTES; i++) {
        rsv[i] = 0;
    }
    //send[0] = send[1] = send[2] = send[3] = 0;
    //send[4] = send[5] = send[6] = send[7] = 0x80; // 停止
    for (i=0; i<4; i++) {
        data1.pwm[i] = 12000;
        data2.pwm[i] = 12000;
    }

    uint8_t charged = 0; // 1:充電完了 0:充電未完
    uint8_t broken = 0; // 1:IGBT破損 0:正常
    uint8_t loaded = 0; //  1:装填されている 0:装填されていない
    uint8_t empty = 0; // 送信機に弾切れ通知するなら1
    HL16 data_back;

    //SPI2_CLOCK_SetLow();
    while (1)
    {
        WATCHDOG_TimerClear();
//        LCD_i2C_cmd(0x80);
//        sprintf(buf, "%6d", dc++);
//        LCD_i2C_data(buf);
        data_back.HL = 0;
        
        for (t=0; t<TIMEOUT; t++) {
            if (check_rsv()) {
                break;
            }
            __delay_ms(1);
        }
        clear_rsv_size();
        
        for (i=0; i<4; i++) {
            data3.pwm[i] = 0;
        }

        if (t >= TIMEOUT) { // 電波が届かない
            for (i=0; i<RSVA_BYTES; i++) {
                rsv[i] = 0;
            }
            id = 0;
            WiFi = 1;
        }
        else {
            // rsv[2] 応答ID
            id = rsv[2];
            // rsv[11] 受信強度
            WiFi = rsv[11];
            
            // rsv[14]
            // bit0  TRR-U
            // bit1  TRR-D
            // bit2  TRR-L
            // bit3  TRR-R
            // bit4  B（LCD周囲のボタン）
            // bit5  A（LCD周囲のボタン）
            // bit6  Down（LCD周囲のボタン）
            // bit7  Up（LCD周囲のボタン）

            // rsv[15]
            // bit0  TRL-U
            // bit1  TRL-D
            // bit2  TRL-L
            // bit3  TRL-R
            // bit4  左トグルスイッチが↓なら1
            // bit5  トリガーが押されたら1
            // bit6  左トリガーが押されたら1
            // bit7  トリガー同時押しで1

            // rsv[16] と [17] は使われていない
            
            // rsv[18] スロットル　右十字の上下　上で＋
            // rsv[19] エルロン　右十字の左右　右で＋
            // rsv[20] 旋回　左十字の左右　左で＋
            // rsv[21] 上下　左十字の上下　上で＋

            // スロットル送信
            uint16_t pw = rsv[18];
            pw = pw * 63 + 3936;
            data1.pwm[1] = pw;

            pw = rsv[19];
            pw = pw * 63 + 3936;
            data1.pwm[2] = pw;

            // 旋回送信
            pw = rsv[20];
            pw = pw * 63 + 3936;
            data2.pwm[1] = pw; // 前ステアリング
            pw = rsv[20];
            pw = pw * 32 + 7904;
            data2.pwm[0] = pw; // 後ステアリング

            // 充電系受信
            if (CHARGED_GetValue()) {
                charged = 1;
            }
            else {
                charged = 0;
            }
            if (ERROR_GetValue()) {
                broken = 1;
            }
            else {
                broken = 0;
            }
            if (EMPTY_GetValue()) {
                loaded = 0;
            }
            else {
                loaded = 1;
            }
broken = 0;
charged = 0;

            // 充電系送信
            charge = 0;
            if ((rsv[15] & 16) == 0) { // 左トグルが上（充電ONを受信）
                charge = 1;
                if (charge0 == 0) { // OFF→ONになったばかり
                    if (loaded) { // 装填完了している
                        mode_charger = 5; // コンデンサー充電電圧確認に移動
                    }
                    else { // 装填されていない
                        // パチンコ玉装填動作・往路に移行
                        countdown = LOADING_COUNT;
                        mode_charger = 1;
                    }
                }
                else {
                    if (broken) { // IGBT破損
                        servo_close();
                    }
                    else if (mode_charger == 1) { // パチンコ玉装填動作・往路
                        // サーボ動作待ち
                        servo_open();
                        if (countdown == 0) { // 動作待ち終了している
                            // パチンコ玉装填動作・復路に移行
                            countdown = LOADING_COUNT;
                            mode_charger = 2;
                        }
                    }
                    else if (mode_charger == 2) { // パチンコ玉装填動作・復路
                        // サーボ動作待ち
                        servo_close();
                        if (countdown == 0) { // 動作待ち終了している
                            mode_charger = 3; // 装填状態再確認
                        }
                    }
                    else if (mode_charger == 3) { // 装填状態再確認
                        if (loaded) { // 装填済み
                            mode_charger = 5; // コンデンサー充電に移動
                        }
                        else {
                            mode_charger = 4; // 弾切れに移動
                        }
                    }
                    else if (mode_charger == 4) { // 弾切れ
                        empty = 1;
                    }
                    else if (mode_charger == 5) { // コンデンサー充電
                        CHARGE_SetHigh(); // 充電ON
                        if (charged) { // 充電完了している
                            if (rsv[15] & 32) { // トリガーが押された
                                FIRE_SetHigh(); // トリガーON
                                mode_charger = 6; // 射撃後に移動
                            }
                        }
                    }
                    else { // 射撃後
                        if (charged) { // 充電完了している
                            if (rsv[15] & 32) { // トリガーが押された
                                FIRE_SetHigh(); // トリガーON
                            }
                        }
                    }
                }
            }
            else { // 充電OFFを受信
                if (mode_charger == 1) { // パチンコ玉装填動作・往路
                    // サーボ動作待ち
                    servo_open();
                    if (countdown == 0) { // 動作待ち終了している
                        // パチンコ玉装填動作・復路に移行
                        countdown = LOADING_COUNT;
                        mode_charger = 2;
                    }
                }
                else if (mode_charger == 2) { // 2: パチンコ玉装填動作・復路
                    // サーボ動作待ち
                    servo_close();
                    if (countdown == 0) { // 動作待ち終了している
                        mode_charger = 0; // 充電OFF
                    }
                }
                else {
                    // 充電OFF
                    empty = 0;
                    mode_charger = 0;
                    servo_close();
                }
            }
        }
        spi_send();
        
        // TWE LITE モジュールと通信
        uint8_t *cp = (uint8_t *)buf;
        cp[0] = 0x00; // 親機あて
        cp[1] = 0xA0; // 拡張形式
        cp[2] = id;   // 応答ID
        cp[3] = 0xFF; // オプション無し
        cp[4] = WiFi; // 受信感度

        HL16 v; // バッテリー電圧
        uint16_t vv = (uint16_t)(((long)10185 * battery) >> 12); // 1mV単位に変換 キャリブレーションする
        v.HL = (vv / 10); // 0.01V単位に変換
        if ((vv % 10) >= 5) v.HL ++; // 四捨五入
        cp[5] = v.H;
        cp[6] = v.L;

        if (charged) { // 充電完了している
            data_back.HL |= 0x8000; // 充電完了フラグ立てる            
        }
        if (broken) { // 破損している
            data_back.HL |= 0x4000; // 破損フラグ立てる            
        }
        if (empty) {
            data_back.HL |= 0x2000; // 弾切れフラグ立てる            
        }
        cp[7] = data_back.L;
        cp[8] = data_back.H;
        TWE_send(9, cp);
        
//        LCD_i2C_cmd(0x80);
//        sprintf(buf, "%4d%4d%4d%4d", rsv[14], rsv[15], rsv[16], rsv[17]);
//        LCD_i2C_data(buf);
//        LCD_i2C_cmd(0xC0);
//        sprintf(buf, "%4d%4d%4d%4d", rsv[18], rsv[19], rsv[20], rsv[21]);
//        LCD_i2C_data(buf);

        charge0 = charge;
    }
    return 1; 
}
/**
 End of File
*/

