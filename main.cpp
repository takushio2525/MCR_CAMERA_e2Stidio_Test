﻿//------------------------------------------------------------------//
//Supported MCU:   RZ/A1H
//File Contents:   kit_kirokukai2021_gr_peach ｸﾛｽﾗｲﾝ,左右ﾊｰﾌﾗｲﾝあり版
//Version number:  Ver.1.00
//Date:            2020.09.08
//Copyright:       Renesas Electronics Corporation
//                 Hitachi Document Solutions Co., Ltd.
//                 宮崎県高等学校教育研究会　工業部会
//                 　　　マイコンカーラリー指導担当代表者会
//------------------------------------------------------------------//

//------------------------------------------------------------------//
//Include
//------------------------------------------------------------------//
#include "mbed.h"
#include "iodefine.h"
#include "DisplayBace.h"
#include "image_process.h"
#include "SdUsbConnect.h"

//------------------------------------------------------------------//
//Define
//------------------------------------------------------------------//
//Motor PWM cycle
#define MOTOR_PWM_CYCLE     33332       // Motor PWM period
                                         // 1ms    P0φ/1  = 0.03us
//Servo PWM cycle
#define SERVO_PWM_CYCLE     33332       // SERVO PWM period
                                         // 16ms   P0φ/16 = 0.48us
#define SERVO_CENTER        3050//3124        // 1.5ms / 0.48us - 1 = 3124
#define HANDLE_STEP         18          // 1 degree value

//------------------------------------------------------------------//
// マスク値設定 ×：マスクあり(無効)　○：マスク無し(有効)
//------------------------------------------------------------------//
#define MASK2_2             0x66        // ×○○××○○×
#define MASK2_0             0x60        // ×○○×××××
#define MASK0_2             0x06        // ×××××○○×
#define MASK3_3             0xe7        // ○○○××○○○
#define MASK0_3             0x07        // ×××××○○○
#define MASK3_0             0xe0        // ○○○×××××
#define MASK4_0             0xf0        // ○○○○××××
#define MASK0_4             0x0f        // ××××○○○○
#define MASK4_4             0xff        // ○○○○○○○○

//------------------------------------------------------------------//
//Define(NTSC-Video)
//------------------------------------------------------------------//
#define VIDEO_INPUT_CH         (DisplayBase::VIDEO_INPUT_CHANNEL_0)
#define VIDEO_INT_TYPE         (DisplayBase::INT_TYPE_S0_VFIELD)
#define DATA_SIZE_PER_PIC      (2u)

//! Frame buffer stride: Frame buffer stride should be set to a multiple of 32 or 128
//  in accordance with the frame buffer burst transfer mode.
#define PIXEL_HW               (160u)  // QVGA
#define PIXEL_VW               (120u)  // QVGA
#define VIDEO_BUFFER_STRIDE    (((PIXEL_HW * DATA_SIZE_PER_PIC) + 31u) & ~31u)
#define VIDEO_BUFFER_HEIGHT    (PIXEL_VW)

//------------------------------------------------------------------//
//Constructor
//------------------------------------------------------------------//
// Create DisplayBase object
DisplayBase Display;

Ticker      interrput;
Serial      pc(USBTX, USBRX);

DigitalOut  LED_R(P6_13);               // LED1 on the GR-PEACH board
DigitalOut  LED_G(P6_14);               // LED2 on the GR-PEACH board
DigitalOut  LED_B(P6_15);               // LED3 on the GR-PEACH board
DigitalOut  USER_LED(P6_12);            // USER_LED on the GR-PEACH board
DigitalIn   user_botton(P6_0);          // SW1 on the GR-PEACH board

BusIn       dipsw( P7_15, P8_1, P2_9, P2_10 ); // SW1 on Shield board

DigitalOut  Left_motor_signal(P4_6);    // Used by motor function
DigitalOut  Right_motor_signal(P4_7);   // Used by motor function
DigitalIn   push_sw(P2_13);             // SW1 on the Motor Drive board
DigitalOut  LED_3(P2_14);               // LED3 on the Motor Drive board
DigitalOut  LED_2(P2_15);               // LED2 on the Motor Drive board

//------------------------------------------------------------------//
//Prototype
//------------------------------------------------------------------//
void init_Camera( void );
void ChangeFrameBuffer( void );
static void IntCallbackFunc_Vfield(DisplayBase::int_type_t int_type);
static void WaitVfield(const int32_t wait_count);
static void IntCallbackFunc_Vsync(DisplayBase::int_type_t int_type);
static void WaitVsync(const int32_t wait_count);
void init_MTU2_PWM_Motor( void );       // Initialize PWM functions
void init_MTU2_PWM_Servo( void );       // Initialize PWM functions
void intTimer( void );                   // 1ms period
unsigned char user_button_get( void );
void led_m(int time, int r,int g, int b);
void led_m_process( void );             // Only function for interrupt

unsigned char sensor_inp( unsigned char mask );
int check_crossline( void );
int check_rightline( void );
int check_leftline( void );
void led_out(int led);
unsigned char pushsw_get( void );
void motor( int accele_l, int accele_r );
void handle( int angle );
unsigned char dipsw_get( void );
unsigned char shikiichi_henkan( int gyou, int s, int sa );
char getImage( int ix, int iy );

int getCompileYear( const char *p );
int getCompileMonth( const char *p );
int getCompileDay( const char *p );
int getCompileHour( const char *p );
int getCompilerMinute( const char *p );
int getCompilerSecond( const char *p );
unsigned long convertBCD_CharToLong( unsigned char hex );

//------------------------------------------------------------------//
//Global variable (NTSC-video)
//------------------------------------------------------------------//
static uint8_t FrameBuffer_Video_A[VIDEO_BUFFER_STRIDE * VIDEO_BUFFER_HEIGHT]__attribute((section("NC_BSS"),aligned(16)));  //16 bytes aligned!;
static uint8_t FrameBuffer_Video_B[VIDEO_BUFFER_STRIDE * VIDEO_BUFFER_HEIGHT]__attribute((section("NC_BSS"),aligned(16)));  //16 bytes aligned!;
uint8_t * write_buff_addr = FrameBuffer_Video_A;
uint8_t * save_buff_addr  = FrameBuffer_Video_B;
static volatile int32_t vsync_count;
static volatile int32_t vfield_count;
static volatile int32_t vfield_count2 = 1;
static volatile int32_t vfield_count2_buff;

//------------------------------------------------------------------//
//Global variable for Image process
//------------------------------------------------------------------//
unsigned char   ImageData_A[ ( ( PIXEL_HW * 2) * PIXEL_VW ) ];
unsigned char   ImageData_B[ ( PIXEL_HW * PIXEL_VW ) ];
unsigned char   ImageComp_B[ ( PIXEL_HW * PIXEL_VW ) ];
unsigned char   ImageBinary[ ( PIXEL_HW * PIXEL_VW ) ];

//------------------------------------------------------------------//
//Global variable for Trace program
//------------------------------------------------------------------//
const char *C_DATE = __DATE__;         // コンパイルした日付
const char *C_TIME = __TIME__;         // コンパイルした時間
const char monthStr[] = { "JanFebMarAprMayJunJulAugSepOctNovDec" };
                                        // 月変換テーブル

volatile unsigned long cnt1;            // Used within main
volatile unsigned long cnt_printf;
volatile unsigned long cnt_msd;
volatile unsigned long cnt_msdwritetime;
volatile unsigned long cnt_debug;

volatile int pattern;                  // Pattern numbers
volatile int initFlag = 1;             // Initialize flag

volatile int mled_time, mled_r, mled_g, mled_b;
volatile int debug_mode;

FILE *fp;
struct tm t;                            // microSDの日付用
volatile unsigned char sensor_bin;
volatile unsigned char bar;
volatile int log_pattern = 901;
volatile int log_mode;                 // 0:待機 1:ファイルオープン
                                        // 2:ログ記録中 3:ログファイルクローズ
                                        // ※クローズしないとファイルが保存されない
volatile int msdError;
volatile int msd_handle, msd_l, msd_r;

///****************************************************************
// Main function
///****************************************************************
int main( void )
{
    int x, y, i;
    char moji_work[128], c;
    int sw_now = 0, sw_before = -1;

    initFlag = 1;                       // Initialization start

    // 起動時にボタンが押されているとデバッグモードON
    if( user_button_get() != 0 ) {
        debug_mode = 1;
    }

    // Camera start
    init_Camera();
    // wait to stabilize NTSC signal (about 170ms)
    wait(0.2);

    // Initialize MCU functions
    init_MTU2_PWM_Motor();
    init_MTU2_PWM_Servo();
    interrput.attach(&intTimer, 0.001);
    pc.baud(230400);
    pc.printf( "\033[H" );      // デバッグ用画面クリア
    pc.printf( "\033[2J" );

    // Initialize Micon Car state
    handle( 0 );
    motor( 0, 0 );
    led_out( 0x0 );
    led_m( 10, 0, 1, 1);  // 10% R=0,G=1,B=1

    t.tm_sec  = getCompilerSecond( C_TIME );    // 0-59
    t.tm_min  = getCompilerMinute( C_TIME );    // 0-59
    t.tm_hour = getCompileHour( C_TIME );       // 0-23
    t.tm_mday = getCompileDay( C_DATE );        // 1-31
    t.tm_mon  = getCompileMonth( C_DATE ) - 1;  // 0-11 +1する
    t.tm_year = getCompileYear( C_DATE ) - 1900;// -1900する
    time_t seconds = mktime(&t);
    set_time(seconds);

    SdUsbConnect storage("storage");
    //storage.wait_connect(); // 認識するまでここで止まってしまうのでこの命令は使えない
    cnt_msd = 0;
    while( storage.connect() == 0 ) { // STORAGE_NON = 0
        Thread::wait(100);
        if( cnt_msd >= 1000) { // この時間以上microSDの接続が確認できなければエラー
            msdError = 1;
            break;
        }
    }
    if( msdError == 1 ) {
        led_m( 50, 1, 0, 0);
        cnt_msd = 0;
        while( cnt_msd < 2000 ) {   // 2s待つ
        }
    }

    initFlag = 0;                   // Initialization end

    // Debug Program
    if( debug_mode == 1 ) {
        led_m( 10, 0, 1, 1 );

        while( 1 ) {
            // LEDにモニターする
            led_out( (sensor_inp(0x80)>>6) | sensor_inp(0x01) );
            USER_LED = sensor_inp(0x18) != 0x00 ? 1 : 0;

            // ユーザーボタンを(長めに)押すとデバッグ画面を切り換える
            if( cnt_debug >= 10 ) {
                cnt_debug = 0;
                sw_now = user_button_get();
                if( sw_now == 1 && sw_before == 0) {
                    debug_mode++;
                    if( debug_mode >= 5 ) {
                        debug_mode = 1;
                    }
                    pc.printf( "\033[H" );
                    pc.printf( "\033[2J" ); // 画面クリア
                }
                sw_before = sw_now;
            }

            if( cnt_printf >= 200 ) {
                cnt_printf=0;

                switch( debug_mode ) {
                case 1:
                    // 補正値で表示 しきい値180以上を"1" 180を変えると、しきい値が変わる
                    pc.printf( "shikii chi 180\r\n" );
                    for( y=0; y<30; y++ ) {
                        pc.printf( "%3d:%08ld ", y+ 0, convertBCD_CharToLong(shikiichi_henkan( y+ 0, 180, 8)));
                        pc.printf( "%3d:%08ld ", y+30, convertBCD_CharToLong(shikiichi_henkan( y+30, 180, 8)));
                        pc.printf( "%3d:%08ld ", y+60, convertBCD_CharToLong(shikiichi_henkan( y+60, 180, 8)));
                        pc.printf( "%3d:%08ld ", y+90, convertBCD_CharToLong(shikiichi_henkan( y+90, 180, 8)));
                        pc.printf( "\r\n");
                    }
                    pc.printf( "\033[H" );
                    break;

                case 2:
                    // 補正値で表示 しきい値120以上を"1" 180を変えると、しきい値が変わる
                    pc.printf( "shikii chi 120\r\n" );
                    for( y=0; y<30; y++ ) {
                        pc.printf( "%3d:%08ld ", y+ 0, convertBCD_CharToLong(shikiichi_henkan( y+ 0, 120, 8)));
                        pc.printf( "%3d:%08ld ", y+30, convertBCD_CharToLong(shikiichi_henkan( y+30, 120, 8)));
                        pc.printf( "%3d:%08ld ", y+60, convertBCD_CharToLong(shikiichi_henkan( y+60, 120, 8)));
                        pc.printf( "%3d:%08ld ", y+90, convertBCD_CharToLong(shikiichi_henkan( y+90, 120, 8)));
                        pc.printf( "\r\n");
                    }
                    pc.printf( "\033[H" );
                    break;

                case 3:
                    // https://www.sejuku.net/blog/24934
                    // 文字の色
                    // \x1b[30m 黒 \x1b[31m 赤 \x1b[32m 緑 \x1b[33m 黄
                    // \x1b[34m 青 \x1b[35m マゼンダ \x1b[36m シアン
                    // \x1b[37m 白 \x1b[39m デフォルトに戻す
                    //背景色の指定
                    // \x1b[40m 黒 \x1b[41m 赤 \x1b[42m 緑 \x1b[43m 黄
                    // \x1b[44m 青 \x1b[45m マゼンダ \x1b[46m シアン
                    // \x1b[47m 灰 \x1b[49m デフォルトに戻す

                    // 1行飛ばしで表示(しきい値180以上を"1"とする)
                    pc.printf( "shi 0         0         0         0         0         0         0         0         0         0         1         1         1         1         1         1        1\r\n");
                    pc.printf( "kii 0         1         2         3         4         5         6         7         8         9         0         1         2         3         4         5        5\r\n");
                    pc.printf( "180 0123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789\r\n");
                    for( y=0; y<120; y+=2 ) {
                        pc.printf("%03d:", y );
                        for( x=0;x<160;x++ ) {
                            c = getImage(x, y) >= 180 ? 1 : 0;  // 180を変えるとしきい値が変わる
                            if( y==60 && (x==31 || x==43 || x==54 || x==71 || x==88 || x==105 || x==116 || x==128) ) {
                                pc.printf( "\x1b[43m%d\x1b[49m", c );
                            } else {
                                pc.printf( "%d", c );
                            }
                        }
                        pc.printf( "  \r\n" );
                    }
                    pc.printf( "\033[H" );
                    break;

                case 4:
                    // 60～119行を表示(しきい値120以上を"1"とする)
                    pc.printf( "shi 0         0         0         0         0         0         0         0         0         0         1         1         1         1         1         1        1\r\n");
                    pc.printf( "kii 0         1         2         3         4         5         6         7         8         9         0         1         2         3         4         5        5\r\n");
                    pc.printf( "120 0123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789\r\n");
                    for( y=60; y<120; y++ ) {
                        pc.printf("%03d:", y );
                        for( x=0;x<160;x++ ) {
                            c = getImage(x, y) >= 120 ? 1 : 0;  // 120を変えるとしきい値が変わる
                            if( (y==110 || y==111 || y==112 || y==113 || y==114) && (x==31 || x==128) ) {
                                pc.printf( "\x1b[43m%d\x1b[49m", c );
                            } else {
                                pc.printf( "%d", c );
                            }
                        }
                        pc.printf( "  \r\n" );
                    }
                    pc.printf( "\033[H" );
                    break;
                }
            }
        }
    }

    // 通常走行
    // ターミナルに出力とmicroSDへログ保存
    // 走行プログラムは、intTimer関数(1msごとの割り込み)で行う
    while( 1 ) {
        // 通常走行中のデバッグ用　ターミナルに出力
        if( cnt_printf >= 200 ) {
            cnt_printf = 0;
            pc.printf( "sensor = %08ld\r\n", convertBCD_CharToLong(sensor_inp(0xff) ) );
            pc.printf( "b a r  = %08ld\r\n", convertBCD_CharToLong(bar) );
            pc.printf( "bit7=%3d bit6=%3d bit5=%3d bit4=%3d ", getImage( 31, 60), getImage( 43, 60), getImage( 54, 60), getImage( 71, 60) );
            pc.printf( "bit3=%3d bit2=%3d bit1=%3d bit0=%3d ", getImage( 88, 60), getImage(105, 60), getImage(116, 60), getImage(128, 60) );
            pc.printf( "\r\n" );
            pc.printf( "\033[H" );
        }

        // ログ保存処理
        switch( log_pattern ) {
        case 901:
            if( msdError == 1 ) {
                log_pattern = 999;
            } else {
                log_pattern = 902;
            }
            break;

        case 902:
            if( log_mode == 1 ) {
                log_pattern = 903;
            }
            break;

        case 903:
            i = 0;
            if( (fp = fopen("/storage/renban.txt", "r") ) != NULL) {
                fscanf( fp , "%d", &i );
                fclose( fp );
            }
            if( i < 0 || i > 9999 ) {
                i = 0;
            }
            if( (fp = fopen("/storage/renban.txt" , "w") ) != NULL ) {
                fprintf( fp,"%d", i + 1 );
                fclose( fp );
            }
            sprintf( moji_work, "/storage/data%04d.csv", i );
            if( (fp = fopen( moji_work , "w") ) != NULL ) {
                fprintf( fp,"Log %d\n", i );
                log_pattern = 904;
            } else {
                log_pattern = 999;
            }
            break;

        case 904:
            if( log_mode == 2 ) {
                log_pattern = 905;
                cnt_msd = 0;
            }
            break;

        case 905:
            // log_mode == 2 の処理
            if( cnt_msd >= 10 ) {
                cnt_msd = 0;
                // microSDに保存する内容
                // 文字数に制限はないが、増やしすぎると10msごとにならない
                fprintf( fp,"%d,%d,b%08ld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                        int(cnt_msdwritetime % 1000),
                        pattern,
                        convertBCD_CharToLong( sensor_inp(0xff) ),
                        msd_handle,
                        msd_l,
                        msd_r,
                        getImage(  31, 60),
                        getImage(  43, 60),
                        getImage(  54, 60),
                        getImage(  71, 60),
                        getImage(  88, 60),
                        getImage( 105, 60),
                        getImage( 116, 60),
                        getImage( 128, 60)
                );
            }
            if( log_mode == 3 ) {
                fclose(fp);     // fcloseしないと保存されない
                log_pattern = 999;
            }
            break;

        case 999:
            // 終了　何もしない
            break;
        }
    }
}

///****************************************************************
// Initialize functions
///****************************************************************
//------------------------------------------------------------------//
//Initialize MTU2 PWM functions
//------------------------------------------------------------------//
//MTU2_3, MTU2_4
//Reset-Synchronized PWM mode
//TIOC4A(P4_4) :Left-motor
//TIOC4B(P4_5) :Right-motor
//------------------------------------------------------------------//
void init_MTU2_PWM_Motor( void )
{
    // Port setting for S/W I/O Control
    // alternative mode

    // MTU2_4 (P4_4)(P4_5)
    GPIOPBDC4   = 0x0000;               // Bidirection mode disabled
    GPIOPFCAE4 &= 0xffcf;               // The alternative function of a pin
    GPIOPFCE4  |= 0x0030;               // The alternative function of a pin
    GPIOPFC4   &= 0xffcf;               // The alternative function of a pin
                                        // 2nd altemative function/output
    GPIOP4     &= 0xffcf;               //
    GPIOPM4    &= 0xffcf;               // p4_4,P4_5:output
    GPIOPMC4   |= 0x0030;               // P4_4,P4_5:double

    // Module stop 33(MTU2) canceling
    CPGSTBCR3  &= 0xf7;

    // MTU2_3 and MTU2_4 (Motor PWM)
    MTU2TCR_3   = 0x20;                 // TCNT Clear(TGRA), P0φ/1
    MTU2TOCR1   = 0x04;                 //
    MTU2TOCR2   = 0x40;                 // N L>H  P H>L
    MTU2TMDR_3  = 0x38;                 // Buff:ON Reset-Synchronized PWM mode
    MTU2TMDR_4  = 0x30;                 // Buff:ON
    MTU2TOER    = 0xc6;                 // TIOC3B,4A,4B enabled output
    MTU2TCNT_3  = MTU2TCNT_4 = 0;       // TCNT3,TCNT4 Set 0
    MTU2TGRA_3  = MTU2TGRC_3 = MOTOR_PWM_CYCLE;
                                        // PWM-Cycle(1ms)
    MTU2TGRA_4  = MTU2TGRC_4 = 0;       // Left-motor(P4_4)
    MTU2TGRB_4  = MTU2TGRD_4 = 0;       // Right-motor(P4_5)
    MTU2TSTR   |= 0x40;                 // TCNT_4 Start
}

//------------------------------------------------------------------//
//Initialize MTU2 PWM functions
//------------------------------------------------------------------//
//MTU2_0
//PWM mode 1
//TIOC0A(P4_0) :Servo-motor
//------------------------------------------------------------------//
void init_MTU2_PWM_Servo( void )
{
    // Port setting for S/W I/O Control
    // alternative mode

    // MTU2_0 (P4_0)
    GPIOPBDC4   = 0x0000;               // Bidirection mode disabled
    GPIOPFCAE4 &= 0xfffe;               // The alternative function of a pin
    GPIOPFCE4  &= 0xfffe;               // The alternative function of a pin
    GPIOPFC4   |= 0x0001;               // The alternative function of a pin
                                        // 2nd alternative function/output
    GPIOP4     &= 0xfffe;               //
    GPIOPM4    &= 0xfffe;               // p4_0:output
    GPIOPMC4   |= 0x0001;               // P4_0:double

    // Module stop 33(MTU2) canceling
    CPGSTBCR3  &= 0xf7;

    // MTU2_0 (Motor PWM)
    MTU2TCR_0   = 0x22;                 // TCNT Clear(TGRA), P0φ/16
    MTU2TIORH_0 = 0x52;                 // TGRA L>H, TGRB H>L
    MTU2TMDR_0  = 0x32;                 // TGRC and TGRD = Buff-mode
                                        // PWM-mode1
    MTU2TCNT_0  = 0;                    // TCNT0 Set 0
    MTU2TGRA_0  = MTU2TGRC_0 = SERVO_PWM_CYCLE;
                                        // PWM-Cycle(16ms)
    MTU2TGRB_0  = MTU2TGRD_0 = 0;       // Servo-motor(P4_0)
    MTU2TSTR   |= 0x01;                 // TCNT_0 Start
}

//------------------------------------------------------------------//
//Initialize Camera function
//------------------------------------------------------------------//
void init_Camera( void )
{
    // NTSC-Video
    DisplayBase::graphics_error_t error;

    // Graphics initialization process
    error = Display.Graphics_init(NULL);
    if (error != DisplayBase::GRAPHICS_OK) {
        pc.printf("Line %d, error %d\n", __LINE__, error);
        while (1);
    }

    error = Display.Graphics_Video_init( DisplayBase::INPUT_SEL_VDEC, NULL);
    if( error != DisplayBase::GRAPHICS_OK ) {
        while(1);
    }

    // Interrupt callback function setting (Vsync signal input to scaler 0)
    error = Display.Graphics_Irq_Handler_Set(DisplayBase::INT_TYPE_S0_VI_VSYNC, 0, IntCallbackFunc_Vsync);
    if (error != DisplayBase::GRAPHICS_OK) {
        pc.printf("Line %d, error %d\n", __LINE__, error);
        while (1);
    }

    // Video capture setting (progressive form fixed)
    error = Display.Video_Write_Setting(
                VIDEO_INPUT_CH,
                DisplayBase::COL_SYS_NTSC_358,
                write_buff_addr,
                VIDEO_BUFFER_STRIDE,
                DisplayBase::VIDEO_FORMAT_YCBCR422,
                DisplayBase::WR_RD_WRSWA_32_16BIT,
                PIXEL_VW,
                PIXEL_HW
            );
    if (error != DisplayBase::GRAPHICS_OK) {
        pc.printf("Line %d, error %d\n", __LINE__, error);
        while (1);
    }

    // Interrupt callback function setting (Field end signal for recording function in scaler 0)
    error = Display.Graphics_Irq_Handler_Set(VIDEO_INT_TYPE, 0, IntCallbackFunc_Vfield);
    if (error != DisplayBase::GRAPHICS_OK) {
        pc.printf("Line %d, error %d\n", __LINE__, error);
        while (1);
    }

    // Video write process start
    error = Display.Video_Start (VIDEO_INPUT_CH);
    if (error != DisplayBase::GRAPHICS_OK) {
        pc.printf("Line %d, error %d\n", __LINE__, error);
        while (1);
    }

    // Video write process stop
    error = Display.Video_Stop (VIDEO_INPUT_CH);
    if (error != DisplayBase::GRAPHICS_OK) {
        pc.printf("Line %d, error %d\n", __LINE__, error);
        while (1);
    }

    // Video write process start
    error = Display.Video_Start (VIDEO_INPUT_CH);
    if (error != DisplayBase::GRAPHICS_OK) {
        pc.printf("Line %d, error %d\n", __LINE__, error);
        while (1);
    }

    // Wait vsync to update resister
    WaitVsync(1);

    // Wait 2 Vfield(Top or bottom field)
    WaitVfield(2);
}

//------------------------------------------------------------------//
//ChangeFrameBuffer function
//------------------------------------------------------------------//
void ChangeFrameBuffer( void )
{
    // NTSC-Video
    DisplayBase::graphics_error_t error;

    // Change write buffer
    if (write_buff_addr == FrameBuffer_Video_A) {
        write_buff_addr = FrameBuffer_Video_B;
        save_buff_addr  = FrameBuffer_Video_A;
    } else {
        write_buff_addr = FrameBuffer_Video_A;
        save_buff_addr  = FrameBuffer_Video_B;
    }
    error = Display.Video_Write_Change(
                VIDEO_INPUT_CH,
                write_buff_addr,
                VIDEO_BUFFER_STRIDE);
    if (error != DisplayBase::GRAPHICS_OK) {
        pc.printf("Line %d, error %d\n", __LINE__, error);
        while (1);
    }
}

//------------------------------------------------------------------//
// @brief       Interrupt callback function
// @param[in]   int_type    : VDC5 interrupt type
// @retval      None
//------------------------------------------------------------------//
static void IntCallbackFunc_Vfield(DisplayBase::int_type_t int_type)
{
    (void)int_type;

    if (vfield_count > 0) {
        vfield_count--;
    }
    // top or bottom (Change)
    if( vfield_count2 == 0 )  vfield_count2 = 1;
    else                      vfield_count2 = 0;
}

//------------------------------------------------------------------//
// @brief       Wait for the specified number of times Vsync occurs
// @param[in]   wait_count          : Wait count
// @retval      None
//------------------------------------------------------------------//
static void WaitVfield(const int32_t wait_count)
{
    vfield_count = wait_count;
    while (vfield_count > 0) {
        // Do nothing
    }
}

//------------------------------------------------------------------//
// @brief       Interrupt callback function for Vsync interruption
// @param[in]   int_type    : VDC5 interrupt type
// @retval      None
//------------------------------------------------------------------//
static void IntCallbackFunc_Vsync(DisplayBase::int_type_t int_type)
{
    (void)int_type;

    if (vsync_count > 0) {
        vsync_count--;
    }
}

//------------------------------------------------------------------//
// @brief       Wait for the specified number of times Vsync occurs
// @param[in]   wait_count          : Wait count
// @retval      None
//------------------------------------------------------------------//
static void WaitVsync(const int32_t wait_count)
{
    vsync_count = wait_count;
    while (vsync_count > 0) {
        // Do nothing
    }
}

//------------------------------------------------------------------//
// Interrupt function( intTimer )
//------------------------------------------------------------------//
void intTimer( void )
{
    static int      counter = 0;    // Only variable for image process
    unsigned char b;

    cnt1++;
    cnt_msdwritetime++;
    cnt_printf++;
    cnt_msd++;
    cnt_debug++;

    // field check
    if( vfield_count2 != vfield_count2_buff ) {
        vfield_count2_buff = vfield_count2;
        counter = 0;
    }

    // Top field / bottom field
    switch( counter++ ) {
    case 0:
        ImageCopy( write_buff_addr, PIXEL_HW, PIXEL_VW, ImageData_A, vfield_count2 );  //  0 - 59行を変換
        break;
    case 1:
        ImageCopy( write_buff_addr, PIXEL_HW, PIXEL_VW, ImageData_A, vfield_count2 );  // 60 - 119行を変換
        break;
    case 2:
        Extraction_Brightness( ImageData_A, PIXEL_HW, PIXEL_VW, ImageData_B, vfield_count2 );
        break;
    case 3:
        Extraction_Brightness( ImageData_A, PIXEL_HW, PIXEL_VW, ImageData_B, vfield_count2 );

        sensor_bin = shikiichi_henkan(  60, 180, 8 ); //  60行目 しきい値180 隣同士の差分が8以下なら0x00にする
        // スタートバー検出 bit7とbit0のみ見る
        bar        = ( shikiichi_henkan( 110, 120, 8) |
                       shikiichi_henkan( 111, 120, 8) |
                       shikiichi_henkan( 112, 120, 8) |
                       shikiichi_henkan( 113, 120, 8) |
                       shikiichi_henkan( 114, 120, 8) ) & 0x81;
        break;

    default:
        break;
    }

    // LED(rgb) on the GR-peach board
    led_m_process(); // LEDの点滅処理を行う

    if( debug_mode != 0 || initFlag != 0 ) {
        return;    // デバッグモードなら、ここで割り込み終了
                    // 走行プログラムは実行しない
    }

    // モータドライブ基板のLED2個は、スタートバーの反応チェック用 スタート時にLEDが点灯するようにセットする
    led_out( ( (bar&0x80) >> 6 ) | (bar&0x01) );

    // GR-PEACHのUSER_LED(赤色)は、sensor_inp関数の中心2個のどれかが"1"なら、点灯する
    USER_LED = sensor_inp(0x18) != 0x00 ? 1 : 0;

    if( pattern >= 11 && pattern <= 100 && user_button_get() == 1 && log_mode == 2 ) {
        pattern = 101;
        log_mode = 3; // ログ保存終了
    }

    switch( pattern ) {

    case 0:
        // スイッチ入力待ち
        if( pushsw_get() == 1 ) {
            pattern = 1;
            log_mode = 1; // ログファイルオープン
            cnt1 = 0;
            break;
        }
        if( bar != 0x00 ) {
            led_m( 50, 0, 1, 0);  // スタートバーセットOK状態→緑色点灯
        } else {
            led_m( 10, 1, 0, 0);  // スタートバーセットNG状態→赤色点灯
        }
        break;

    case 1:
        // スタートバーが開いたかチェック
        if( bar == 0x00 ) {
            // スタート！！
            led_m( 0, 0, 0, 0);
            log_mode = 2;       // ログ記録中
            cnt_msdwritetime = 0;
            pattern = 2;
            cnt1 = 0;
            break;
        }
        led_m( 80, 1, 1, 0);
        break;

    case 2:
        handle( 0 );
        motor( 50 ,50 );
        if( cnt1 >= 200 ) {
            pattern = 11;
        }
        break;

    case 11:
        // 通常トレース
        if( check_crossline() == 1 ) {       // クロスラインチェック
            pattern = 21;
            break;
        }
        if( check_rightline() == 1 ) {       // 右ハーフラインチェック
            pattern = 51;
            break;
        }
        if( check_leftline() == 1 ) {        // 左ハーフラインチェック
            pattern = 61;
            break;
        }
        switch( sensor_inp(MASK3_3) ) {
            case 0x00:
                // センタ→まっすぐ
                handle( 0 );
                motor( 100 ,100 );
                break;

            case 0x04:
                // 微妙に左寄り→右へ微曲げ
                handle( 5 );
                motor( 100 ,100 );
                break;

            case 0x06:
                // 少し左寄り→右へ小曲げ
                handle( 10 );
                motor( 80 ,67 );
                break;

            case 0x07:
                // 中くらい左寄り→右へ中曲げ
                handle( 15 );
                motor( 50 ,38 );
                break;

            case 0x03:
                // 大きく左寄り→右へ大曲げ
                handle( 25 );
                motor( 30 ,19 );
                pattern = 12;
                break;

            case 0x20:
                // 微妙に右寄り→左へ微曲げ
                handle( -5 );
                motor( 100 ,100 );
                break;

            case 0x60:
                // 少し右寄り→左へ小曲げ
                handle( -10 );
                motor( 67 ,80 );
                break;

            case 0xe0:
                // 中くらい右寄り→左へ中曲げ
                handle( -15 );
                motor( 38 ,50 );
                break;

            case 0xc0:
                // 大きく右寄り→左へ大曲げ
                handle( -25 );
                motor( 19 ,30 );
                pattern = 13;
                break;

            default:
                break;
        }
        break;

    case 12:
        // 右へ大曲げの終わりのチェック
        if( check_crossline() == 1 ) {       // 大曲げ中もクロスラインチェック
            pattern = 21;
            break;
        }
        if( check_rightline() == 1 ) {       // 右ハーフラインチェック
            pattern = 51;
            break;
        }
        if( check_leftline() == 1 ) {        // 左ハーフラインチェック
            pattern = 61;
            break;
        }
        if( sensor_inp(MASK3_3) == 0x06 ) {
            pattern = 11;
        }
        break;

    case 13:
        // 左へ大曲げの終わりのチェック
        if( check_crossline() == 1 ) {       // 大曲げ中もクロスラインチェック
            pattern = 21;
            break;
        }
        if( check_rightline() == 1 ) {       // 右ハーフラインチェック
            pattern = 51;
            break;
        }
        if( check_leftline() == 1 ) {        // 左ハーフラインチェック
            pattern = 61;
            break;
        }
        if( sensor_inp(MASK3_3) == 0x60 ) {
            pattern = 11;
        }
        break;

    case 21:
        // クロスライン検出時の処理
        led_m( 100, 1, 0, 0);
        motor( 0 ,0 );
        pattern = 22;
        cnt1 = 0;
        break;

    case 22:
        // クロスラインを読み飛ばす
        if( cnt1 >= 100 ) {
            pattern = 23;
            cnt1 = 0;
        }
        break;

    case 23:
        // クロスライン後のトレース、クランク検出
        if( sensor_inp(MASK4_0) == 0xf0 ) {
            // 左クランクと判断→左クランククリア処理へ
            led_m( 100, 0, 1, 0);
            handle( -38 );
            motor( 10 ,50 );
            pattern = 31;
            cnt1 = 0;
            break;
        }
        if( sensor_inp(MASK0_4) == 0x0f ) {
            // 右クランクと判断→右クランククリア処理へ
            led_m( 100, 0, 0, 1);
            handle( 38 );
            motor( 50 ,10 );
            pattern = 41;
            cnt1 = 0;
            break;
        }
        switch( sensor_inp(MASK3_3) ) {
            case 0x00:
                // センタ→まっすぐ
                handle( 0 );
                motor( 40 ,40 );
                break;
            case 0x04:
            case 0x06:
            case 0x07:
            case 0x03:
                // 左寄り→右曲げ
                handle( 8 );
                motor( 40 ,35 );
                break;
            case 0x20:
            case 0x60:
            case 0xe0:
            case 0xc0:
                // 右寄り→左曲げ
                handle( -8 );
                motor( 35 ,40 );
                break;
        }
        break;

    case 31:
        // 左クランククリア処理　安定するまで少し待つ
        led_m( 100, 0, 1, 0);
        if( cnt1 >= 200 ) {
            pattern = 32;
            cnt1 = 0;
        }
        break;

    case 32:
        // 左クランククリア処理　曲げ終わりのチェック
        if( sensor_inp(MASK3_3) == 0x60 ) {
            led_m( 100, 0, 0, 0);
            pattern = 11;
            cnt1 = 0;
        }

        // 外側の白線を読んだら
        if( sensor_inp(MASK3_3) == 0x07 ) {
            pattern = 33;
            break;
        }

        break;

    case 33:
        // 左クランククリア処理　外側の白線と見間違わないようにする
        b =  sensor_inp(MASK3_3);
        if( b == 0x83 || b == 0x81 || b == 0xc1 ) {
            pattern = 32;
        }
        break;

    case 41:
        // 右クランククリア処理　安定するまで少し待つ
        led_m( 100, 0, 0, 1);
        if( cnt1 >= 200 ) {
            pattern = 42;
            cnt1 = 0;
        }
        break;

    case 42:
        // 右クランククリア処理　曲げ終わりのチェック
        if( sensor_inp(MASK3_3) == 0x06 ) {
            led_m( 100, 0, 0, 0);
            pattern = 11;
            cnt1 = 0;
        }

        // 外側の白線を読んだら
        if( sensor_inp(MASK3_3) == 0xe0 ) {
            pattern = 43;
            break;
        }
        break;

    case 43:
        // 右クランククリア処理　外側の白線と見間違わないようにする
        b =  sensor_inp(MASK3_3);
        if( b == 0xc1 || b == 0x81 || b == 0x83 ) {
            pattern = 42;
        }
        break;

    case 51:
        // 右ハーフライン検出時の処理
        led_m( 100, 0, 1, 0);
        handle( 0 );
        motor( 0 ,0 );
        pattern = 52;
        cnt1 = 0;
        break;

    case 52:
        // 右ハーフラインを読み飛ばす
        if( cnt1 >= 100 ) {
            pattern = 53;
            cnt1 = 0;
        }
        if( check_crossline() == 1 ) {
            pattern = 21;
            break;
        }
        break;

    case 53:
        // 右ハーフライン後のトレース、レーンチェンジ
        if( sensor_inp(MASK4_4) == 0x00 ) {
            handle( 15 );
            motor( 40 ,31 );
            pattern = 54;
            cnt1 = 0;
            break;
        }
        switch( sensor_inp(MASK3_3) ) {
            case 0x00:
                // センタ→まっすぐ
                handle( 0 );
                motor( 40 ,40 );
                break;
            case 0x04:
            case 0x06:
            case 0x07:
            case 0x03:
                // 左寄り→右曲げ
                handle( 8 );
                motor( 40 ,35 );
                break;
            case 0x20:
            case 0x60:
            case 0xe0:
            case 0xc0:
                // 右寄り→左曲げ
                handle( -8 );
                motor( 35 ,40 );
                break;
            default:
                break;
        }
        break;

    case 54:
        // 右レーンチェンジ終了のチェック
        b = sensor_inp( MASK4_4 );
        if( b == 0x3c || b == 0x1c || b == 0x38 ) {
            led_m( 100, 0, 0, 0);
            pattern = 11;
            cnt1 = 0;
        }
        break;

    case 61:
        // 左ハーフライン検出時の処理
        led_m( 100, 0, 0, 1);
        handle( 0 );
        motor( 0 ,0 );
        pattern = 62;
        cnt1 = 0;
        break;

    case 62:
        // 左ハーフラインを読み飛ばす
        if( cnt1 >= 100 ) {
            pattern = 63;
            cnt1 = 0;
        }
        if( check_crossline() == 1 ) {
            pattern = 21;
            break;
        }
        break;

    case 63:
        // 左ハーフライン後のトレース、レーンチェンジ
        if( sensor_inp(MASK4_4) == 0x00 ) {
            handle( -15 );
            motor( 31 ,40 );
            pattern = 64;
            cnt1 = 0;
            break;
        }
        switch( sensor_inp(MASK3_3) ) {
            case 0x00:
                // センタ→まっすぐ
                handle( 0 );
                motor( 40 ,40 );
                break;
            case 0x04:
            case 0x06:
            case 0x07:
            case 0x03:
                // 左寄り→右曲げ
                handle( 8 );
                motor( 40 ,35 );
                break;
            case 0x20:
            case 0x60:
            case 0xe0:
            case 0xc0:
                // 右寄り→左曲げ
                handle( -8 );
                motor( 35 ,40 );
                break;
            default:
                break;
        }
        break;

    case 64:
        // 左レーンチェンジ終了のチェック
        b = sensor_inp( MASK4_4 );
        if( b == 0x38 || b == 0x1c || b == 0x3c ) {
            led_m( 100, 0, 0, 0);
            pattern = 11;
            cnt1 = 0;
        }
        break;

    case 101:
        // 終了　ログ保存中など
        led_m( 20, 1, 1, 1);
        motor( 0, 0);
        break;

    default:
        // どれでもない場合は待機状態に戻す
        pattern = 0;
        break;
    }
}

////****************************************************************
// functions ( on GR-PEACH board )
////****************************************************************
//------------------------------------------------------------------//
//user_button_get Function
//------------------------------------------------------------------//
unsigned char user_button_get( void )
{
    return (~user_botton) & 0x1;        // Read ports with switches
}

//------------------------------------------------------------------//
//led_m Function
//------------------------------------------------------------------//
void led_m(int time, int r,int g, int b)
{
    mled_time = time;
    mled_r = r;
    mled_g = g;
    mled_b = b;
}

//------------------------------------------------------------------//
//led_m_process Function for only interrupt
//------------------------------------------------------------------//
void led_m_process( void )
{
    static int cnt_led_m = 0;

    if( cnt_led_m < mled_time*5 ) {
        LED_R = mled_r;
        LED_G = mled_g;
        LED_B = mled_b;
    } else {
        LED_R = 0;
        LED_G = 0;
        LED_B = 0;
    }

    cnt_led_m++;
    if( cnt_led_m >= 500 ) {
        cnt_led_m = 0;
    }
}

///*****************************************************************
// functions ( on Motor drive board )
///*****************************************************************
//------------------------------------------------------------------//
//led_out Function
//------------------------------------------------------------------//
void led_out(int led)
{
    led = ~led;
    LED_3 = led & 0x1;
    LED_2 = ( led >> 1 ) & 0x1;
}

//------------------------------------------------------------------//
//pushsw_get Function
//------------------------------------------------------------------//
unsigned char pushsw_get( void )
{
    return (!push_sw);            // Read ports with switches
}

//------------------------------------------------------------------//
//motor speed control(PWM)
//Arguments: motor:-100 to 100
//Here, 0 is stop, 100 is forward, -100 is reverse
//------------------------------------------------------------------//
void motor( int accele_l, int accele_r )
{
    int    sw_data ;

    sw_data = dipsw_get() + 5;
    accele_l = ( accele_l * sw_data ) / 20;
    accele_r = ( accele_r * sw_data ) / 20;

    msd_l = accele_l;
    msd_r = accele_r;

    // Left Motor Control
    if( accele_l >= 0 ) {
        // forward
        Left_motor_signal = 0;
        MTU2TGRC_4 = (long)( MOTOR_PWM_CYCLE - 1 ) * accele_l / 100;
    } else {
        // reverse
        Left_motor_signal = 1;
        MTU2TGRC_4 = (long)( MOTOR_PWM_CYCLE - 1 ) * ( -accele_l ) / 100;
    }

    // Right Motor Control
    if( accele_r >= 0 ) {
        // forward
        Right_motor_signal = 0;
        MTU2TGRD_4 = (long)( MOTOR_PWM_CYCLE - 1 ) * accele_r / 100;
    } else {
        // reverse
        Right_motor_signal = 1;
        MTU2TGRD_4 = (long)( MOTOR_PWM_CYCLE - 1 ) * ( -accele_r ) / 100;
    }
}

//------------------------------------------------------------------//
//handle Function
//------------------------------------------------------------------//
void handle( int angle )
{
    msd_handle = angle;
    // When the servo move from left to right in reverse, replace "-" with "+"
    MTU2TGRD_0 = SERVO_CENTER - angle * HANDLE_STEP;
}

///*****************************************************************
// functions ( on Shield board )
///*****************************************************************
//------------------------------------------------------------------//
//Dipsw get Function
//------------------------------------------------------------------//
unsigned char dipsw_get( void )
{
    return( dipsw.read() & 0x0f );
}

//------------------------------------------------------------------//
//sensor Function
//------------------------------------------------------------------//
unsigned char sensor_inp( unsigned char mask )
{
    return (sensor_bin & mask);         // 中
}

///*********************************************************************
// モジュール名 getCompileYear
// 処理概要     コンパイルした時の年を取得
// 引数　       __DATE__ のポインタ
// 戻り値       年
///*********************************************************************
int getCompileYear( const char *p )
{
    int i;

    i = atoi( p + 7 );
    if( i < 1980 || i > 2107 ) i = 2019;

    return i;
}

///*********************************************************************
// モジュール名 getCompileMonth
// 処理概要     コンパイルした時の月を取得
// 引数　       __DATE__ のポインタ
// 戻り値       月
///*********************************************************************
int getCompileMonth( const char *p )
{
    int i, r;

    for( i=0; i<12; i++ ) {
        r = strncmp( monthStr + i * 3, p, 3 );
        if( r == 0 ) return i + 1;
    }
    return 1;
}

///*********************************************************************
// モジュール名 getCompileDay
// 処理概要     コンパイルした時の日を取得
// 引数　       __DATE__ のポインタ
// 戻り値       日
///*********************************************************************
int getCompileDay( const char *p )
{
    int i;

    i = atoi( p + 4 );
    if( i < 1 || i > 31 ) i = 1;

    return i;
}

///*********************************************************************
// モジュール名 getCompileHour
// 処理概要     コンパイルした時の時を取得
// 引数　       __TIME__ のポインタ
// 戻り値       時
///*********************************************************************
int getCompileHour( const char *p )
{
    int i;

    i = atoi( p );
    if( i < 0 || i > 23 ) i = 0;

    return i;
}

///*********************************************************************
// モジュール名 getCompilerMinute
// 処理概要     コンパイルした時の分を取得
// 引数　       __TIME__ のポインタ
// 戻り値       分
///*********************************************************************
int getCompilerMinute( const char *p )
{
    int i;

    i = atoi( p + 3 );
    if( i < 0 || i > 59 ) i = 0;

    return i;
}

///*********************************************************************
// モジュール名 getCompilerSecond
// 処理概要     コンパイルした時の秒を取得
// 引数　       __TIME__ のポインタ
// 戻り値       秒
///*********************************************************************
int getCompilerSecond( const char *p )
{
    int i;

    i = atoi( p + 6 );
    if( i < 0 || i > 59 ) i = 0;

    return i;
}

///*********************************************************************
// char型データの値をlong型変数に2進数で変換
// 引数　 unsigned char 変換元の8bitデータ
// 戻り値 unsigned long 変換先の変数(0～11111111) ※0か1しかありません
///*********************************************************************
unsigned long convertBCD_CharToLong( unsigned char hex )
{
    int             i;
    unsigned long  l = 0;

    for( i=0; i<8; i++ ) {
        l *= 10;
        if( hex & 0x80 ) l += 1;
        hex <<= 1;
    }

    return l;
}

///*********************************************************************
// クロスライン検出処理
// 戻り値 0:クロスラインなし 1:あり
///*********************************************************************
int check_crossline( void )
{
    unsigned char b;
    int ret;

    ret = 0;
    b = sensor_inp(MASK3_3);
    if( b == 0xe7 ) {
        ret = 1;
    }
    return ret;
}

///*********************************************************************
// 右ハーフライン検出処理
// 戻り値 0:なし 1:あり
///*********************************************************************
int check_rightline( void )
{
    unsigned char b;
    int ret;

    ret = 0;
    b = sensor_inp(MASK4_4);
    if( b==0x0f || b==0x1f || b==0x3f || b==0x7f) {
        ret = 1;
    }
    return ret;
}

///*********************************************************************
// 左ハーフライン検出処理
// 戻り値 0:なし 1:あり
///*********************************************************************
int check_leftline( void )
{
    unsigned char b;
    int ret;

    ret = 0;
    b = sensor_inp(MASK4_4);
    if( b==0xf0 || b==0xf8 || b==0xfc || b==0xfe) {
        ret = 1;
    }
    return ret;
}

///********************************************************************
// 指定した行数の８点を取得し、しきい値を自動で調整し2進数8ビットに変換する
// 引数：行数(0-119), しきい値, 差
// 戻り値：センサの８ビット
///********************************************************************
unsigned char shikiichi_henkan( int gyou, int s, int sa )
{
    int max, min, i, shiki;
    int d[8];
    int sa_7_6, sa_6_5, sa_5_4, sa_4_3, sa_3_2, sa_2_1, sa_1_0;
    unsigned char ret;

    d[7] = getImage(  31, gyou);
    d[6] = getImage(  43, gyou);
    d[5] = getImage(  54, gyou);
    d[4] = getImage(  71, gyou);
    d[3] = getImage(  88, gyou);
    d[2] = getImage( 105, gyou);
    d[1] = getImage( 116, gyou);
    d[0] = getImage( 128, gyou);

    min = max = d[0];
    for( i=1; i<8; i++ ) {
        if( max <= d[i] ) {
            max = d[i]; // 8個のうち、最大を見つける
        }
        if( min >= d[i] ) {
            min = d[i]; // 8個のうち、最小を見つける
        }
    }

    // 隣同士の差の絶対値
    sa_7_6 = abs( d[7] - d[6] );
    sa_6_5 = abs( d[6] - d[5] );
    sa_5_4 = abs( d[5] - d[4] );
    sa_4_3 = abs( d[4] - d[3] );
    sa_3_2 = abs( d[3] - d[2] );
    sa_2_1 = abs( d[2] - d[1] );
    sa_1_0 = abs( d[1] - d[0] );

    if( max >= s ) {
        // 最大値がs以上なら、sをしきい値とする
        shiki = s;
    } else if( sa_7_6 >= sa || sa_6_5 >= sa || sa_5_4 >= sa ||
                sa_4_3 >= sa || sa_3_2 >= sa || sa_2_1 >= sa || sa_1_0 >= sa ) {
        // 隣同士の差が１つでも「差」以上なら、８点の（最大値－最小値）×0.7 + 最小値　をしきい値とする
        shiki =  ( max - min ) * 7 / 10 + min;
    } else {
        // 当てはまらなければ、しきい値を256とする、すなわちすべて0となる
        shiki = 256;
    }

    // d[7]～d[0]をbit7～bit0に割り当てる
    ret = 0;
    for( i=7; i>=0; i-- ) {
        ret <<= 1;
        ret |= (d[i] >= shiki ? 1 : 0);
    }

    return ret;
}

///********************************************************************
// イメージ領域のx,yの値(0-255)を取得
// 引数：x列数(0-159) , y行数(0-119)
// 戻り値：0～255
///********************************************************************
char getImage( int ix, int iy)
{
    return ImageData_B[ ix + 160U * iy ];
}

//------------------------------------------------------------------//
// End of file
//------------------------------------------------------------------//
