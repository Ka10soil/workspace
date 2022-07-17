/** ヘッダファイルの include **/
#include <ev3api.h>
#include "app.h"


/** ファイル内だけで使う関数の宣言 **/
static void prepareDevices(void);
static void cleanUpDevices(void);
static void ISR_button(intptr_t exif);


/** ファイル内だけで使う定義 **/
#define REFLECT_TARGET_VALUE    ((uint8_t)40U)
#define DISTANCE_TARGET_VALUE   ((int16_t)10)
#define POWER_LINETRACE         ((int16_t)30)

#define PORT_SENSOR_TOUCH_LEFT  EV3_PORT_1
#define PORT_SENSOR_TOUCH_RIGHT EV3_PORT_4
#define PORT_SENSOR_COLOR       EV3_PORT_2
#define PORT_SENSOR_ULTRASONIC  EV3_PORT_3
#define PORT_MOTOR_LEFT         EV3_PORT_B
#define PORT_MOTOR_RIGHT        EV3_PORT_C


/** 変数の定義 **/
/* 中央ボタンが押されたら trueとなる変数 */
static bool_t isPressed = false; 


/** すべてのファイルから実行できる関数の定義 **/
/** メインタスク関数 **/
void main_task(intptr_t exinf) {
    /* この関数(main_task)の中でしか使えない変数 */
    int16_t power = POWER_LINETRACE;
    int16_t turnRatio = 0;
    uint8_t reflect = 0U;

    /* モーターやセンサーの設定を行う関数を実行する */
    prepareDevices();

    /* ボタンが押されたときのイベントハンドラーをEV3RTに教える */
    /*  [1]:ENTER_BUTTON: 中央ボタンが押されたときに動かす */
    /*  [2]:ISR_button  : イベントハンドラーは ISR_button という関数にする */
    /*  [3]:ENTER_BUTTON: 実行されるとき、引数にボタン番号が入るようにする */
    (void)ev3_button_set_on_clicked(ENTER_BUTTON, &ISR_button, (intptr_t)ENTER_BUTTON);

    /* ボタンが押されていない状態から開始 */
    isPressed = false;

    /* 中央ボタンが押されていない間はくりかえす  */
    while (false == isPressed) {
        /* カラーセンサーから反射光強さを得る */
        reflect = ev3_color_sensor_get_reflect(PORT_SENSOR_COLOR);

        /* turnRatio を決める */
        if (reflect < REFLECT_TARGET_VALUE) {
            turnRatio = -50;
        } else if (REFLECT_TARGET_VALUE < reflect) {
            turnRatio = 50;
        } else {
            turnRatio = 0;
        }

        /* モーターに turnRatio を設定する */
        (void)ev3_motor_steer(PORT_MOTOR_LEFT, PORT_MOTOR_RIGHT, power, turnRatio);
    }

    /* モーターやセンサーの終了時処理を行う */
    cleanUpDevices();

    /* 実行中のタスク(main_task)を終了する */
    ext_tsk();
}


/** ファイルの中だけで実行できる関数の定義 **/
static void prepareDevices(void) {
    /* モーターの設定を行う */
    (void)ev3_motor_config(PORT_MOTOR_LEFT, LARGE_MOTOR);
    (void)ev3_motor_config(PORT_MOTOR_RIGHT, LARGE_MOTOR);

    /* センサーの設定を行う */
    (void)ev3_sensor_config(PORT_SENSOR_TOUCH_LEFT, TOUCH_SENSOR);
    (void)ev3_sensor_config(PORT_SENSOR_COLOR, COLOR_SENSOR);
    (void)ev3_sensor_config(PORT_SENSOR_ULTRASONIC, ULTRASONIC_SENSOR);

    /* 画面を消すために、画面いっぱいの白四角を描く */
    (void)ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    /* フォントの大きさを EV3_FONT_MEDIUM にする */
    (void)ev3_lcd_set_font(EV3_FONT_MEDIUM);

    /* スピーカーの音量を最大にする */
    (void)ev3_speaker_set_volume(100U);
}


static void cleanUpDevices(void) {
    /* モーターを止める */
    (void)ev3_motor_stop(PORT_MOTOR_LEFT, true);
    (void)ev3_motor_stop(PORT_MOTOR_RIGHT, true);

    /* センサーポートを未接続にする */
    (void)ev3_sensor_config(PORT_SENSOR_TOUCH_LEFT, NONE_SENSOR);
    (void)ev3_sensor_config(PORT_SENSOR_COLOR, NONE_SENSOR);
    (void)ev3_sensor_config(PORT_SENSOR_ULTRASONIC, NONE_SENSOR);

    /* 画面を消すために、画面いっぱいの白四角を描く */
    (void)ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
}


static void ISR_button(intptr_t exif) {
    /* ボタンのイベントハンドラーの場合、引数(intptr_t exif)には */
    /* ボタンの番号を入れると決めたので、exifには押されたボタン番号が入っている */
    button_t buttonNo = (button_t)exif;

    /* 押されたボタンによって、分岐する */
    switch(buttonNo) {
    case ENTER_BUTTON:  /* 中央ボタンを押されたとき */
        isPressed = true;
        break;
    default: /* 上のどれにも当たらない場合 */
        break;
    }
}