/**
 * This sample program shows a PID controller for line following.
 *
 * Robot construction: Educator Vehicle
 *
 * References:
 * http://robotsquare.com/wp-content/uploads/2013/10/45544_educator.pdf
 * http://thetechnicgear.com/2014/03/howto-create-line-following-robot-using-mindstorms/
 */

#include "ev3api.h"
#include "app.h"
#include "stdlib.h"
#include <stdio.h>
#include "math.h"

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

static FILE *bt = NULL;

FILE *file;
const char* logfilename = "/wro_result.txt";
/**
 * Define the connection ports of the sensors and motors.
 * By default, this application uses the following ports:
 * Touch sensor: Port 2
 * Color sensor: Port 3
 * Left motor:   Port B
 * Right motor:  Port C
 */
int battery;

static const sensor_port_t  PortSensorColor1 = EV3_PORT_1;
static const sensor_port_t  PortSensorColor2 = EV3_PORT_2; //左
static const sensor_port_t  PortSensorColor3 = EV3_PORT_3; //右
static const sensor_port_t  PortSensorColor4 = EV3_PORT_4; //超音波はここ

static const motor_port_t   PortMotorArmUp   = EV3_PORT_A; //オブジェをとる
static const motor_port_t   PortMotorLeft    = EV3_PORT_B; //lb
static const motor_port_t   PortMotorRight   = EV3_PORT_C; //rc
static const motor_port_t   PortMotorArmDown = EV3_PORT_D; //オブジェを落とす


void stopping();
void gain_set(int power, float *p_gain, float *d_gain);
void gain_set_pid(int power, float *p_gain, float *d_gain, float *i_gain);
void linetrace_cm_pd_SP(float cm, int power, bool_t stop);

static void button_clicked_handler(intptr_t button) {
    switch(button) {
    case BACK_BUTTON:
#if !defined(BUILD_MODULE)
        syslog(LOG_NOTICE, "Back button clicked.");
#endif
        break;
    }
}
    
void stopping(){
    while(ev3_button_is_pressed(ENTER_BUTTON) == false) {}    
    tslp_tsk(2000*MSEC);
}

void gain_set(int power, float *p_gain, float *d_gain){
    *p_gain = 0.3;
    *d_gain = 10;
    if(power > 0 && power <= 10){
        *p_gain = 0.7;   
        *d_gain = 55;   
    }
    if(power > 10 && power <= 20){
        *p_gain = 0.75;   
        *d_gain = 55;   
    }
    if(power == 24){
        *p_gain = -0.5;   //power24だけrightのセンサーでライントレース
        *d_gain = 60;   
    }
    if(power > 20 && power <= 30 && power != 24){
        *p_gain = 0.4;   
        *d_gain = 80;   
    }
    if(power > 30 && power <= 40){
        *p_gain = 0.25;   
        *d_gain = 70;   
    }
    if(power > 40 && power <= 50){
        *p_gain = 0.15;   
        *d_gain = 60;   
    }
    if(power > 50 && power <= 60){
        *p_gain = 0.16;   
        *d_gain = 70;   
    }
    if(power > 60 && power <= 70){
        *p_gain = 0.15;   
        *d_gain = 15;   
    }
    if(power > 70 && power <= 80){
        *p_gain = 0.15;
        *d_gain = 10;
    }
}

void gain_set_pid(int power, float *p_gain, float *d_gain, float *i_gain){
    *p_gain = 1.4;
    *d_gain = 100;
    *i_gain = 0.00001;
    if(power > 40){
        *p_gain = 1.2;   
        *d_gain = 200;
        *i_gain = 0.000000;
    }
    if(power > 50){
        *p_gain = 2;   
        *d_gain = 1500;
        *i_gain = 0.000000;
    }
    if(power > 60){
        *p_gain = 1;   
        *d_gain = 600;
        *i_gain = 0.00000;
    }
    if(power > 65){
        *p_gain = 1;   
        *d_gain = 300;
        *i_gain = 0.00000;
    }
    
    
    
}


void linetrace_cm_pd_SP(float cm, int power, bool_t stop){
    power = -power;
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    int now_angle_lb = 0;
    int now_angle_rc = 0;
    int average = 0;
    int lb_power;
    int rc_power;
    int now_reflect_2; 
    int now_reflect_3; 
    int last_diff = 0;
    int diff = 0;
    float d;
    int steering;
    float p_gain;
    float d_gain;
    gain_set(power, &p_gain, &d_gain);

    now_reflect_2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
    now_reflect_3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
    diff = now_reflect_2 - now_reflect_3;
    while (true) {
        
        now_reflect_2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
        now_reflect_3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
        now_angle_lb = abs(ev3_motor_get_counts(EV3_PORT_B));
        now_angle_rc = abs(ev3_motor_get_counts(EV3_PORT_C));
        average = (now_angle_lb + now_angle_rc) / 2;
        last_diff = diff;
        diff = now_reflect_2 - now_reflect_3;
        d = (diff - last_diff);
        steering = diff * 0.3 + d * 100;
        if(steering > 0) {
            lb_power = power;
            rc_power = power - (power * steering / 50);
            lb_power = -lb_power;
        }
        else {
            lb_power = power + (power * steering / 50);
            rc_power = power;
            lb_power = -lb_power;
        }
        ev3_motor_set_power(EV3_PORT_B, lb_power);
        ev3_motor_set_power(EV3_PORT_C, rc_power);

        if (average >= ROBOT1CM*cm) break;
    }
    ev3_motor_stop(EV3_PORT_B, stop);
    ev3_motor_stop(EV3_PORT_C, stop);
}

void linetrace_new(float cm, int power, bool_t stop){
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    int now_angle_lb = 0;
    int now_angle_rc = 0;
    int average = 0;
    int lb_power;
    int rc_power;
    int now_reflect_2; 
    int now_reflect_3; 
    int last_diff = 0;
    int diff = 0;
    float d;
    float i = 0;
    int steering;
    float p_gain;
    float d_gain;
    float i_gain;
    float max_power = 40;
    gain_set(power, &p_gain, &d_gain);

    now_reflect_2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
    now_reflect_3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
    diff = now_reflect_2 - now_reflect_3;
    while (true) {
        
        now_reflect_2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
        now_reflect_3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
        now_angle_lb = abs(ev3_motor_get_counts(EV3_PORT_B));
        now_angle_rc = abs(ev3_motor_get_counts(EV3_PORT_C));
        average = (now_angle_lb + now_angle_rc) / 2;
        power = (max_power / (20*ROBOT1CM) / (20*ROBOT1CM)) * average * average + 30;
        if (power > 70) power = 70;
        gain_set_pid(power, &p_gain, &d_gain, &i_gain);
        last_diff = diff;
        diff = now_reflect_2 - now_reflect_3;
        i = diff + i;
        d = (diff - last_diff);
        steering = diff * p_gain + d * d_gain + i * i_gain;
        if(steering > 0) {
            lb_power = power;
            rc_power = power - (power * steering / 50);
            lb_power = -lb_power;
        }
        else {
            lb_power = power + (power * steering / 50);
            rc_power = power;
            lb_power = -lb_power;
        }
        ev3_motor_set_power(EV3_PORT_B, lb_power);
        ev3_motor_set_power(EV3_PORT_C, rc_power);

        if (average >= ROBOT1CM*cm) break;
    }
    ev3_motor_stop(EV3_PORT_B, stop);
    ev3_motor_stop(EV3_PORT_C, stop);
}

void linetrace_rgb(float cm, int power, bool_t stop){
    rgb_raw_t rgb_val;//カラーセンサーの値を保存するために必要な変数(必須)
    float red2 = 0;
    float green2 = 0;
    float blue2 = 0;
    float red3 = 0;
    float green3 = 0;
    float blue3 = 0;
    float judgement = 0;
    int lb_power;
    int rc_power;
    int steering;
    int last_diff = 0;
    int diff = 0;
    float d;
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    int now_angle_lb = 0;
    int now_angle_rc = 0;
    int average = 0;

    int before_average = 0;
    int zure = 0;
    float theta = 0;
    float x = 0;
    float y = 0;
    float now_x = 0;
    float now_y = 0;
    int count = 0;
    float a = 0;


    ev3_color_sensor_get_rgb_raw(EV3_PORT_2, &rgb_val);
    red2 = rgb_val.r;
    green2 = rgb_val.g;
    blue2 = rgb_val.b;
    ev3_color_sensor_get_rgb_raw(EV3_PORT_3, &rgb_val);
    red3 = rgb_val.r;
    green3 = rgb_val.g;
    blue3 = rgb_val.b;
    judgement = (red2 + green2 + blue2) - (red3 + green3 + blue3);
    diff = judgement;
    while (true){
        now_angle_lb = abs(ev3_motor_get_counts(EV3_PORT_B));
        now_angle_rc = abs(ev3_motor_get_counts(EV3_PORT_C));

        
        zure = now_angle_lb - now_angle_rc;
        theta = zure * 0.181818181818;
        
        average = (now_angle_lb + now_angle_rc) / 2;
        count = count + 1;
        if(count >= 100){
            a = average - before_average;
            y = sin(theta) * a;
            x = cos(theta) * a;
            now_x = x + now_x;
            now_y = y + now_y;
            before_average = average;
            count = 0;
        }
        

        ev3_color_sensor_get_rgb_raw(EV3_PORT_2, &rgb_val);
        red2 = rgb_val.r;
        green2 = rgb_val.g;
        blue2 = rgb_val.b;
        ev3_color_sensor_get_rgb_raw(EV3_PORT_3, &rgb_val);
        red3 = rgb_val.r;
        green3 = rgb_val.g;
        blue3 = rgb_val.b;
        judgement = ((red2 + green2 + blue2) - (red3 + green3 + blue3)) / 7.5;

        last_diff = diff;
        diff = judgement;
        d = (diff - last_diff);
        steering = judgement * 0.4 + d * 0;
        //steering = judgement * 0.4 + d * 80  po=18;
        
        if(steering > 0) {
            lb_power = power;
            rc_power = power - (power * steering / 50);
            lb_power = -lb_power;
        }
        else {
            lb_power = power + (power * steering / 50);
            rc_power = power;
            lb_power = -lb_power;
        }
        ev3_motor_set_power(EV3_PORT_B, lb_power);
        ev3_motor_set_power(EV3_PORT_C, rc_power);
        fprintf(bt,"%f,%f,%f\r\n", now_x, now_y, theta);
        if (now_x >= ROBOT1CM * cm) break;
    }
    ev3_motor_stop(EV3_PORT_B, stop);
    ev3_motor_stop(EV3_PORT_C, stop);

}

void straight_dr(float cm, int power, bool_t stop){
    rgb_raw_t rgb_val;//カラーセンサーの値を保存するために必要な変数(必須)
    float red2 = 0;
    float green2 = 0;
    float blue2 = 0;
    float red3 = 0;
    float green3 = 0;
    float blue3 = 0;
    float judgement = 0;
    int lb_power;
    int rc_power;
    int steering;
    int last_diff = 0;
    int diff = 0;
    float d;
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    int now_angle_lb = 0;
    int now_angle_rc = 0;
    int average = 0;

    int before_average = 0;
    int zure = 0;
    float theta = 0;
    float x = 0;
    float y = 0;
    float now_x = 0;
    float now_y = 0;
    int count = 0;
    float a = 0;


    ev3_color_sensor_get_rgb_raw(EV3_PORT_2, &rgb_val);
    red2 = rgb_val.r;
    green2 = rgb_val.g;
    blue2 = rgb_val.b;
    ev3_color_sensor_get_rgb_raw(EV3_PORT_3, &rgb_val);
    red3 = rgb_val.r;
    green3 = rgb_val.g;
    blue3 = rgb_val.b;
    judgement = (red2 + green2 + blue2) - (red3 + green3 + blue3);
    diff = judgement;
    while (true){
        now_angle_lb = abs(ev3_motor_get_counts(EV3_PORT_B));
        now_angle_rc = abs(ev3_motor_get_counts(EV3_PORT_C));

        
        zure = now_angle_lb - now_angle_rc;
        theta = zure * 0.181818181818;
        
        average = (now_angle_lb + now_angle_rc) / 2;
        count = count + 1;
        if(count >= 100){
            a = average - before_average;
            y = sin(theta) * a;
            x = cos(theta) * a;
            now_x = x + now_x;
            now_y = y + now_y;
            before_average = average;
            count = 0;
        }
        

        ev3_color_sensor_get_rgb_raw(EV3_PORT_2, &rgb_val);
        red2 = rgb_val.r;
        green2 = rgb_val.g;
        blue2 = rgb_val.b;
        ev3_color_sensor_get_rgb_raw(EV3_PORT_3, &rgb_val);
        red3 = rgb_val.r;
        green3 = rgb_val.g;
        blue3 = rgb_val.b;
        judgement = ((red2 + green2 + blue2) - (red3 + green3 + blue3)) / 7.5;

        last_diff = diff;
        diff = judgement;
        d = (diff - last_diff);
        steering = judgement * 0.4 + d * 0;
        //steering = judgement * 0.4 + d * 80  po=18;
        
        if(steering > 0) {
            lb_power = power;
            rc_power = power - (power * steering / 50);
            lb_power = -lb_power;
        }
        else {
            lb_power = power + (power * steering / 50);
            rc_power = power;
            lb_power = -lb_power;
        }
        ev3_motor_set_power(EV3_PORT_B, lb_power);
        ev3_motor_set_power(EV3_PORT_C, rc_power);
        fprintf(bt,"%f,%f,%f\r\n", now_x, now_y, theta);
        if (now_x >= ROBOT1CM * cm) break;
    }
    ev3_motor_stop(EV3_PORT_B, stop);
    ev3_motor_stop(EV3_PORT_C, stop);

}

void turn(int angle, int lb_power, int rc_power) {
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    int lb_sign = lb_power / abs(lb_power);
    int rc_sign = rc_power / abs(rc_power);
    int now_right_angle = 0;
    int now_left_angle = 0;
    int average = 0;
    int maximum = 80;
    int points = 40;
    float turn_num = 0.1505;
    if (abs(lb_power) == 0 || abs(rc_power) == 0) {
        turn_num = 0.152;
    }
    if (abs(lb_power) >= abs(rc_power)) maximum = abs(lb_power);
    if (abs(rc_power) > abs(lb_power)) maximum = abs(rc_power);
    float changing_power = 15;
    int goal_angle = angle*turn_num*ROBOT1CM;
    while (true) {
        now_left_angle = abs(ev3_motor_get_counts(EV3_PORT_B));
        now_right_angle = abs(ev3_motor_get_counts(EV3_PORT_C));
        average = (now_left_angle + now_right_angle) / 2;
        
        if (changing_power <= 15) changing_power = 15;
        if (lb_power == 0) {
            if (changing_power < maximum && goal_angle - (points*turn_num*ROBOT1CM) > now_right_angle) changing_power = changing_power + 0.005;
            if (goal_angle - ((points + 5)*turn_num*ROBOT1CM) <= now_right_angle) changing_power = changing_power - 0.14;
            if (changing_power <= 10) changing_power = 10;
            if (goal_angle <= now_right_angle) break; 
            rc_power = changing_power*rc_sign;
            ev3_motor_set_power(EV3_PORT_C, rc_power);
        }
        if (rc_power == 0) {
            if (changing_power < maximum && goal_angle - (points*turn_num*ROBOT1CM) > now_left_angle) changing_power = changing_power + 0.005;
            if (goal_angle - ((points + 5)*turn_num*ROBOT1CM) <= now_left_angle) changing_power = changing_power - 0.14;
            if (changing_power <= 10) changing_power = 10;
            if (goal_angle <= now_left_angle) break; 
            lb_power = -changing_power*lb_sign;
            ev3_motor_set_power(EV3_PORT_B, lb_power);
        }
        if (lb_power != 0 && rc_power != 0){
            if (changing_power < maximum && goal_angle - (points*turn_num*ROBOT1CM) > average) changing_power = changing_power + 0.004;
            if (goal_angle - (points*turn_num*ROBOT1CM) <= now_right_angle) changing_power = changing_power - 0.14;
            if (changing_power <= 10) changing_power = 10;
            if (goal_angle <= average) break; 
            rc_power = changing_power*rc_sign;
            lb_power = -changing_power*lb_sign;
            ev3_motor_set_power(EV3_PORT_C, rc_power);
            ev3_motor_set_power(EV3_PORT_B, lb_power);
        }  
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
}

void straight(float cm, int power){
    power = -power;
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    //if (power == 30) power = 40;
    int steer = 0;
    int lb_power;
    int rc_power;
    int now_right_angle = 0;
    int now_left_angle = 0;
    int now_angle = 0; 
    int diff = 0;
    int maximum = abs(power);
    float changing_power = 14;
    int sign = power/abs(power);
    int goal_angle = cm * ROBOT1CM;
    while (true){
        now_left_angle = abs(ev3_motor_get_counts(EV3_PORT_B));
        now_right_angle = abs(ev3_motor_get_counts(EV3_PORT_C));
        now_angle = (now_right_angle + now_left_angle) / 2;
        diff = now_right_angle - now_left_angle;
        steer = (diff*4);
        if(steer > 0) {
            lb_power = changing_power;
            rc_power = changing_power - (changing_power * steer / 50);
            rc_power = sign*rc_power;
            lb_power = -sign*lb_power;
        }
        else {
            lb_power = changing_power + (changing_power * steer / 50);
            rc_power = changing_power;
            rc_power = sign*rc_power;
            lb_power = -sign*lb_power;
        }
        ev3_motor_set_power(EV3_PORT_B, lb_power);
        ev3_motor_set_power(EV3_PORT_C, rc_power);
        if (changing_power < maximum && goal_angle - (6*ROBOT1CM) > now_angle) changing_power = changing_power + 0.005;
        if (goal_angle - (6*ROBOT1CM) <= now_angle) changing_power = changing_power - 0.01;
        if (changing_power <= 14) changing_power = 14;
        if (goal_angle <= now_angle) break;   
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
}

void diagonal(float x, float y, float finish_degree, int power){
    float theta;
    float a;
    if (x > 0) theta = atan2(y, x) * (180 / PAI);
    else theta = 180 - abs(atan2(y, x) * (180 / PAI));
    a = sqrt(pow(x, 2) + pow(y, 2));
    if (x * y > 0) turn(90 - abs(theta), 30, -30);
    else turn(90 - abs(theta), -30, 30);
    if (y > 0) straight(a, abs(power));
    else straight(a, -abs(power));
    if ((finish_degree - (90 - abs(theta))) > 0) turn(finish_degree - (90 - abs(theta)), 30, -30);
    else turn(abs(finish_degree - (90 - abs(theta))), 30, -30);
    fprintf(bt, "%f\r\n", theta);

}

void main_task(intptr_t unused) {

    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);

    /* Register button handlers */
    ev3_button_set_on_clicked(BACK_BUTTON, &button_clicked_handler, BACK_BUTTON);

    /* Configure motors */
    ev3_motor_config(PortMotorLeft, MEDIUM_MOTOR);
    ev3_motor_config(PortMotorRight, MEDIUM_MOTOR);
    ev3_motor_config(PortMotorArmDown, MEDIUM_MOTOR);
    ev3_motor_config(PortMotorArmUp, MEDIUM_MOTOR);

    /* Configure sensors */
    ev3_sensor_config(PortSensorColor1, COLOR_SENSOR);
    ev3_sensor_config(PortSensorColor2, COLOR_SENSOR);
    ev3_sensor_config(PortSensorColor3, COLOR_SENSOR);
    ev3_sensor_config(PortSensorColor4, COLOR_SENSOR);
    
    fprintf(bt, "----GAME_START----\r\n");
    file=fopen(logfilename,"a");//ファイルをオープン(名前の指定)
    fprintf(file, "----GAME_START----\r\n");
    fclose(file);

    battery = ev3_battery_voltage_mV();
    fprintf(bt, "BATTERY:%d\r\n", battery);
    file=fopen(logfilename,"a");//ファイルをオープン(名前の指定)
    fprintf(file, "BATTERY:%d\r\n", battery);
    fclose(file);
    
    

    /* ここからコーディング */
    stopping();
    straight(30, 30);
    stopping();
    turn(90, 30, -30);
    stopping();
    linetrace_cm_pd_SP(40, 30, true);
    stopping();





}