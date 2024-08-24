#include "ev3api.h"
#include "app.h"
#include "stdlib.h"
#include <stdio.h>
#include "math.h"
#include "stopping.h"
#include "straight.h"
#include "turn.h"
#include "linetrace.h"

armmode_new_t now_mode;
void gain_set_rgb(int power, float *p_gain, float *d_gain){
    *p_gain = 0.3;
    *d_gain = 10;
    if(power > 0 && power <= 10){
        *p_gain = 0.18;   
        *d_gain = 80;   
    }
    if(power > 10 && power <= 20){
        *p_gain = 0.25;   
        *d_gain = 80;   
        if(now_mode == THREE || now_mode == FOUR || now_mode == ALLUP) {
            *p_gain = 0.16;   
            *d_gain = 80; 
        }
    }
    if(power == 24){
        *p_gain = -0.5;   //power24だけrightのセンサーでライントレース
        *d_gain = 60;   
    }
    if(power > 20 && power <= 30 && power != 24){
        *p_gain = 0.14;   
        *d_gain = 80;   
        if(now_mode == THREE || now_mode == FOUR || now_mode == ALLUP) {
            *p_gain = 0.1;   
            *d_gain = 80; 
        }
    }
    if(power > 30 && power <= 40){
        *p_gain = 0.08;   
        *d_gain = 80;   
    }
    if(power > 40 && power <= 50){
        *p_gain = 0.05;   
        *d_gain = 100;   
    }
    if(power > 50 && power <= 60){
        *p_gain = 0.03;   
        *d_gain = 100;   
    }
    if(power > 60 && power <= 70){
        *p_gain = 0.02;   
        *d_gain = 60;   
    }
    if(power > 70 && power <= 80){
        *p_gain = 0.6;
        *d_gain = 10;
    }
}

void linetrace_cm_rgb_pd_SP(float cm, int power, bool_t stop){
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    int now_angle_lb = 0;
    int now_angle_rc = 0;
    int average = 0;
    int lb_power;
    int rc_power;
    rgb_raw_t rgb_val;//カラーセンサーの値を保存するために必要な変数(必須)
    float red2 = 0;
    float green2 = 0;
    float blue2 = 0;
    float red3 = 0;
    float green3 = 0;
    float blue3 = 0;
    float judgement2 = 0;
    float judgement3 = 0;
    int last_diff = 0;
    int diff = 0;
    float d;
    int steering;
    float p_gain;
    float d_gain;
    gain_set_rgb(power, &p_gain, &d_gain);

    ev3_color_sensor_get_rgb_raw(EV3_PORT_2, &rgb_val);
    red2 = rgb_val.r;
    green2 = rgb_val.g;
    blue2 = rgb_val.b;
    ev3_color_sensor_get_rgb_raw(EV3_PORT_3, &rgb_val);
    red3 = rgb_val.r;
    green3 = rgb_val.g;
    blue3 = rgb_val.b;
    judgement2 = (red2 + green2 + blue2);
    judgement3 = (red3 + green3 + blue3);
    diff = judgement2 - judgement3;

    
    
    while (true) {
        
        ev3_color_sensor_get_rgb_raw(EV3_PORT_2, &rgb_val);
        red2 = rgb_val.r;
        green2 = rgb_val.g;
        blue2 = rgb_val.b;
        ev3_color_sensor_get_rgb_raw(EV3_PORT_3, &rgb_val);
        red3 = rgb_val.r;
        green3 = rgb_val.g;
        blue3 = rgb_val.b;
        now_angle_lb = abs(ev3_motor_get_counts(EV3_PORT_B));
        now_angle_rc = abs(ev3_motor_get_counts(EV3_PORT_C));
        average = (now_angle_lb + now_angle_rc) / 2;
        judgement2 = (red2 + green2 + blue2);
        judgement3 = (red3 + green3 + blue3);
        last_diff = diff;
        diff = judgement2 - judgement3;
        d = (diff - last_diff);
        steering = diff * p_gain + d * d_gain;
        average = (now_angle_lb + now_angle_rc) / 2;
        if(steering > 0) {
            lb_power = power;
            rc_power = power - (power * steering / 50);
            //if(lb_power < 7) lb_power = 7;
            //if(rc_power < 7) rc_power = 7;
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
    if (stop == true){
        ev3_motor_stop(EV3_PORT_B, stop);
        ev3_motor_stop(EV3_PORT_C, stop);
    }
}
