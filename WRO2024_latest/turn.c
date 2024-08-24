#include "ev3api.h"
#include "app.h"
#include "stdlib.h"
#include <stdio.h>
#include "math.h"
#include "stopping.h"
#include "turn.h"



//  関数
void turn(int angle, int lb_power, int rc_power){
    tslp_tsk(100*MSEC);
    if(angle > 100) tslp_tsk(100*MSEC);
    if (lb_power == 30) lb_power = lb_power + 35;
    if (lb_power == -30) lb_power = lb_power - 35;
    if (rc_power == 30) rc_power = rc_power + 35;
    if (rc_power == -30) rc_power = rc_power - 35;

    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    int lb_sign = lb_power / abs(lb_power);
    int rc_sign = rc_power / abs(rc_power);
    int now_right_angle = 0;
    int now_left_angle = 0;
    int average = 0;
    int maximum = 80;
    float points = 0.6;
    float turn_num = 0.1525;
    if (abs(lb_power) == 0) {
        turn_num = 0.157;
        if (rc_power < 0) {
            turn_num = 0.1595;
        }
    }
    if (abs(rc_power) == 0) {
        turn_num = 0.156;
    }
    if (lb_power > 0 && rc_power < 0) {
        turn_num = 0.1563;
        if (angle >= 180) {
            turn_num = 0.159;
        }
        //turn_num = 0.159;
    }
    if (lb_power < 0 && rc_power > 0) {
        turn_num = 0.1563;
        if (angle >= 180) {
            turn_num = 0.1553;
        }
        //turn_num = 0.1557;
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
            if (changing_power < maximum && goal_angle - (angle*points*turn_num*ROBOT1CM) > now_right_angle) changing_power = changing_power + 0.003;
            if (goal_angle - ((angle*points + 5)*turn_num*ROBOT1CM) <= now_right_angle) changing_power = changing_power - 0.003;
            if (changing_power <= 20) changing_power = 20;
            if (goal_angle <= now_right_angle) break; 
            rc_power = changing_power*rc_sign;
            ev3_motor_set_power(EV3_PORT_C, rc_power);
        }
        if (rc_power == 0) {
            if (changing_power < maximum && goal_angle - (angle*points*turn_num*ROBOT1CM) > now_left_angle) changing_power = changing_power + 0.005;
            if (goal_angle - ((angle*points + 5)*turn_num*ROBOT1CM) <= now_left_angle) changing_power = changing_power - 0.003;
            if (changing_power <= 20) changing_power = 20;
            if (goal_angle <= now_left_angle) break; 
            lb_power = -changing_power*lb_sign;
            ev3_motor_set_power(EV3_PORT_B, lb_power);
        }
        if (lb_power != 0 && rc_power != 0 && (angle <= 85 || angle >= 95)){
            if (changing_power < maximum && goal_angle - (angle*points*turn_num*ROBOT1CM) > average) changing_power = changing_power + 0.003;
            if (goal_angle - (angle*points*turn_num*ROBOT1CM) <= now_right_angle && angle <= 90) changing_power = changing_power - 0.021;
            if (goal_angle - (angle*points*turn_num*ROBOT1CM) <= now_right_angle && angle > 90) changing_power = changing_power - 0.004;
            
            
            if (changing_power <= 20) changing_power = 20;

            if (changing_power >= maximum) changing_power = maximum;

            if (goal_angle <= now_left_angle && goal_angle <= now_right_angle) break; 
            rc_power = changing_power*rc_sign;
            lb_power = -changing_power*lb_sign;
            if(now_right_angle <= goal_angle)ev3_motor_set_power(EV3_PORT_C, rc_power);
            else ev3_motor_stop(EV3_PORT_C, true);
            if(now_left_angle <= goal_angle)ev3_motor_set_power(EV3_PORT_B, rc_power);
            else ev3_motor_stop(EV3_PORT_B, true);
        }  
        if (lb_power != 0 && rc_power != 0 && (angle > 85 && angle < 95)){
            if (changing_power < maximum && goal_angle - (angle*points*turn_num*ROBOT1CM) > average && lb_power < 0) changing_power = changing_power + 0.008;
            if (changing_power < maximum && goal_angle - (angle*points*turn_num*ROBOT1CM) > average && lb_power > 0) changing_power = changing_power + 0.004;
            if (goal_angle - (angle*points*turn_num*ROBOT1CM) <= now_right_angle) changing_power = changing_power - 0.014;
            
            
            if (changing_power <= 20) changing_power = 20;


            if (changing_power >= maximum) changing_power = maximum;

            if (goal_angle <= now_left_angle && goal_angle <= now_right_angle) break; 
            rc_power = changing_power*rc_sign;
            lb_power = -changing_power*lb_sign;
            if(now_right_angle <= goal_angle)ev3_motor_set_power(EV3_PORT_C, rc_power);
            else ev3_motor_stop(EV3_PORT_C, true);
            if(now_left_angle <= goal_angle)ev3_motor_set_power(EV3_PORT_B, rc_power);
            else ev3_motor_stop(EV3_PORT_B, true);
        }  
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
    //fprintf(bt, "now_LR:%d :%d\r\nchange%f\r\n", now_left_angle, now_right_angle, changing_power);
}
