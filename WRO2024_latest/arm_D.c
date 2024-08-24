#include "ev3api.h"
#include "app.h"
#include "stdlib.h"
#include <stdio.h>
#include "math.h"
#include "stopping.h"
#include "turn.h"
#include "arm_A.h"


extern int now_arm_angle;
armmode_new_t now_mode;
//  関数
void arm_D(armmode_new_t mode) {
    now_arm_angle = ev3_motor_get_counts(EV3_PORT_D);
    now_mode = mode;
    switch (mode) {
        case DOWN:
            if(now_arm_angle <= 5)ev3_motor_set_power(EV3_PORT_D, 30);
            else ev3_motor_set_power(EV3_PORT_D, -80);
            break;
        case UP:
            if(now_arm_angle <= 310)ev3_motor_set_power(EV3_PORT_D, 80);
            else ev3_motor_set_power(EV3_PORT_D, -80);
            break;
        case ONE:
            if(now_arm_angle <= 265)ev3_motor_set_power(EV3_PORT_D, 80);
            else ev3_motor_set_power(EV3_PORT_D, -80);
            break;
        case TWO:
            if(now_arm_angle <= 480)ev3_motor_set_power(EV3_PORT_D, 80);
            else ev3_motor_set_power(EV3_PORT_D, -80);
            break;
        case THREE:
            if(now_arm_angle <= 695)ev3_motor_set_power(EV3_PORT_D, 80);
            else ev3_motor_set_power(EV3_PORT_D, -80);
            break;
        case FOUR:
            if(now_arm_angle <= 905)ev3_motor_set_power(EV3_PORT_D, 80);
            else ev3_motor_set_power(EV3_PORT_D, -80);
            break;
        case ALLUP:
            ev3_motor_set_power(EV3_PORT_D, 80);
            break;
        default:
            break;
    }
    now_mode = mode;
    //sta_cyc(ARM_CYC);
    while (true) {
        now_arm_angle = ev3_motor_get_counts(EV3_PORT_D);
        if(now_arm_angle <= 6 && now_arm_angle >= 4 && mode == DOWN) break;
        if(now_arm_angle <= 311 && now_arm_angle >= 309 && mode == UP) break;
        if(now_arm_angle <= 266 && now_arm_angle >= 264 && mode == ONE) break;
        if(now_arm_angle <= 481 && now_arm_angle >= 479 && mode == TWO) break;
        if(now_arm_angle <= 696 && now_arm_angle >= 694 && mode == THREE) break;
        if(now_arm_angle <= 906 && now_arm_angle >= 904 && mode == FOUR) break;
        if(now_arm_angle >= 980 && mode == ALLUP) break;
    }
    /*if(mode == SETNEW) {
        ev3_motor_stop(EV3_PORT_D, true);
        tslp_tsk(200*MSEC);
        ev3_motor_set_power(EV3_PORT_D, -10);
        while (true) {
            now_arm_angle = ev3_motor_get_counts(EV3_PORT_D);
            if(now_arm_angle <= -76 && now_arm_angle >= -78 && mode == SETNEW) break;
        }
    }*/
    if(mode == ONE || mode == TWO || mode == THREE || mode == FOUR ||mode == ALLUP || mode == DOWN || mode == UP)ev3_motor_stop(EV3_PORT_D, true);
    now_arm_angle = ev3_motor_get_counts(EV3_PORT_D);
    tslp_tsk(100*MSEC);
}