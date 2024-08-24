#include "ev3api.h"
#include "app.h"
#include "stdlib.h"
#include <stdio.h>
#include "math.h"
#include "stopping.h"
#include "turn.h"
#include "arm_A.h"


int now_arm_angle_A;

//  関数
void arm_A(armmode_new_t mode){
    now_arm_angle_A = ev3_motor_get_counts(EV3_PORT_A);
    switch (mode) {
        case OPEN:
            if(now_arm_angle_A <= 90)ev3_motor_set_power(EV3_PORT_A, 80);
            else ev3_motor_set_power(EV3_PORT_A, -80);
            break;
        case CLOSE:
            if(now_arm_angle_A <= -110)ev3_motor_set_power(EV3_PORT_A, 80);
            else ev3_motor_set_power(EV3_PORT_A, -80);

            break;
        default:
            break;
    }
    while (true) {
        now_arm_angle_A = ev3_motor_get_counts(EV3_PORT_A);
        if(now_arm_angle_A <= -199 && now_arm_angle_A >= -201 && mode == SET) break;
        if(now_arm_angle_A <= -219 && now_arm_angle_A >= -221 && mode == GET_OBJ_2) break;
        if(now_arm_angle_A <= -179 && now_arm_angle_A >= -181 && mode == GETDEBRIS) break;
        if(now_arm_angle_A <= 91 && now_arm_angle_A >= 89 && mode == OPEN) break;
        if(now_arm_angle_A <= -109 && now_arm_angle_A >= -111 && mode == CLOSE) break;

        
    }
    if(mode == SET)ev3_motor_stop(EV3_PORT_A, true);
    if(mode == GET_OBJ_2) ev3_motor_stop(EV3_PORT_A, true);
    if(mode == GETDEBRIS)ev3_motor_stop(EV3_PORT_A, true);
    if(mode == OPEN)ev3_motor_stop(EV3_PORT_A, true);
    if(mode == CLOSE)ev3_motor_stop(EV3_PORT_A, true);
    now_arm_angle_A = ev3_motor_get_counts(EV3_PORT_A);
}