最初の動き
arm_A(OPEN);
tslp_tsk(200*MSEC);


オブジェクトを読む位置までの動き
straight_on(-30);
tslp_tsk(500*MSEC);
ev3_motor_stop(EV3_PORT_A,true);
ev3_motor_stop(EV3_PORT_D, true);

straight(16, 30);
turn(90, 30, -30);
straight_on(20);
while (true){
    color_2 = ev3_color_sensor_get_color(EV3_PORT_2);
    if(color_2 == COLOR_RED) break;
}
straight(23, 30);


オブジェクト一つ目を取る動き
arm_D(UP);
straight(10, 30);
arm_A(SET);
arm_D(DOWN);
arm_A(CLOSE);
tslp_tsk(500*MSEC);
arm_D(UP);


オブジェクト二つ目以降を取る動
straight(10, 30);
arm_A(GET_OBJ_2);
arm_D(DOWN);
straight_on(10);
straight_off(0.8, false);
arm_A(CLOSE);
straight_off(1, true);
straight(11.7, -30);
arm_D(UP);












void turn(int angle, int lb_power, int rc_power){
    tslp_tsk(100*MSEC);
    if(angle > 100) tslp_tsk(200*MSEC);
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
    int points = 55;
    float turn_num = 0.1525;
    if (abs(lb_power) == 0 || abs(rc_power) == 0) {
        turn_num = 0.152;
    }
    if (lb_power > 0 && rc_power < 0) {
        turn_num = 0.154;
    }
    if (lb_power < 0 && rc_power > 0) {
        turn_num = 0.154;
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
            if (changing_power <= 15) changing_power = 15;
            if (goal_angle <= now_right_angle) break; 
            rc_power = changing_power*rc_sign;
            ev3_motor_set_power(EV3_PORT_C, rc_power);
        }
        if (rc_power == 0) {
            if (changing_power < maximum && goal_angle - (points*turn_num*ROBOT1CM) > now_left_angle) changing_power = changing_power + 0.005;
            if (goal_angle - ((points + 5)*turn_num*ROBOT1CM) <= now_left_angle) changing_power = changing_power - 0.14;
            if (changing_power <= 15) changing_power = 15;
            if (goal_angle <= now_left_angle) break; 
            lb_power = -changing_power*lb_sign;
            ev3_motor_set_power(EV3_PORT_B, lb_power);
        }
        if (lb_power != 0 && rc_power != 0){
            if (changing_power < maximum && goal_angle - (points*turn_num*ROBOT1CM) > average) changing_power = changing_power + 0.004;
            if (goal_angle - (points*turn_num*ROBOT1CM) <= now_right_angle) changing_power = changing_power - 0.03;
            if (changing_power <= 20) changing_power = 20;
            if (changing_power >= maximum) changing_power = maximum;
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








原本
    stopping();
    arm_A(OPEN);
    tslp_tsk(200*MSEC);

    
    


    ev3_motor_reset_counts(EV3_PORT_D);




    ev3_motor_reset_counts(EV3_PORT_D);
    ev3_motor_reset_counts(EV3_PORT_A);

    ev3_motor_stop(EV3_PORT_A,true);
    ev3_motor_stop(EV3_PORT_D, true);

    straight_on(-30);
    tslp_tsk(500*MSEC);
    ev3_motor_stop(EV3_PORT_A,true);
    ev3_motor_stop(EV3_PORT_D, true);

    straight(16, 30);
    turn(90, 30, -30);
    straight_on(20);
    while (true){
        color_2 = ev3_color_sensor_get_color(EV3_PORT_2);
        if(color_2 == COLOR_RED) break;
    }
    straight(23, 30);
    rg_check(EV3_PORT_1, 0);
    straight(9.8, 30);
    rg_check(EV3_PORT_1, 1);

    straight(9.8, 30);
    rg_check(EV3_PORT_1, 2);

    straight(9.8, 30);
    rg_check(EV3_PORT_1, 3);

    straight(9.8, 30);
    rg_check(EV3_PORT_1, 4);

    straight(9.8, 30);
    rg_check(EV3_PORT_1, 5);
    straight(20, -30);

    turn(180, 0, 30);

    straight(5, -30);
    turn(90, 30, -30);
    linetrace_rgb_pd_SP(BOTH, COLOR_BLACK, 20, true);
    turn(180, 30, -30);
    linetrace_cm_rgb_pd_SP(17, 20, true);

    arm_D(UP);
    
    if(mg_color[5] == COLOR_RED) {
        turn(90, -30, 30);
        straight(10, 20);
        arm_A(SET);
        arm_D(DOWN);
        arm_A(CLOSE);
        tslp_tsk(500*MSEC);
        arm_D(UP);
        straight(10, -20);
        turn(90, 30, -30);
    }

    linetrace_cm_rgb_pd_SP(9.8, 20, true);

    if(mg_color[4] == COLOR_RED) {
        turn(90, -30, 30);

        straight(10, 20);
        arm_A(SET);
        arm_D(DOWN);
        arm_A(CLOSE);
        tslp_tsk(500*MSEC);
        arm_D(UP);
        straight(10, -20);
        turn(90, 30, -30);
    }

    linetrace_cm_rgb_pd_SP(9.8, 20, true);

    if(mg_color[3] == COLOR_RED) {
        turn(90, -30, 30);
        straight(10, 20);
        arm_A(SET);
        arm_D(DOWN);
        arm_A(CLOSE);
        tslp_tsk(500*MSEC);
        arm_D(UP);
        straight(10, -20);
        turn(90, 30, -30);
    }

    linetrace_cm_rgb_pd_SP(9.8, 20, true);

    if(mg_color[2] == COLOR_RED) {
        turn(90, -30, 30);
        straight(10, 20);
        arm_A(SET);
        arm_D(DOWN);
        arm_A(CLOSE);
        tslp_tsk(500*MSEC);
        arm_D(UP);
        straight(10, -20);
        turn(90, 30, -30);
    }

    linetrace_cm_rgb_pd_SP(9.8, 20, true);

    if(mg_color[1] == COLOR_RED) {
        turn(90, -30, 30);
        straight(10, 20);
        arm_A(SET);
        arm_D(DOWN);
        arm_A(CLOSE);
        tslp_tsk(500*MSEC);
        arm_D(UP);
        straight(10, -20);
        turn(90, 30, -30);
    }

    linetrace_cm_rgb_pd_SP(9.8, 20, true);

    if(mg_color[0] == COLOR_RED) {
        turn(90, -30, 30);
        straight(10, 20);
        arm_A(SET);
        arm_D(DOWN);
        arm_A(CLOSE);
        tslp_tsk(500*MSEC);
        arm_D(UP);
        straight(10, -20);
        turn(90, 30, -30);
    }

    turn(180,30, -30);

    linetrace_rgb_pd_SP(BOTH, COLOR_BLACK, 20, true);
    straight(15, -30);
    turn(90, -30, 30);
    straight(20, 30);
    arm_D(DOWN);
    arm_A(OPEN);




