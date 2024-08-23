mg6_1　土台の上のオブジェをゲットしてゴール
　　straight(30, 40);
    turn(90, 30, -30);
    straight_on(-40);
    tslp_tsk(500*MSEC);
    straight_on(-20);
    tslp_tsk(300*MSEC);
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
    straight(30, 30);
    turn(90, -30, 30);




    straight(15, 30);
    arm_D(THREE);
    arm_A(OPEN);
    straight_on(20);
    while (true){
        color_3 = ev3_color_sensor_get_color(EV3_PORT_3);
        if(color_3 == COLOR_WHITE) break;
    }
    while (true){
        color_3 = ev3_color_sensor_get_color(EV3_PORT_3);
        if(color_3 == COLOR_BLACK) break;
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
    arm_D(ONE);
    arm_A(CLOSE);
    arm_D(TWO);
    straight(60, -40);
    turn(90, 30, -30);
    straight_on(-30);
    tslp_tsk(1000*MSEC);
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true)