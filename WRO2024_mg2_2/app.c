mg2_2　黄色いハウスエリアに２個積んであるハウスエレメンツを初期状態の場所（６個並んでるうちの２つ）に配置
    straight_on(20);
    while (true){
        color_3 = ev3_color_sensor_get_color(EV3_PORT_3);
        if(color_3 == COLOR_WHITE) break;
    }
    while (true){
        color_3 = ev3_color_sensor_get_color(EV3_PORT_3);
        if(color_3 == COLOR_RED) break;
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
    turn(90, 30, -30);
    straight(10, 30);
    linetrace_cm_rgb_pd_SP(10, 20, false);
    linetrace_rgb_pd_SP(LEFT, COLOR_BLACK, 20, true);
    straight(4, 20);
    turn(90, -30, 30);
    arm_A(GET_OBJ_2);
    arm_D(THREE);
    linetrace_cm_rgb_pd_SP(18, 20, true);




    arm_D(DOWN);
    arm_A(CLOSE);
    tslp_tsk(400*MSEC);
    turn(180, 30, -30);
    linetrace_cm_rgb_pd_SP(10, 20, false);
    linetrace_rgb_pd_SP(BOTH, COLOR_BLACK, 20, true);
    straight(12, 30);


    arm_A(GET_OBJ_2);
    arm_D(ONE);
    arm_A(CLOSE);
    tslp_tsk(400*MSEC);
    arm_D(UP);
    straight(12, -30);
    turn(20, 30, -30);
    straight(10, 30);
    arm_D(DOWN);
    arm_A(SET);