SP1
中心寄りのなんか置かれそうだなあエリアに白いホームオブジェクトを配置。
反対のスタートエリアに運べ。



startは2で白いオブジェクトを掴むプログラム
    arm_A(SET);
    straight_on(20);
    while (true){
        color_3 = ev3_color_sensor_get_color(EV3_PORT_3);
        if(color_3 == COLOR_RED) break;
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
    straight(4.5, 30);//
    turn(90, 30, -30);
    speed_arm(EV3_PORT_D, DOWN, 20);
    straight_on(-30);
    tslp_tsk(400*MSEC);
    arm_D(DOWN);
    straight(35, 40);//
    arm_A(CLOSE);
    tslp_tsk(200*MSEC);//armはCLOSE DOWN



area_2または、area_1から反対側のスタートエリアの赤ラインまで爆速で行くプログラム
    straight_on(50);
    while (true){
        color_3 = ev3_color_sensor_get_color(EV3_PORT_3);
        if(color_3 == COLOR_WHITE) break;
    }
    straight_on(20);
    while (true){
        color_3 = ev3_color_sensor_get_color(EV3_PORT_3);
        if(color_3 == COLOR_BLACK) break;
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
    straight(8, 30);
    turn(90, 30, -30);
    linetrace_cm_rgb_pd_SP(15, 20, false);
    linetrace_rgb_pd_SP(BOTH, COLOR_RED, 20, true);







    straight(10,-30);
    turn(90, 30, -30);
    arm_D(UP);



    straight(45, 50);
    turn(90, -30, 30);





    straight(30, 60);
    
    turn(180, 30, -30);
    straight(6, -30);
    arm_D(DOWN);
    arm_A(SET);
    straight(12, -30);
    turn(90, -30, 30);
    //
    straight_on(-30);
    tslp_tsk(800*MSEC);