rgのカラー読み
void rg_check(port_t port){
    rgb_raw_t rgb_val;//カラーセンサーの値を保存するために必要な変数(必須)
    int red = 0;
    int green = 0;
    int blue = 0;
    ev3_color_sensor_get_rgb_raw(port, &rgb_val);
    ev3_color_sensor_get_rgb_raw(port, &rgb_val);
    red = rgb_val.r;
    green = rgb_val.g;
    blue = rgb_val.b;
    obj_color = rgb_color(red, green, blue);
    if(red > green) bg_color = COLOR_RED;
    else bg_color = COLOR_GREEN;
    fprintf(bt, "sensor:r%d g:%d b:%d\r\njudge_color %d\r\n", red, green, blue, bg_color);
}

取って捨てる関数
void rg_get(){
    rg_check(EV3_PORT_1);
    if(bg_color == 3){
        arm_D(DOWN);
        turn(90, -30, 30);
        straight(10, 30);
        arm_A(OPEN);
        arm_D(ONE);
        arm_A(CLOSE);
        arm_D(UP);
        straight(10, -30);
        turn(91, 30, -30);
    }
}



スタートから6個のオブジェクトを取りに行く
    straight_on(-50);
    tslp_tsk(300*MSEC);
    straight_on(-20);
    tslp_tsk(200*MSEC);
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
    straight(2.5, 20);
    tslp_tsk(200*MSEC);
    turn(91, 30, -30);
    straight_on(20);
    while(true){
        color_3 = ev3_color_sensor_get_color(EV3_PORT_3);
        if(color_3 == COLOR_WHITE) break;
    }
    while(true){
        color_3 = ev3_color_sensor_get_color(EV3_PORT_3);
        if(color_3 == COLOR_RED) break;
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);




    arm_D(UP);
   
    straight(8.5, 30);
    arm_D(DOWN);
    arm_A(CLOSE);
    arm_D(UP);




    straight(9.8, 30);
    arm_A(OPEN);
   




    arm_D(DOWN);
    arm_A(CLOSE);
    arm_D(UP);


    straight(9.8, 30);
    arm_A(OPEN);




    arm_D(DOWN);
    arm_A(CLOSE);
    arm_D(UP);


    straight(9.8, 30);
    arm_A(OPEN);




    arm_D(DOWN);
    arm_A(CLOSE);
    arm_D(UP);


    straight(9.8, 30);
    arm_A(OPEN);
   
    arm_D(DOWN);
    arm_A(CLOSE);
    arm_D(UP);




    straight(9.8, 30);
    arm_A(OPEN);


    arm_D(DOWN);
    arm_A(CLOSE);
    arm_D(UP);


