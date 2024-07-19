SP2
全3パターンのspゾーンへの移動



スタート1から中心のオブジェクトを掴むまで

   straight_on(20);
   while (true){
       color_3 = ev3_color_sensor_get_color(EV3_PORT_3);
       if(color_3 == COLOR_RED) break;
   }
   ev3_motor_stop(EV3_PORT_B, true);
   ev3_motor_stop(EV3_PORT_C, true);
   straight(3.5, 20);
   turn(90, 30, -30);
   straight(10, 30);
   linetrace_cm_rgb_pd_SP(10, 20, false);
   linetrace_cm_rgb_pd_SP(30, 40, false);
   linetrace_rgb_pd_SP(BOTH, COLOR_BLACK, 20, true);
   straight(3, 20);
   turn(90, -30, 30);
   arm_A(SET);
   arm_D(DOWN);
   straight(14, 30);
   arm_A(CLOSE);



スタート2から中心のオブジェクトをつかむまで
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





スタート1から隅っこのオブジェクトをつかむまで
    straight_on(-30);
    tslp_tsk(400*MSEC);
    straight(15, 30);
    turn(90, -30, 30);
    straight_on(20);
    while (true){
        color_3 = ev3_color_sensor_get_color(EV3_PORT_3);
        if(color_3 == COLOR_RED) break;
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
    arm_A(SET);
    arm_D(DOWN);
    straight(44, 40);
    arm_A(CLOSE);