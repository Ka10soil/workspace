if(start==1){
        straight(30, -30);
        turn(90, 30, -30);
        straight_on(-30);
        tslp_tsk(800*MSEC);
        straight(13, 30);
        turn(90, 30, -30);
        straight_on(30);
        arm_D(ALLUP);
        while (true){
            color_3 = ev3_color_sensor_get_color(EV3_PORT_3);
            if(color_3 == COLOR_WHITE) break;
        }
        arm_A(OPEN);
        while (true){
            color_3 = ev3_color_sensor_get_color(EV3_PORT_3);
            if(color_3 == COLOR_RED) break;
        }
        ev3_motor_stop(EV3_PORT_B, true);
        ev3_motor_stop(EV3_PORT_C, true);
        straight(6.5, -30);
    }


    else{
        straight(30, -30);
        turn(90, 30, -30);
        straight_on(-30);
        tslp_tsk(800*MSEC);
        straight(20, 30);
        turn(90, 30, -30);
        straight(20, 30);
        linetrace_cm_pd_SP(60, 30, false);
        linetrace_color_pd_SP(BOTH, COLOR_BLACK, 30, true);
        turn(90, 30, -30);
        straight_on(-30);
        tslp_tsk(2500*MSEC);
        straight(6, 30);
    }
