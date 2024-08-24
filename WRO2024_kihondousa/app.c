
        straight_on(20);
        while (true){
            color_3 = ev3_color_sensor_get_color(EV3_PORT_3);
            if(color_3 == COLOR_WHITE) break;
        }
        ev3_motor_stop(EV3_PORT_B, true);
        ev3_motor_stop(EV3_PORT_C, true);


        straight_on(20);
        while (true){
            now_reflect_3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
            if(now_reflect_3 <= 10) break;
        }
        ev3_motor_stop(EV3_PORT_B, true);
        ev3_motor_stop(EV3_PORT_C, true);


        

        straight_on(-20);
        tslp_tsk(300*MSEC);
        ev3_motor_stop(EV3_PORT_B, true);
        ev3_motor_stop(EV3_PORT_C, true);