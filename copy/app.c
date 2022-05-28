/**
 * This sample program shows a PID controller for line following.
 *
 * Robot construction: Educator Vehicle
 *
 * References:
 * http://robotsquare.com/wp-content/uploads/2013/10/45544_educator.pdf
 * http://thetechnicgear.com/2014/03/howto-create-line-following-robot-using-mindstorms/
 */

#include "ev3api.h"
#include "app.h"


#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

#define MSEC (1000)
#define ROBOT1CM (18.2)
#define TURN (0.1705)

/**
 * Define the connection ports of the sensors and motors.
 * By default, this application uses the following ports:
 * Touch sensor: Port 2
 * Color sensor: Port 3
 * Left motor:   Port B
 * Right motor:  Port C
 */
static const sensor_port_t  PortSensorColor2 = EV3_PORT_2;
static const sensor_port_t  PortSensorColor3 = EV3_PORT_3;
static const motor_port_t   PortMotorLeft   = EV3_PORT_B;
static const motor_port_t   PortMotorRight  = EV3_PORT_C;

static void button_clicked_handler(intptr_t button) {
    switch(button) {
    case BACK_BUTTON:
#if !defined(BUILD_MODULE)
        syslog(LOG_NOTICE, "Back button clicked.");
#endif
        break;
    }
}

int kakudo_C; 
int power = 50;
int steer;

/*ライントレース用の変数の定義*/
int reflect, reflect2, reflect3;
float p, i, d, d2;

int start;
colorid_t color;

/*タイムアウトタスクの変数の定義*/
int TIME;
SYSTIM NOWTIME;
SYSTIM STARTTIME;

typedef enum sensorType {
    RIGHT,
    LEFT,
    BOTH
} sensortype_t ;

void steering(float length, int power, int steering){
    int true_steering = 0;
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    	
    if(steering > 0) {
        (void)ev3_motor_rotate(EV3_PORT_B, length*ROBOT1CM, -power, false);
        (void)ev3_motor_rotate(EV3_PORT_C, length*ROBOT1CM, power-(power*steering/50), false);
    }
    else {
        (void)ev3_motor_rotate(EV3_PORT_B, length*ROBOT1CM, -(power+(power*steering/50)), false);
        (void)ev3_motor_rotate(EV3_PORT_C, length*ROBOT1CM, power, false);
    }
    while(true_steering < 1){
        if (ev3_motor_get_counts(EV3_PORT_B) > length*ROBOT1CM) {
            (void)ev3_motor_stop(EV3_PORT_B, true);
            (void)ev3_motor_stop(EV3_PORT_C, true);
            true_steering = true_steering + 1;
        }
        else if (ev3_motor_get_counts(EV3_PORT_C) > length*ROBOT1CM) {
            (void)ev3_motor_stop(EV3_PORT_B, true);
            (void)ev3_motor_stop(EV3_PORT_C, true);
            true_steering = true_steering + 1;
        }
    }
}

void tank_turn(float angle, int power_L, int power_R){
    (void)ev3_motor_rotate(EV3_PORT_B, angle*TURN*ROBOT1CM, (int16_t)-power_L, false);
    (void)ev3_motor_rotate(EV3_PORT_C, angle*TURN*ROBOT1CM, (int16_t)power_R, true);
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, false);
}

void tank_turn_color(int power_L, int power_R){

    colorid_t color;
    ev3_motor_set_power(EV3_PORT_C, power_R);
    ev3_motor_set_power(EV3_PORT_B, -power_L);
    while(color != COLOR_WHITE) {
        if(power_L > 0) {
            color = ev3_color_sensor_get_color(EV3_PORT_2);
        } 
        else {
            color = ev3_color_sensor_get_color(EV3_PORT_3);
        }
    }
    while(color != COLOR_BLACK) {
        if(power_L > 0) {
            color = ev3_color_sensor_get_color(EV3_PORT_2);
        } 
        else {
            color = ev3_color_sensor_get_color(EV3_PORT_3);
        }
    }
    (void)ev3_motor_stop(EV3_PORT_B, true);
    (void)ev3_motor_stop(EV3_PORT_C, true);    
}

void steering_color(colorid_t color_stop, int power, int steering){
    colorid_t color;
    if(steering > 0) {
        ev3_motor_set_power(EV3_PORT_B, -power);
        ev3_motor_set_power(EV3_PORT_C, power-(power*steering/50));
    }
    else {
        ev3_motor_set_power(EV3_PORT_B, -(power+(power*steering/50)));
        ev3_motor_set_power(EV3_PORT_C, power);
    }
    while (color_stop != color) {
        color = ev3_color_sensor_get_color(EV3_PORT_3);
    }
    (void)ev3_motor_stop(EV3_PORT_B, true);
    (void)ev3_motor_stop(EV3_PORT_C, true);    
}

void steering_time(colorid_t time_stop_4d, int power, int steering){
    if(steering > 0) {
        ev3_motor_set_power(EV3_PORT_B, -power);
        ev3_motor_set_power(EV3_PORT_C, power-(power*steering/50));
    }
    else {
        ev3_motor_set_power(EV3_PORT_B, -power);
        ev3_motor_set_power(EV3_PORT_C, power-(power*steering/50));
    }
    tslp_tsk(time_stop_4d * MSEC);
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
}

void linetrace_color(sensortype_t type, colorid_t color_stop, int power){
    colorid_t color2 = COLOR_NONE;
    colorid_t color3 = COLOR_NONE;
    while (true) {
        color2 = ev3_color_sensor_get_color(EV3_PORT_2);
        color3 = ev3_color_sensor_get_color(EV3_PORT_3);
        reflect2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
        reflect3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
        reflect = reflect2 - reflect3;
        p = reflect;
        i = (reflect + i);
        d = (reflect - d2); 
        d2 = reflect;
        steer =  p * P_GEIN; 
        if(steer > 0) {
            ev3_motor_set_power(EV3_PORT_B, -power);
            ev3_motor_set_power(EV3_PORT_C, power-(power*steer/50));
        }
        else {
            ev3_motor_set_power(EV3_PORT_B, -power);
            ev3_motor_set_power(EV3_PORT_C, power-(power*steer/50));
        }
        if(color2 == color_stop && color3 == color_stop && type == BOTH) break;
        if(color2 == color_stop && type == RIGHT) break;
        if(color3 == color_stop && type == LEFT) break;
    }  
    (void)ev3_motor_stop(EV3_PORT_B, true);
    (void)ev3_motor_stop(EV3_PORT_C, true);
    
}

void linetrace_length(float length, int power){
    ev3_motor_reset_counts(EV3_PORT_C);
    while (length * ROBOT1CM > kakudo_C) {
        kakudo_C = ev3_motor_get_counts(EV3_PORT_C);
        reflect2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
        reflect3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
        reflect = reflect2 - reflect3;
        p = reflect;
        i = (reflect + i);
        d = (reflect - d2); 
        d2 = reflect;
        steer =  p * P_GEIN; 
        if(steer > 0) {
            ev3_motor_set_power(EV3_PORT_B, -power);
            ev3_motor_set_power(EV3_PORT_C, power-(power*steer/50));
        }
        else {
            ev3_motor_set_power(EV3_PORT_B, -power);
            ev3_motor_set_power(EV3_PORT_C, power-(power*steer/50));
        }

    }
    (void)ev3_motor_stop(EV3_PORT_B, true);
    (void)ev3_motor_stop(EV3_PORT_C, true);
}

void sensor_check(motor_port_t port) {
    switch (port) {
    case EV3_PORT_1:
        /* code */
        break;
    case EV3_PORT_2:
        /* code */
        break;
    case EV3_PORT_3:
        /* code */
        break;    
    case EV3_PORT_4:
        /* code */
        break;
    default:
        break;
    }
}

void timeout_task(intptr_t unused) {
    SYSTIM nowtime, time;   
    char str[64];
    get_tim(&nowtime);
    time = nowtime - STARTTIME;
    sprintf(str, "TIME:%ld", time);
    ev3_lcd_draw_string(str, 1, 1);
}

void main_task(intptr_t unused) {

    
    /* Register button handlers */
    ev3_button_set_on_clicked(BACK_BUTTON, &button_clicked_handler, BACK_BUTTON);

    /* Configure motors */
    ev3_motor_config(PortMotorLeft, LARGE_MOTOR);
    ev3_motor_config(PortMotorRight, LARGE_MOTOR);

    /* Configure sensors */
    ev3_sensor_config(PortSensorColor2, COLOR_SENSOR);
    ev3_sensor_config(PortSensorColor3, COLOR_SENSOR);

    get_tim(&STARTTIME);
    ev3_lcd_set_font(EV3_FONT_SMALL);

    ev3_color_sensor_get_color(EV3_PORT_3);
    ev3_color_sensor_get_color(EV3_PORT_2);



    (void)sta_cyc(TIMEOUT_CYC);

    /*ここからコーディング */

    ev3_motor_set_power(EV3_PORT_C, -25);
    ev3_motor_set_power(EV3_PORT_B, -25);
    tank_turn_color(25, -25);

}   
