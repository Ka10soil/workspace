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
#include "stdlib.h"
#include <stdio.h>
#include "math.h"

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

/**
 * Define the connection ports of the sensors and motors.
 * By default, this application uses the following ports:
 * Touch sensor: Port 2
 * Color sensor: Port 3
 * Left motor:   Port B
 * Right motor:  Port C
 */
static const sensor_port_t  PortSensorColor1 = EV3_PORT_1;
static const sensor_port_t  PortSensorColor2 = EV3_PORT_2; //左
static const sensor_port_t  PortSensorColor3 = EV3_PORT_3; //右
static const sensor_port_t  PortSensorColor4 = EV3_PORT_4; //超音波はここ

static const motor_port_t   PortMotorArmUp   = EV3_PORT_A; //オブジェをとる
static const motor_port_t   PortMotorLeft    = EV3_PORT_B; //lb
static const motor_port_t   PortMotorRight   = EV3_PORT_C; //rc
static const motor_port_t   PortMotorArmDown = EV3_PORT_D; //オブジェを落とす


void stopping();
void arm_A(armmode_new_t mode);

armmode_t now_armmode = SET;

int now_arm_angle;
int now_arm_angle_A;

static void button_clicked_handler(intptr_t button) {
    switch(button) {
    case BACK_BUTTON:
#if !defined(BUILD_MODULE)
        syslog(LOG_NOTICE, "Back button clicked.");
#endif
        break;
    }
}
    
void stopping(){
    while(ev3_button_is_pressed(ENTER_BUTTON) == false) {}    
    tslp_tsk(2000*MSEC);
}

void arm_A(armmode_new_t mode){
    now_arm_angle_A = ev3_motor_get_counts(EV3_PORT_A);
    switch (mode) {
        case OPEN:
            ev3_motor_set_power(EV3_PORT_A, 30);
            break;
        case CLOSE:
            ev3_motor_set_power(EV3_PORT_A, -30);
            break;
        default:
            break;
    }
    now_arm_angle_A = ev3_motor_get_counts(EV3_PORT_A);
}

void arm_D(armmode_new_t mode) {
    now_arm_angle = ev3_motor_get_counts(EV3_PORT_D);
    switch (mode) {
        case DOWN:
            ev3_motor_set_power(EV3_PORT_D, 40);
            break;
        case UP:
            ev3_motor_set_power(EV3_PORT_D, -40);
            break;
        default:
            break;
    }
    now_arm_angle = ev3_motor_get_counts(EV3_PORT_D);
}

void main_task(intptr_t unused) {



    /* Register button handlers */
    ev3_button_set_on_clicked(BACK_BUTTON, &button_clicked_handler, BACK_BUTTON);

    /* Configure motors */
    ev3_motor_config(PortMotorLeft, MEDIUM_MOTOR);
    ev3_motor_config(PortMotorRight, MEDIUM_MOTOR);
    ev3_motor_config(PortMotorArmDown, MEDIUM_MOTOR);
    ev3_motor_config(PortMotorArmUp, MEDIUM_MOTOR);

    /* Configure sensors */
    ev3_sensor_config(PortSensorColor1, COLOR_SENSOR);
    ev3_sensor_config(PortSensorColor2, COLOR_SENSOR);
    ev3_sensor_config(PortSensorColor3, COLOR_SENSOR);
    ev3_sensor_config(PortSensorColor4, COLOR_SENSOR);


    

    /* ここからコーディング */
    arm_A(OPEN);
    arm_D(UP);
    tslp_tsk(800*MSEC);
    arm_D(DOWN);
    tslp_tsk(800*MSEC);
    arm_A(CLOSE);
    tslp_tsk(800*MSEC);
}