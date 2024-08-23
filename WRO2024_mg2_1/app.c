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

static FILE *bt = NULL;

FILE *file;
const char* logfilename = "/wro_result.txt";
/**
 * Define the connection ports of the sensors and motors.
 * By default, this application uses the following ports:
 * Touch sensor: Port 2
 * Color sensor: Port 3
 * Left motor:   Port B
 * Right motor:  Port C
 */
int battery;

int start = 2;
int yellow = 4;
colorid_t bg_color = 2;


colorid_t color_3 = 0;
colorid_t color_2 = 0;
rgb_raw_t rgb_val;
int red;
int green;
int blue;

armmode_t now_armmode = SET;
armmode_new_t now_mode = SET;

int now_arm_angle;
int now_arm_angle_A;

int now_reflect_2; 
int now_reflect_3; 
colorid_t obj_color;

static const sensor_port_t  PortSensorColor1 = EV3_PORT_1;
static const sensor_port_t  PortSensorColor2 = EV3_PORT_2; //左
static const sensor_port_t  PortSensorColor3 = EV3_PORT_3; //右
static const sensor_port_t  PortSensorGyro = EV3_PORT_4; //ジャイロセンサー

static const motor_port_t   PortMotorArmUp   = EV3_PORT_A; //オブジェをとる
static const motor_port_t   PortMotorLeft    = EV3_PORT_B; //lb
static const motor_port_t   PortMotorRight   = EV3_PORT_C; //rc
static const motor_port_t   PortMotorArmDown = EV3_PORT_D; //オブジェを落とす


void stopping();
void straight(float cm, int power);
void turn(int angle, int lb_power, int rc_power);
void gain_set(int power, float *p_gain, float *d_gain);
void gain_set_rgb(int power, float *p_gain, float *d_gain);
void gain_set_pid(int power, float *p_gain, float *d_gain, float *i_gain);
void linetrace_cm_pd_SP(float cm, int power, bool_t stop);
void arm_A(armmode_new_t mode);
void arm_D(armmode_new_t mode);
void color_check(port_t port);
void start_2();
void start_1();
void area_1();
void area_2();
void water();
void take_house();
void build_house();
colorid_t rgb_color(float r, float g, float b);
void color_check (port_t port);

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
    tslp_tsk(1000*MSEC);
}

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
    float points = 0.6;
    float turn_num = 0.1525;
    if (abs(lb_power) == 0) {
        turn_num = 0.157;
        if (rc_power < 0) {
            turn_num = 0.1595;
        }
    }
    if (abs(rc_power) == 0) {
        turn_num = 0.156;
    }
    if (lb_power > 0 && rc_power < 0) {
        turn_num = 0.154;
        if (angle >= 180) {
            turn_num = 0.159;
        }
        //turn_num = 0.159;
    }
    if (lb_power < 0 && rc_power > 0) {
        turn_num = 0.1563;
        if (angle >= 180) {
            turn_num = 0.1553;
        }
        //turn_num = 0.1557;
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
            if (changing_power < maximum && goal_angle - (angle*points*turn_num*ROBOT1CM) > now_right_angle) changing_power = changing_power + 0.003;
            if (goal_angle - ((angle*points + 5)*turn_num*ROBOT1CM) <= now_right_angle) changing_power = changing_power - 0.003;
            if (changing_power <= 20) changing_power = 20;
            if (goal_angle <= now_right_angle) break; 
            rc_power = changing_power*rc_sign;
            ev3_motor_set_power(EV3_PORT_C, rc_power);
        }
        if (rc_power == 0) {
            if (changing_power < maximum && goal_angle - (angle*points*turn_num*ROBOT1CM) > now_left_angle) changing_power = changing_power + 0.005;
            if (goal_angle - ((angle*points + 5)*turn_num*ROBOT1CM) <= now_left_angle) changing_power = changing_power - 0.003;
            if (changing_power <= 20) changing_power = 20;
            if (goal_angle <= now_left_angle) break; 
            lb_power = -changing_power*lb_sign;
            ev3_motor_set_power(EV3_PORT_B, lb_power);
        }
        if (lb_power != 0 && rc_power != 0 && (angle <= 85 || angle >= 95)){
            if (changing_power < maximum && goal_angle - (angle*points*turn_num*ROBOT1CM) > average) changing_power = changing_power + 0.003;
            if (goal_angle - (angle*points*turn_num*ROBOT1CM) <= now_right_angle && angle <= 90) changing_power = changing_power - 0.021;
            if (goal_angle - (angle*points*turn_num*ROBOT1CM) <= now_right_angle && angle > 90) changing_power = changing_power - 0.004;
            
            
            if (changing_power <= 20) changing_power = 20;

            if (changing_power >= maximum) changing_power = maximum;

            if (goal_angle <= now_left_angle && goal_angle <= now_right_angle) break; 
            rc_power = changing_power*rc_sign;
            lb_power = -changing_power*lb_sign;
            if(now_right_angle <= goal_angle)ev3_motor_set_power(EV3_PORT_C, rc_power);
            else ev3_motor_stop(EV3_PORT_C, true);
            if(now_left_angle <= goal_angle)ev3_motor_set_power(EV3_PORT_B, rc_power);
            else ev3_motor_stop(EV3_PORT_B, true);
        }  
        if (lb_power != 0 && rc_power != 0 && (angle > 85 && angle < 95)){
            if (changing_power < maximum && goal_angle - (angle*points*turn_num*ROBOT1CM) > average) changing_power = changing_power + 0.008;
            if (goal_angle - (angle*points*turn_num*ROBOT1CM) <= now_right_angle) changing_power = changing_power - 0.014;
            
            
            if (changing_power <= 20) changing_power = 20;


            if (changing_power >= maximum) changing_power = maximum;

            if (goal_angle <= now_left_angle && goal_angle <= now_right_angle) break; 
            rc_power = changing_power*rc_sign;
            lb_power = -changing_power*lb_sign;
            if(now_right_angle <= goal_angle)ev3_motor_set_power(EV3_PORT_C, rc_power);
            else ev3_motor_stop(EV3_PORT_C, true);
            if(now_left_angle <= goal_angle)ev3_motor_set_power(EV3_PORT_B, rc_power);
            else ev3_motor_stop(EV3_PORT_B, true);
        }  
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
    fprintf(bt, "now_LR:%d :%d\r\nchange%f\r\n", now_left_angle, now_right_angle, changing_power);
}



/*void turn(float target_angle, int lb_power, int rc_power){
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
    tslp_tsk(100*MSEC);
    int lb_sign = lb_power / abs(lb_power);
    int rc_sign = rc_power / abs(rc_power);
    

    if(lb_power == 0 || rc_power == 0) target_angle = target_angle / 2;
    target_angle += 0.2;

    float changing_power;
    float p_gain = 1.2;
    float d_gain = -0.14;
    float p = 1;
    float d;
    int max_power = 70;
    int min_power = 9;
    SYSTIM timer = 0;
    ulong_t now_timer = 0;
    SYSTIM last_tim;
    SYSTIM now_tim;
    SYSTIM first_tim;
    float deltat;
    float deltats;
    ev3_gyro_sensor_reset(EV3_PORT_4);
    float now_rate = ev3_gyro_sensor_get_rate(EV3_PORT_4);
    float last_rate = 0;
    float now_angle = 0;
    get_tim(&now_tim);
    get_tim(&first_tim);
    int counts;
    while (true)
    {
        last_rate = now_rate;
        last_tim = now_tim;
        if(lb_sign > 0 || rc_sign < 0) now_rate = ev3_gyro_sensor_get_rate(EV3_PORT_4);
        else now_rate = -ev3_gyro_sensor_get_rate(EV3_PORT_4);
        get_tim(&now_tim);
        deltat  = now_tim - last_tim;
        deltats  = deltat / 1000000;
        now_angle += (now_rate + last_rate) * deltats / 2.0;
        //fprintf(bt, "angle%f\r\n", now_angle);
        //fprintf(bt, "rate%f\r\n", now_angle);
        //fprintf(bt, "change%lu angle%f\r\n", now_timer, now_angle);
        d = now_rate;
        if(now_angle < target_angle / 3) p = (now_angle / 3) + 30;
        if(now_angle >= target_angle / 3) p = target_angle  - now_angle;
        changing_power = p * p_gain + d * d_gain + min_power * sin(atan(3*p));
        if(changing_power > max_power) changing_power = max_power;
        if(lb_power == 0){
            ev3_motor_set_power(EV3_PORT_C, rc_sign*changing_power);
        }
        if(rc_power == 0){
            ev3_motor_set_power(EV3_PORT_B, -lb_sign*changing_power);
        }
        if(lb_power != 0 && rc_power != 0){
            ev3_motor_set_power(EV3_PORT_B, -lb_sign*changing_power);
            ev3_motor_set_power(EV3_PORT_C, rc_sign*changing_power);
        }
        
        if (now_angle > target_angle - 0.01){
            break;
            if(timer == 0) get_tim(&timer);
            else now_timer = now_tim - timer;
            if(now_timer > 100000) break;
        
        }
        if(now_tim - first_tim > 3000000) {
            fprintf(bt, "angle%f\r\n", now_rate);
            ev3_speaker_play_tone(540, 100);
            break;
         
        }


        tslp_tsk(10000);
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
    fprintf(bt, "angle%f\r\n", now_angle);

}*/

void syuuuki(){


}

void turn_gyro(int angle, int lb_power, int rc_power){
    tslp_tsk(200*MSEC);
    if(angle > 100) tslp_tsk(200*MSEC);
    if (lb_power == 30) lb_power = lb_power + 35;
    if (lb_power == -30) lb_power = lb_power - 35;
    if (rc_power == 30) rc_power = rc_power + 35;
    if (rc_power == -30) rc_power = rc_power - 35;

    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    ev3_gyro_sensor_reset(EV3_PORT_4);
    int lb_sign = lb_power / abs(lb_power);
    int rc_sign = rc_power / abs(rc_power);
    int now_right_angle = 0;
    int now_left_angle = 0;
    int average = 0;
    int maximum = 80;
    int points = 55;
    int gyro_power = 10;
    float turn_num = 0.1525;
    float gyro_angle = 0;
    if (abs(lb_power) == 0 || abs(rc_power) == 0) {
        turn_num = 0.152;
    }
    if (lb_power > 0 && rc_power < 0) {
        //turn_num = 0.1533;
        turn_num = 0.159;
    }
    if (lb_power < 0 && rc_power > 0) {
        //turn_num = 0.153;
        turn_num = 0.1557;
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
            if (changing_power < maximum && goal_angle - (points*turn_num*ROBOT1CM) > average) changing_power = changing_power + 0.003;
            if (goal_angle - (points*turn_num*ROBOT1CM) <= now_right_angle) changing_power = changing_power - 0.014;
            if (changing_power <= 15) changing_power = 15;
            if (changing_power >= maximum) changing_power = maximum;
            if (goal_angle <= average) break; 
            rc_power = changing_power*rc_sign;
            lb_power = -changing_power*lb_sign;
            ev3_motor_set_power(EV3_PORT_C, rc_power);
            ev3_motor_set_power(EV3_PORT_B, lb_power);
        }  
    }
    //gyro_turn
    gyro_angle = abs(ev3_gyro_sensor_get_angle(EV3_PORT_4));
    if (angle - 5 <= gyro_angle && gyro_angle <= angle + 5){
        while (true) {
            gyro_angle = ev3_gyro_sensor_get_angle(EV3_PORT_4);
            if (gyro_angle > angle) gyro_power = gyro_power * -1; 
            rc_power = 10*rc_sign;
            lb_power = -10*lb_sign;
            ev3_motor_set_power(EV3_PORT_C, rc_power);
            ev3_motor_set_power(EV3_PORT_B, lb_power);
            if(gyro_angle == angle) break;
        }
    }
    
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
}

void arm_A(armmode_new_t mode){
    now_arm_angle_A = ev3_motor_get_counts(EV3_PORT_A);
    switch (mode) {
        case OPEN:
            if(now_arm_angle_A <= 90)ev3_motor_set_power(EV3_PORT_A, 80);
            else ev3_motor_set_power(EV3_PORT_A, -80);
            break;
        case CLOSE:
            if(now_arm_angle_A <= -110)ev3_motor_set_power(EV3_PORT_A, 80);
            else ev3_motor_set_power(EV3_PORT_A, -80);

            break;
        default:
            break;
    }
    while (true) {
        now_arm_angle_A = ev3_motor_get_counts(EV3_PORT_A);
        if(now_arm_angle_A <= -199 && now_arm_angle_A >= -201 && mode == SET) break;
        if(now_arm_angle_A <= -219 && now_arm_angle_A >= -221 && mode == GET_OBJ_2) break;
        if(now_arm_angle_A <= -179 && now_arm_angle_A >= -181 && mode == GETDEBRIS) break;
        if(now_arm_angle_A <= 91 && now_arm_angle_A >= 89 && mode == OPEN) break;
        if(now_arm_angle_A <= -109 && now_arm_angle_A >= -111 && mode == CLOSE) break;

        
    }
    if(mode == SET)ev3_motor_stop(EV3_PORT_A, true);
    if(mode == GET_OBJ_2) ev3_motor_stop(EV3_PORT_A, true);
    if(mode == GETDEBRIS)ev3_motor_stop(EV3_PORT_A, true);
    if(mode == OPEN)ev3_motor_stop(EV3_PORT_A, true);
    if(mode == CLOSE)ev3_motor_stop(EV3_PORT_A, true);
    now_arm_angle_A = ev3_motor_get_counts(EV3_PORT_A);
}


void arm_D(armmode_new_t mode) {
    now_arm_angle = ev3_motor_get_counts(EV3_PORT_D);
    now_mode = mode;
    switch (mode) {
        case DOWN:
            if(now_arm_angle <= 5)ev3_motor_set_power(EV3_PORT_D, 30);
            else ev3_motor_set_power(EV3_PORT_D, -80);
            break;
        case UP:
            if(now_arm_angle <= 300)ev3_motor_set_power(EV3_PORT_D, 80);
            else ev3_motor_set_power(EV3_PORT_D, -80);
            break;
        case ONE:
            if(now_arm_angle <= 265)ev3_motor_set_power(EV3_PORT_D, 80);
            else ev3_motor_set_power(EV3_PORT_D, -80);
            break;
        case TWO:
            if(now_arm_angle <= 480)ev3_motor_set_power(EV3_PORT_D, 80);
            else ev3_motor_set_power(EV3_PORT_D, -80);
            break;
        case THREE:
            if(now_arm_angle <= 695)ev3_motor_set_power(EV3_PORT_D, 80);
            else ev3_motor_set_power(EV3_PORT_D, -80);
            break;
        case FOUR:
            if(now_arm_angle <= 905)ev3_motor_set_power(EV3_PORT_D, 80);
            else ev3_motor_set_power(EV3_PORT_D, -80);
            break;
        case ALLUP:
            ev3_motor_set_power(EV3_PORT_D, 80);
            break;
        default:
            break;
    }
    now_mode = mode;
    //sta_cyc(ARM_CYC);
    while (true) {
        now_arm_angle = ev3_motor_get_counts(EV3_PORT_D);
        if(now_arm_angle <= 6 && now_arm_angle >= 4 && mode == DOWN) break;
        if(now_arm_angle <= 301 && now_arm_angle >= 299 && mode == UP) break;
        if(now_arm_angle <= 266 && now_arm_angle >= 264 && mode == ONE) break;
        if(now_arm_angle <= 481 && now_arm_angle >= 479 && mode == TWO) break;
        if(now_arm_angle <= 696 && now_arm_angle >= 694 && mode == THREE) break;
        if(now_arm_angle <= 906 && now_arm_angle >= 904 && mode == FOUR) break;
        if(now_arm_angle >= 940 && mode == ALLUP) break;
    }
    /*if(mode == SETNEW) {
        ev3_motor_stop(EV3_PORT_D, true);
        tslp_tsk(200*MSEC);
        ev3_motor_set_power(EV3_PORT_D, -10);
        while (true) {
            now_arm_angle = ev3_motor_get_counts(EV3_PORT_D);
            if(now_arm_angle <= -76 && now_arm_angle >= -78 && mode == SETNEW) break;
        }
    }*/
    if(mode == ONE || mode == TWO || mode == THREE || mode == FOUR ||mode == ALLUP || mode == DOWN || mode == UP)ev3_motor_stop(EV3_PORT_D, true);
    now_arm_angle = ev3_motor_get_counts(EV3_PORT_D);
    tslp_tsk(100*MSEC);
}

/*void arm_A(armmode_new_t mode){
    now_arm_angle_A = ev3_motor_get_counts(EV3_PORT_A);
    switch (mode) {
        case OPEN:
            ev3_motor_set_power(EV3_PORT_A, 20);
            break;
        case CLOSE:
            ev3_motor_set_power(EV3_PORT_A, -60);
            break;
        case SET:
            if(now_arm_angle_A >= -220)ev3_motor_set_power(EV3_PORT_A, -20);
            else ev3_motor_set_power(EV3_PORT_A, 15);
            break;
        case GET_OBJ_2:
            if(now_arm_angle_A >= -300)ev3_motor_set_power(EV3_PORT_A, -20);
            else ev3_motor_set_power(EV3_PORT_A, 15);
            break;
        case GETDEBRIS:
            if(now_arm_angle_A >= -180)ev3_motor_set_power(EV3_PORT_A, -20);
            else ev3_motor_set_power(EV3_PORT_A, 15);
            break;




        default:
            break;
    }
    while (true) {
        now_arm_angle_A = ev3_motor_get_counts(EV3_PORT_A);
        if(now_arm_angle_A <= -249 && now_arm_angle_A >= -251 && mode == SET) break;
        if(now_arm_angle_A <= -299 && now_arm_angle_A >= -301 && mode == GET_OBJ_2) break;
        if(now_arm_angle_A <= -79 && now_arm_angle_A >= -81 && mode == GETDEBRIS) break;
        if(mode == CLOSE) break;
        if(mode == OPEN) break;
    }
    if(mode == SET)ev3_motor_stop(EV3_PORT_A, true);
    if(mode == GET_OBJ_2) ev3_motor_stop(EV3_PORT_A, true);
    if(mode == GETDEBRIS)ev3_motor_stop(EV3_PORT_A, true);
    now_arm_angle_A = ev3_motor_get_counts(EV3_PORT_A);
}


void arm_D(armmode_new_t mode) {
    now_arm_angle = ev3_motor_get_counts(EV3_PORT_D);
    switch (mode) {
        case DOWN:
            if(now_arm_angle <= 5)ev3_motor_set_power(EV3_PORT_D, 30);
            else ev3_motor_set_power(EV3_PORT_D, -80);
            break;
        case UP:
            if(now_arm_angle <= 300)ev3_motor_set_power(EV3_PORT_D, 80);
            else ev3_motor_set_power(EV3_PORT_D, -80);
            break;
        case ONE:
            if(now_arm_angle <= 265)ev3_motor_set_power(EV3_PORT_D, 80);
            else ev3_motor_set_power(EV3_PORT_D, -80);
            break;
        case TWO:
            if(now_arm_angle <= 480)ev3_motor_set_power(EV3_PORT_D, 80);
            else ev3_motor_set_power(EV3_PORT_D, -80);
            break;
        case THREE:
            if(now_arm_angle <= 695)ev3_motor_set_power(EV3_PORT_D, 80);
            else ev3_motor_set_power(EV3_PORT_D, -80);
            break;
        case FOUR:
            if(now_arm_angle <= 905)ev3_motor_set_power(EV3_PORT_D, 80);
            else ev3_motor_set_power(EV3_PORT_D, -80);
            break;
        case ALLUP:
            ev3_motor_set_power(EV3_PORT_D, 80);
            break;
        default:
            break;
    }
    now_mode = mode;
    //sta_cyc(ARM_CYC);
    while (true) {
        now_arm_angle = ev3_motor_get_counts(EV3_PORT_D);
        if(now_arm_angle <= 6 && now_arm_angle >= 4 && mode == DOWN) break;
        if(now_arm_angle <= 301 && now_arm_angle >= 299 && mode == UP) break;
        if(now_arm_angle <= 266 && now_arm_angle >= 264 && mode == ONE) break;
        if(now_arm_angle <= 481 && now_arm_angle >= 479 && mode == TWO) break;
        if(now_arm_angle <= 696 && now_arm_angle >= 694 && mode == THREE) break;
        if(now_arm_angle <= 906 && now_arm_angle >= 904 && mode == FOUR) break;
        if(now_arm_angle >= 940 && mode == ALLUP) break;
    }
    if(mode == ONE || mode == TWO || mode == THREE || mode == FOUR ||mode == ALLUP || mode == DOWN || mode == UP)ev3_motor_stop(EV3_PORT_D, true);
    now_arm_angle = ev3_motor_get_counts(EV3_PORT_D);
    tslp_tsk(100*MSEC);
}*/


void speed_arm(motor_port_t port, armmode_new_t mode, int power) {
    switch (mode){
    case OPEN:
    case UP:
        ev3_motor_set_power(port, power);
        break;
    case CLOSE:
    case DOWN:
        ev3_motor_set_power(port, -power);
        break; 
    default:
        break;
    }
}



void linetrace_cm_pd(int power, float gain, float d_gain, float cm, bool_t stop){
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    int now_angle_lb = 0;
    int now_angle_rc = 0;
    int average = 0;
    int lb_power;
    int rc_power;
    int now_reflect_2; 
    int now_reflect_3; 
    int last_diff = 0;
    int diff = 0;
    float d;
    int steering;

    now_reflect_2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
    now_reflect_3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
    diff = now_reflect_2 - now_reflect_3;
    while (true) {
        
        now_reflect_2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
        now_reflect_3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
        now_angle_lb = abs(ev3_motor_get_counts(EV3_PORT_B));
        now_angle_rc = abs(ev3_motor_get_counts(EV3_PORT_C));
        average = (now_angle_lb + now_angle_rc) / 2;
        last_diff = diff;
        diff = now_reflect_2 - now_reflect_3;
        d = (diff - last_diff);
        steering = diff * gain + d * d_gain;
        if(steering > 0) {
            lb_power = power;
            rc_power = power - (power * steering / 50);
            lb_power = -lb_power;
        }
        else {
            lb_power = power + (power * steering / 50);
            rc_power = power;
            lb_power = -lb_power;
        }
        ev3_motor_set_power(EV3_PORT_B, lb_power);
        ev3_motor_set_power(EV3_PORT_C, rc_power);

        if (average >= ROBOT1CM*cm) break;
    }
    ev3_motor_stop(EV3_PORT_B, stop);
    ev3_motor_stop(EV3_PORT_C, stop);
}

void linetrace_cm_pd_SP(float cm, int power, bool_t stop){
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    int now_angle_lb = 0;
    int now_angle_rc = 0;
    int average = 0;
    int lb_power;
    int rc_power;
    int now_reflect_2; 
    int now_reflect_3; 
    int last_diff = 0;
    int diff = 0;
    float d;
    int steering;
    float p_gain;
    float d_gain;
    gain_set(power, &p_gain, &d_gain);

    now_reflect_2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
    now_reflect_3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
    diff = now_reflect_2 - now_reflect_3;
    while (true) {
        
        now_reflect_2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
        now_reflect_3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
        now_angle_lb = abs(ev3_motor_get_counts(EV3_PORT_B));
        now_angle_rc = abs(ev3_motor_get_counts(EV3_PORT_C));
        average = (now_angle_lb + now_angle_rc) / 2;
        last_diff = diff;
        diff = now_reflect_2 - now_reflect_3;
        d = (diff - last_diff);
        steering = diff * p_gain + d * d_gain;
        if(steering > 0) {
            lb_power = power;
            rc_power = power - (power * steering / 50);
            lb_power = -lb_power;
        }
        else {
            lb_power = power + (power * steering / 50);
            rc_power = power;
            lb_power = -lb_power;
        }
        ev3_motor_set_power(EV3_PORT_B, lb_power);
        ev3_motor_set_power(EV3_PORT_C, rc_power);

        if (average >= ROBOT1CM*cm) break;
    }
    ev3_motor_stop(EV3_PORT_B, stop);
    ev3_motor_stop(EV3_PORT_C, stop);
}

void gain_set(int power, float *p_gain, float *d_gain){
    *p_gain = 0.3;
    *d_gain = 10;
    if(power > 0 && power <= 10){
        *p_gain = 1.4;   
        *d_gain = 80;   
    }
    if(power > 10 && power <= 20){
        *p_gain = 1.4;   
        *d_gain = 120;   
    }
    if(power == 24){
        *p_gain = -0.5;   //power24だけrightのセンサーでライントレース
        *d_gain = 60;   
    }
    if(power > 20 && power <= 30 && power != 24){
        *p_gain = 0.5;   
        *d_gain = 80;   
    }
    if(power > 30 && power <= 40){
        *p_gain = 0.25;   
        *d_gain = 70;   
    }
    if(power > 40 && power <= 50){
        *p_gain = 0.15;   
        *d_gain = 60;   
    }
    if(power > 50 && power <= 60){
        *p_gain = 0.16;   
        *d_gain = 70;   
    }
    if(power > 60 && power <= 70){
        *p_gain = 0.15;   
        *d_gain = 15;   
    }
    if(power > 70 && power <= 80){
        *p_gain = 0.15;
        *d_gain = 10;
    }
}

void gain_set_rgb(int power, float *p_gain, float *d_gain){
    *p_gain = 0.3;
    *d_gain = 10;
    if(power > 0 && power <= 10){
        *p_gain = 0.18;   
        *d_gain = 80;   
    }
    if(power > 10 && power <= 20){
        *p_gain = 0.25;   
        *d_gain = 80;   
        if(now_mode == THREE || now_mode == FOUR) {
            *p_gain = 0.14;   
            *d_gain = 80; 
        }
    }
    if(power == 24){
        *p_gain = -0.5;   //power24だけrightのセンサーでライントレース
        *d_gain = 60;   
    }
    if(power > 20 && power <= 30 && power != 24){
        *p_gain = 0.14;   
        *d_gain = 80;   
        if(now_mode == THREE || now_mode == FOUR) {
            *p_gain = 0.1;   
            *d_gain = 80; 
        }
    }
    if(power > 30 && power <= 40){
        *p_gain = 0.08;   
        *d_gain = 80;   
    }
    if(power > 40 && power <= 50){
        *p_gain = 0.05;   
        *d_gain = 100;   
    }
    if(power > 50 && power <= 60){
        *p_gain = 0.03;   
        *d_gain = 100;   
    }
    if(power > 60 && power <= 70){
        *p_gain = 0.02;   
        *d_gain = 60;   
    }
    if(power > 70 && power <= 80){
        *p_gain = 0.6;
        *d_gain = 10;
    }
}

void gain_set_pro(int power, float *p_gain, float *d_gain){
    *p_gain = 0.3;
    *d_gain = 10;
    if(power > 0 && power <= 10){
        *p_gain = 0.18;   
        *d_gain = 80;   
    }
    if(power > 10 && power <= 20){
        *p_gain = 0.25;   
        *d_gain = 80;   
        if(now_mode == THREE || now_mode == FOUR) {
            *p_gain = 0.17;   
            *d_gain = 80; 
        }
    }
    if(power == 24){
        *p_gain = -0.5;   //power24だけrightのセンサーでライントレース
        *d_gain = 60;   
    }
    if(power > 20 && power <= 30 && power != 24){
        *p_gain = 0.14;   
        *d_gain = 80;   
        if(now_mode == THREE || now_mode == FOUR) {
            *p_gain = 0.1;   
            *d_gain = 80; 
        }
    }
    if(power > 30 && power <= 40){
        *p_gain = 0.08;   
        *d_gain = 80;   
    }
    if(power > 40 && power <= 50){
        *p_gain = 0.05;   
        *d_gain = 100;   
    }
    if(power > 50 && power <= 60){
        *p_gain = 0.03;   
        *d_gain = 100;   
    }
    if(power > 60 && power <= 70){
        *p_gain = 0.02;   
        *d_gain = 60;   
    }
    if(power > 70 && power <= 80){
        *p_gain = 0.6;
        *d_gain = 10;
    }
}



void linetrace_color_pd_SP(port_t port, colorid_t color, int power, bool_t stop){
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    int lb_power;
    int rc_power;
    int now_reflect_2; 
    int now_reflect_3; 
    colorid_t color_2 = 0;
    colorid_t color_3 = 0; 
    int last_diff = 0;
    int diff = 0;
    float d;
    int steering;
    float p_gain;
    float d_gain;
    gain_set(power, &p_gain, &d_gain);

    now_reflect_2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
    now_reflect_3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
    diff = now_reflect_2 - now_reflect_3;
    if (port == RIGHT && power == 24) {
        diff = ev3_color_sensor_get_reflect(EV3_PORT_C) - 25;
    }
    if (port == LEFT && power == 24) {
        diff = ev3_color_sensor_get_reflect(EV3_PORT_C) - 25;
    }
    while (true) {
        
        now_reflect_2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
        now_reflect_3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
        if(color != COLOR_BLACK) {
            color_2 = ev3_color_sensor_get_color(EV3_PORT_2);
            color_3 = ev3_color_sensor_get_color(EV3_PORT_3);
        }
        last_diff = diff;
        diff = now_reflect_2 - now_reflect_3;
        d = (diff - last_diff);
        steering = diff * p_gain + d * d_gain;
        if(steering > 0) {
            lb_power = power;
            rc_power = power - (power * steering / 50);
            lb_power = -lb_power;
        }
        else {
            lb_power = power + (power * steering / 50);
            rc_power = power;
            lb_power = -lb_power;
        }
        ev3_motor_set_power(EV3_PORT_B, lb_power);
        ev3_motor_set_power(EV3_PORT_C, rc_power);
        if (port == RIGHT && color == COLOR_BLACK && now_reflect_3 <= 8 && power != 24)break;
        if (port == LEFT && color == COLOR_BLACK && now_reflect_2 <= 8 && power != 24)break;
        if (port == BOTH && color == COLOR_BLACK && now_reflect_3 <= 13 && now_reflect_2 <= 13)break;
        if (port == RIGHT && color_3 == color && color != COLOR_BLACK)break;
        if (port == LEFT && color_2 == color && color != COLOR_BLACK)break;
        if (port == BOTH && color_2 == color && color_3 == color && color != COLOR_BLACK)break;
        if (port == RIGHT && color == COLOR_BLACK && now_reflect_3 <= 13 && now_reflect_2 <= 13 && power == 24)break;
        if (port == LEFT && color == COLOR_BLACK && now_reflect_3 <= 13 && now_reflect_2 <= 13 && power == 24)break;
        
    }
    ev3_motor_stop(EV3_PORT_B, stop);
    ev3_motor_stop(EV3_PORT_C, stop);
}

void linetrace_cm_rgb_pd_SP(float cm, int power, bool_t stop){
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    int now_angle_lb = 0;
    int now_angle_rc = 0;
    int average = 0;
    int lb_power;
    int rc_power;
    rgb_raw_t rgb_val;//カラーセンサーの値を保存するために必要な変数(必須)
    float red2 = 0;
    float green2 = 0;
    float blue2 = 0;
    float red3 = 0;
    float green3 = 0;
    float blue3 = 0;
    float judgement2 = 0;
    float judgement3 = 0;
    int last_diff = 0;
    int diff = 0;
    float d;
    int steering;
    float p_gain;
    float d_gain;
    gain_set_rgb(power, &p_gain, &d_gain);

    ev3_color_sensor_get_rgb_raw(EV3_PORT_2, &rgb_val);
    red2 = rgb_val.r;
    green2 = rgb_val.g;
    blue2 = rgb_val.b;
    ev3_color_sensor_get_rgb_raw(EV3_PORT_3, &rgb_val);
    red3 = rgb_val.r;
    green3 = rgb_val.g;
    blue3 = rgb_val.b;
    judgement2 = (red2 + green2 + blue2);
    judgement3 = (red3 + green3 + blue3);
    diff = judgement2 - judgement3;

    
    
    while (true) {
        
        ev3_color_sensor_get_rgb_raw(EV3_PORT_2, &rgb_val);
        red2 = rgb_val.r;
        green2 = rgb_val.g;
        blue2 = rgb_val.b;
        ev3_color_sensor_get_rgb_raw(EV3_PORT_3, &rgb_val);
        red3 = rgb_val.r;
        green3 = rgb_val.g;
        blue3 = rgb_val.b;
        now_angle_lb = abs(ev3_motor_get_counts(EV3_PORT_B));
        now_angle_rc = abs(ev3_motor_get_counts(EV3_PORT_C));
        average = (now_angle_lb + now_angle_rc) / 2;
        judgement2 = (red2 + green2 + blue2);
        judgement3 = (red3 + green3 + blue3);
        last_diff = diff;
        diff = judgement2 - judgement3;
        d = (diff - last_diff);
        steering = diff * p_gain + d * d_gain;
        average = (now_angle_lb + now_angle_rc) / 2;
        if(steering > 0) {
            lb_power = power;
            rc_power = power - (power * steering / 50);
            //if(lb_power < 7) lb_power = 7;
            //if(rc_power < 7) rc_power = 7;
            lb_power = -lb_power;
        }
        else {
            lb_power = power + (power * steering / 50);
            rc_power = power;
            lb_power = -lb_power;
        }
        ev3_motor_set_power(EV3_PORT_B, lb_power);
        ev3_motor_set_power(EV3_PORT_C, rc_power);
        if (average >= ROBOT1CM*cm) break;
        
    }
    if (stop == true){
        ev3_motor_stop(EV3_PORT_B, stop);
        ev3_motor_stop(EV3_PORT_C, stop);
    }
}

void linetrace_rgb_pd_SP(port_t port, colorid_t color, int power, bool_t stop){
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    int lb_power;
    int rc_power;
    rgb_raw_t rgb_val;//カラーセンサーの値を保存するために必要な変数(必須)
    float red2 = 0;
    float green2 = 0;
    float blue2 = 0;
    float red3 = 0;
    float green3 = 0;
    float blue3 = 0;
    float judgement2 = 0;
    float judgement3 = 0;
    int last_diff = 0;
    int diff = 0;
    float d;
    int steering;
    float p_gain;
    float d_gain;

    colorid_t result2;
    colorid_t result3;
    gain_set_rgb(power, &p_gain, &d_gain);

    ev3_color_sensor_get_rgb_raw(EV3_PORT_2, &rgb_val);
    red2 = rgb_val.r;
    green2 = rgb_val.g;
    blue2 = rgb_val.b;
    ev3_color_sensor_get_rgb_raw(EV3_PORT_3, &rgb_val);
    red3 = rgb_val.r;
    green3 = rgb_val.g;
    blue3 = rgb_val.b;
    judgement2 = (red2 + green2 + blue2);
    judgement3 = (red3 + green3 + blue3);
    diff = judgement2 - judgement3;

    
    
    while (true) {
        
        ev3_color_sensor_get_rgb_raw(EV3_PORT_2, &rgb_val);
        red2 = rgb_val.r;
        green2 = rgb_val.g;
        blue2 = rgb_val.b;
        ev3_color_sensor_get_rgb_raw(EV3_PORT_3, &rgb_val);
        red3 = rgb_val.r;
        green3 = rgb_val.g;
        blue3 = rgb_val.b;
        judgement2 = (red2 + green2 + blue2);
        judgement3 = (red3 + green3 + blue3);
        if(judgement2 < 10) judgement2 +=5;
        if(judgement3 < 10) judgement3 +=5;
        last_diff = diff;
        diff = judgement2 - judgement3;
        d = (diff - last_diff);
        steering = diff * p_gain + d * d_gain;
        if(steering > 0) {
            lb_power = power;
            rc_power = power - (power * steering / 50);
            lb_power = -lb_power;
        }
        else {
            lb_power = power + (power * steering / 50);
            rc_power = power;
            lb_power = -lb_power;
        }
        ev3_motor_set_power(EV3_PORT_B, lb_power);
        ev3_motor_set_power(EV3_PORT_C, rc_power);
        result2 = rgb_color(red2, green2, blue2);
        result3 = rgb_color(red3, green3, blue3);
        if (port == RIGHT && color == COLOR_BLACK && result3 == COLOR_BLACK && power != 24)break;
        if (port == LEFT && color == COLOR_BLACK && result2 == COLOR_BLACK && power != 24)break;
        if (port == BOTH && color == COLOR_BLACK && result2 == COLOR_BLACK && result3 == COLOR_BLACK)break;
        if (port == RIGHT && result3 == color && result3 != COLOR_BLACK)break;
        if (port == LEFT && result2 == color && color != COLOR_BLACK)break;
        if (port == BOTH && result2 == color && result3 == color && color != COLOR_BLACK)break;
        if (port == RIGHT && color == COLOR_BLACK && result2 == COLOR_BLACK && result3 == COLOR_BLACK && power == 24)break;
        if (port == LEFT && color == COLOR_BLACK && result2 == COLOR_BLACK && result3 == COLOR_BLACK && power == 24)break;
        
    }
    if(stop == true){
        ev3_motor_stop(EV3_PORT_B, stop);
        ev3_motor_stop(EV3_PORT_C, stop);
    }
}

/*void linetrace_cm_rgb_pd_SP_pro(float start_cm, float speed_cm, float last_cm, int power, bool_t stop){
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    int cm = start_cm + speed_cm + last_cm;
    int now_angle_lb = 0;
    int now_angle_rc = 0;
    int average = 0;
    int lb_power;
    int rc_power;
    rgb_raw_t rgb_val;//カラーセンサーの値を保存するために必要な変数(必須)
    float red2 = 0;
    float green2 = 0;
    float blue2 = 0;
    float red3 = 0;
    float green3 = 0;
    float blue3 = 0;
    float judgement2 = 0;
    float judgement3 = 0;
    int last_diff = 0;
    int diff = 0;
    float d;
    int now_power = 20;
    if (start_cm == 0) now_power = power;
    int steering;
    float p_gain;
    float d_gain;
    gain_set_pro(power, &p_gain, &d_gain);

    ev3_color_sensor_get_rgb_raw(EV3_PORT_2, &rgb_val);
    red2 = rgb_val.r;
    green2 = rgb_val.g;
    blue2 = rgb_val.b;
    ev3_color_sensor_get_rgb_raw(EV3_PORT_3, &rgb_val);
    red3 = rgb_val.r;
    green3 = rgb_val.g;
    blue3 = rgb_val.b;
    judgement2 = (red2 + green2 + blue2);
    judgement3 = (red3 + green3 + blue3);
    diff = judgement2 - judgement3;

    
    gain_set_pro(now_power, &p_gain, &d_gain);
    while (true) {
        if(average <= start_cm*ROBOT1CM) {
            now_power = now_power + 0.0025;
        }
        else if(average >= (cm*ROBOT1CM - last_cm*ROBOT1CM)) {
            now_power = now_power - 0.0035;
        }
        else {
            now_power = power;
        }
        
        
        ev3_color_sensor_get_rgb_raw(EV3_PORT_2, &rgb_val);
        red2 = rgb_val.r;
        green2 = rgb_val.g;
        blue2 = rgb_val.b;
        ev3_color_sensor_get_rgb_raw(EV3_PORT_3, &rgb_val);
        red3 = rgb_val.r;
        green3 = rgb_val.g;
        blue3 = rgb_val.b;
        now_angle_lb = abs(ev3_motor_get_counts(EV3_PORT_B));
        now_angle_rc = abs(ev3_motor_get_counts(EV3_PORT_C));
        average = (now_angle_lb + now_angle_rc) / 2;
        judgement2 = (red2 + green2 + blue2);
        judgement3 = (red3 + green3 + blue3);
        last_diff = diff;
        diff = judgement2 - judgement3;
        d = (diff - last_diff);
        steering = diff * p_gain + d * d_gain;
        average = (now_angle_lb + now_angle_rc) / 2;
        if(steering > 0) {
            lb_power = power;
            rc_power = power - (power * steering / 50);
            //if(lb_power < 7) lb_power = 7;
            //if(rc_power < 7) rc_power = 7;
            lb_power = -lb_power;
        }
        else {
            lb_power = power + (power * steering / 50);
            rc_power = power;
            lb_power = -lb_power;
        }
        ev3_motor_set_power(EV3_PORT_B, lb_power);
        ev3_motor_set_power(EV3_PORT_C, rc_power);
        if (average >= ROBOT1CM*cm) break;
        
    }
    if (stop == true){
        ev3_motor_stop(EV3_PORT_B, stop);
        ev3_motor_stop(EV3_PORT_C, stop);
    }
}*/

colorid_t rgb_color(float r, float g, float b){
    float judgement = r + g + b;
    float max = 0;
    float min = 0;
    //float s = 0;
    float h = 0;
    float v = 0;


    if (r >= g && r >= b) { 
        max = r;
        h = 60 * ((g - b) / (max - min));
    }
    if (g >= r && g >= b) { 
        max = g;
        h = 60 * ((b - r) / (max - min)) + 120;
    }
    if (b >= r && b >= g) { 
        max = b;
        h = 60 * ((r - g) / (max - min)) + 240;
    }

    //s = (max - min) / max * 100;
    v = max / 255 * 100;

    colorid_t result = 0;
    if(v > 70 && judgement > 400) result = COLOR_WHITE;
    else if(v < 20 || judgement < 100) result = COLOR_BLACK;
    else if((h > 320 || h < 10) && g < 100) result = COLOR_RED;
    else if(h > 35 && h < 100) result = COLOR_YELLOW;
    else if(h >= 10 && h < 35) result = COLOR_BROWN;
    else if(h >= 100 && h < 140) result = COLOR_GREEN;
    else if(h >= 140 && h < 250) result = COLOR_BLUE;

    if(g > 130 && b < 40) result = COLOR_YELLOW;

    return result;

}

void color_check(port_t port){
    rgb_raw_t rgb_val;//カラーセンサーの値を保存するために必要な変数(必須)
    float red = 0;
    float green = 0;
    float blue = 0;
    ev3_color_sensor_get_rgb_raw(port, &rgb_val);
    ev3_color_sensor_get_rgb_raw(port, &rgb_val);
    red = rgb_val.r;
    green = rgb_val.g;
    blue = rgb_val.b;
    obj_color = rgb_color(red, green, blue);
    fprintf(bt, "sensor:r%f g:%f b:%f\r\njudge_color %d\r\n", red, green, blue, obj_color);
}

void gb_check(port_t port){
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
    if(green > blue) bg_color = COLOR_GREEN;
    else bg_color = COLOR_BLUE;
    fprintf(bt, "sensor:r%d g:%d b:%d\r\njudge_color %d\r\n", red, green, blue, bg_color);
}

void straight_on(int power){
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    ev3_motor_set_power(EV3_PORT_B, -power);
    ev3_motor_set_power(EV3_PORT_C, power);
}

void straight_off(float cm, bool_t logic){
    int now_motor_angle_rc = 0;
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    while (true) {
        now_motor_angle_rc = abs(ev3_motor_get_counts(EV3_PORT_B));
        if(cm*ROBOT1CM <= now_motor_angle_rc) break;
    }
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    if (logic == true) {
        ev3_motor_stop(EV3_PORT_B, true);
        ev3_motor_stop(EV3_PORT_C, true);
    }
}

void linetrace_rgb(float cm, int power, bool_t stop){
    rgb_raw_t rgb_val;//カラーセンサーの値を保存するために必要な変数(必須)
    float red2 = 0;
    float green2 = 0;
    float blue2 = 0;
    float red3 = 0;
    float green3 = 0;
    float blue3 = 0;
    float judgement = 0;
    int lb_power;
    int rc_power;
    int steering;
    int last_diff = 0;
    int diff = 0;
    float d;
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    int now_angle_lb = 0;
    int now_angle_rc = 0;
    int average = 0;

    int before_average = 0;
    int zure = 0;
    float theta = 0;
    float x = 0;
    float y = 0;
    float now_x = 0;
    float now_y = 0;
    int count = 0;
    float a = 0;


    ev3_color_sensor_get_rgb_raw(EV3_PORT_2, &rgb_val);
    red2 = rgb_val.r;
    green2 = rgb_val.g;
    blue2 = rgb_val.b;
    ev3_color_sensor_get_rgb_raw(EV3_PORT_3, &rgb_val);
    red3 = rgb_val.r;
    green3 = rgb_val.g;
    blue3 = rgb_val.b;
    judgement = (red2 + green2 + blue2) - (red3 + green3 + blue3);
    diff = judgement;
    while (true){
        now_angle_lb = abs(ev3_motor_get_counts(EV3_PORT_B));
        now_angle_rc = abs(ev3_motor_get_counts(EV3_PORT_C));

        
        zure = now_angle_lb - now_angle_rc;
        theta = zure * 0.181818181818;
        
        average = (now_angle_lb + now_angle_rc) / 2;
        count = count + 1;
        if(count >= 100){
            a = average - before_average;
            y = sin(theta) * a;
            x = cos(theta) * a;
            now_x = x + now_x;
            now_y = y + now_y;
            before_average = average;
            count = 0;
        }
        

        ev3_color_sensor_get_rgb_raw(EV3_PORT_2, &rgb_val);
        red2 = rgb_val.r;
        green2 = rgb_val.g;
        blue2 = rgb_val.b;
        ev3_color_sensor_get_rgb_raw(EV3_PORT_3, &rgb_val);
        red3 = rgb_val.r;
        green3 = rgb_val.g;
        blue3 = rgb_val.b;
        judgement = ((red2 + green2 + blue2) - (red3 + green3 + blue3)) / 7.5;

        last_diff = diff;
        diff = judgement;
        d = (diff - last_diff);
        steering = judgement * 0.4 + d * 0;
        //steering = judgement * 0.4 + d * 80  po=18;
        
        if(steering > 0) {
            lb_power = power;
            rc_power = power - (power * steering / 50);
            lb_power = -lb_power;
        }
        else {
            lb_power = power + (power * steering / 50);
            rc_power = power;
            lb_power = -lb_power;
        }
        ev3_motor_set_power(EV3_PORT_B, lb_power);
        ev3_motor_set_power(EV3_PORT_C, rc_power);
        fprintf(bt,"%f,%f,%f\r\n", now_x, now_y, theta);
        if (now_x >= ROBOT1CM * cm) break;
    }
    ev3_motor_stop(EV3_PORT_B, stop);
    ev3_motor_stop(EV3_PORT_C, stop);

}

void straight_dr(float cm, int power, bool_t stop){
    rgb_raw_t rgb_val;//カラーセンサーの値を保存するために必要な変数(必須)
    float red2 = 0;
    float green2 = 0;
    float blue2 = 0;
    float red3 = 0;
    float green3 = 0;
    float blue3 = 0;
    float judgement = 0;
    int lb_power;
    int rc_power;
    int steering;
    int last_diff = 0;
    int diff = 0;
    float d;
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    int now_angle_lb = 0;
    int now_angle_rc = 0;
    int average = 0;

    int before_average = 0;
    int zure = 0;
    float theta = 0;
    float x = 0;
    float y = 0;
    float now_x = 0;
    float now_y = 0;
    int count = 0;
    float a = 0;


    ev3_color_sensor_get_rgb_raw(EV3_PORT_2, &rgb_val);
    red2 = rgb_val.r;
    green2 = rgb_val.g;
    blue2 = rgb_val.b;
    ev3_color_sensor_get_rgb_raw(EV3_PORT_3, &rgb_val);
    red3 = rgb_val.r;
    green3 = rgb_val.g;
    blue3 = rgb_val.b;
    judgement = (red2 + green2 + blue2) - (red3 + green3 + blue3);
    diff = judgement;
    while (true){
        now_angle_lb = abs(ev3_motor_get_counts(EV3_PORT_B));
        now_angle_rc = abs(ev3_motor_get_counts(EV3_PORT_C));

        
        zure = now_angle_lb - now_angle_rc;
        theta = zure * 0.181818181818;
        
        average = (now_angle_lb + now_angle_rc) / 2;
        count = count + 1;
        if(count >= 100){
            a = average - before_average;
            y = sin(theta) * a;
            x = cos(theta) * a;
            now_x = x + now_x;
            now_y = y + now_y;
            before_average = average;
            count = 0;
        }
        

        ev3_color_sensor_get_rgb_raw(EV3_PORT_2, &rgb_val);
        red2 = rgb_val.r;
        green2 = rgb_val.g;
        blue2 = rgb_val.b;
        ev3_color_sensor_get_rgb_raw(EV3_PORT_3, &rgb_val);
        red3 = rgb_val.r;
        green3 = rgb_val.g;
        blue3 = rgb_val.b;
        judgement = ((red2 + green2 + blue2) - (red3 + green3 + blue3)) / 7.5;

        last_diff = diff;
        diff = judgement;
        d = (diff - last_diff);
        steering = judgement * 0.4 + d * 0;
        //steering = judgement * 0.4 + d * 80  po=18;
        
        if(steering > 0) {
            lb_power = power;
            rc_power = power - (power * steering / 50);
            lb_power = -lb_power;
        }
        else {
            lb_power = power + (power * steering / 50);
            rc_power = power;
            lb_power = -lb_power;
        }
        ev3_motor_set_power(EV3_PORT_B, lb_power);
        ev3_motor_set_power(EV3_PORT_C, rc_power);
        fprintf(bt,"%f,%f,%f\r\n", now_x, now_y, theta);
        if (now_x >= ROBOT1CM * cm) break;
    }
    ev3_motor_stop(EV3_PORT_B, stop);
    ev3_motor_stop(EV3_PORT_C, stop);

}



void straight(float cm, int power){
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    //if (power == 30) power = 40;
    int steer = 0;
    int lb_power;
    int rc_power;
    int now_right_angle = 0;
    int now_left_angle = 0;
    int now_angle = 0; 
    int diff = 0;
    int maximum = abs(power);
    float changing_power = 17;
    int sign = power/abs(power);
    int goal_angle = cm * ROBOT1CM;
    while (true){
        now_left_angle = abs(ev3_motor_get_counts(EV3_PORT_B));
        now_right_angle = abs(ev3_motor_get_counts(EV3_PORT_C));
        now_angle = (now_right_angle + now_left_angle) / 2;
        diff = now_right_angle - now_left_angle;
        steer = (diff*4);
        if(steer > 0) {
            lb_power = changing_power;
            rc_power = changing_power - (changing_power * steer / 50);
            rc_power = sign*rc_power;
            lb_power = -sign*lb_power;
        }
        else {
            lb_power = changing_power + (changing_power * steer / 50);
            rc_power = changing_power;
            rc_power = sign*rc_power;
            lb_power = -sign*lb_power;
        }
        if (now_left_angle <= goal_angle)ev3_motor_set_power(EV3_PORT_B, lb_power);
        else ev3_motor_stop(EV3_PORT_B, true);
        if (now_right_angle <= goal_angle)ev3_motor_set_power(EV3_PORT_C, rc_power);
        else ev3_motor_stop(EV3_PORT_C, true);
        if (changing_power < maximum && goal_angle - (6*ROBOT1CM) > now_angle) changing_power = changing_power + 0.005;
        if (goal_angle - (6*ROBOT1CM) <= now_angle) changing_power = changing_power - 0.01;
        if (changing_power <= 17) changing_power = 17;
        if (goal_angle <= now_left_angle && goal_angle <= now_right_angle) break;   
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
}

void start_1(){
   
    //pattern1
    speed_arm(EV3_PORT_D, UP, 35);
    straight(30, 50);
    arm_D(THREE);
    turn(15, 30, -30);
    speed_arm(EV3_PORT_D, UP, 20);
    straight(20, 50);
    arm_D(ALLUP);
    straight_on(20);
    while (true){
        color_3 = ev3_color_sensor_get_color(EV3_PORT_2);
        if(color_3 == COLOR_WHITE) break;
    }
    while (true){
        color_3 = ev3_color_sensor_get_color(EV3_PORT_2);
        if(color_3 == COLOR_BLACK) break;
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
    turn(160, -30, 30);
    arm_D(THREE);
    straight(5, -20);
    tslp_tsk(100*MSEC);
    turn(55, 30, -30);




    linetrace_cm_rgb_pd_SP(10, 20, false);
    linetrace_color_pd_SP(BOTH, COLOR_BLACK, 20, true);
    tslp_tsk(100*MSEC);
    straight(2.5, -20);
    tslp_tsk(100*MSEC);
    turn(90, 30, -30);




    straight(8, 30);
    arm_D(ALLUP);
    tslp_tsk(100*MSEC);
    turn(189, 30, -30);
    tslp_tsk(100*MSEC);
    turn(9, -30, 30);
    arm_D(THREE);
    straight(10, 30);
    arm_D(ALLUP);









    straight(51, 50);
    turn(60, 30, -30);
    straight(30, 40);
    turn(60, 30, 0);
    tslp_tsk(100*MSEC);
    speed_arm(EV3_PORT_D, DOWN, 30);
    straight(6, 30);








    straight_on(-50);
    while (true){
    color_2 = ev3_color_sensor_get_color(EV3_PORT_2);
    if(color_2 != COLOR_WHITE) break;
    }
    straight_off(5, false);
    while (true){
    color_2 = ev3_color_sensor_get_color(EV3_PORT_2);
    if(color_2 == COLOR_WHITE) break;
    }
    straight_off(12, true);
    tslp_tsk(200*MSEC);
    arm_D(UP);
    turn(90, 30, -30);
    straight_on(-40);
    tslp_tsk(500*MSEC);
    straight_on(-20);
    tslp_tsk(300*MSEC);
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
    straight(2.2, 20);
    turn(91, 30, -30);
    straight_on(30);
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



}



void start_2() {
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
    tslp_tsk(100*MSEC);
    turn(95, 30, -30);
    straight(10, 30);












    speed_arm(EV3_PORT_D, UP, 34);
    linetrace_cm_rgb_pd_SP(18, 20, true);
    arm_D(THREE);
    linetrace_cm_rgb_pd_SP(55, 40, false);








    linetrace_rgb_pd_SP(BOTH, COLOR_BLACK, 20, true);
    tslp_tsk(100*MSEC);
    straight(3, -20);
    tslp_tsk(100*MSEC);
    turn(90, 30, -30);



    straight(8, 30);
    arm_D(ALLUP);
    straight(15, -40);
    turn(200, 30, -30);
    arm_D(THREE);
    straight(8, 30);
    arm_D(ALLUP);
    straight(48, 50);
    turn(140, 30, 0);
    straight(12, 40);
    arm_D(UP);
    straight_on(-40);
    while (true){
    color_3 = ev3_color_sensor_get_color(EV3_PORT_3);
    if(color_3 != COLOR_WHITE) break;
    }
    while (true){
    color_3 = ev3_color_sensor_get_color(EV3_PORT_3);
    if(color_3 == COLOR_WHITE) break;
    }
    straight_on(-30);
    straight_off(12, true);
    turn(90, 30, -30);
    straight_on(-45);
    tslp_tsk(500*MSEC);
    straight_on(-20);
    tslp_tsk(300*MSEC);
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
    straight(2, 20);
    turn(91, 30, -30);
    straight_on(30);
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
   
}



void area_1(){
    straight(8.5, 20);
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

   
   


    straight(13.2, -30);
    turn(90, -30, 30);
    straight(10, 40);
    straight_on(30);
    //白線見てから黒線で止まるストレート
    while (true){
        color_3 = ev3_color_sensor_get_color(EV3_PORT_3);
        if(color_3 == COLOR_WHITE) break;
    }
    while (true){
        now_reflect_3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
        if(now_reflect_3 <= 10) break;
    }
    straight_off(2, true);    
    linetrace_cm_rgb_pd_SP(16, 20, true);
    straight(28, 40);
    linetrace_rgb_pd_SP(BOTH, COLOR_BLACK, 20, true);
    
    straight(4, 20);
    tslp_tsk(100*MSEC);
    turn(180, 30, -30);


    linetrace_rgb_pd_SP(BOTH, COLOR_YELLOW, 20, true);
    straight(4.5, -20);
    arm_A(OPEN);
    turn(40, 0, 30);
    turn(5, 0, -29);
    arm_D(TWO);
    arm_A(CLOSE);
    arm_D(THREE);
    turn(36, 0, -30);
    straight(17, -40);
    speed_arm(EV3_PORT_D, DOWN, 40);
    turn(90, -30, 30);
    arm_D(DOWN);
}

void area_2(){
    linetrace_cm_rgb_pd_SP(12, 20, false);
    straight_on(50);
    while(true){
        color_2 = ev3_color_sensor_get_color(EV3_PORT_2);
        if(color_2 == COLOR_RED) break;
    }
    straight_off(12, true);
    tslp_tsk(100*MSEC);
    turn(90, 30, -30);
    speed_arm(EV3_PORT_D, UP, 30);
    straight_on(-40);
    tslp_tsk(500*MSEC);
    straight_on(-20);
    tslp_tsk(200*MSEC);
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
    arm_D(UP);
    tslp_tsk(100*MSEC);
    straight(2, 20);
    tslp_tsk(100*MSEC);
    turn(91, 30, -30);
    straight_on(20);
    while (true){
        color_3 = ev3_color_sensor_get_color(EV3_PORT_3);
        if(color_3 == COLOR_WHITE) break;
    }
    while (true){
        color_3 = ev3_color_sensor_get_color(EV3_PORT_3);
        if(color_3 == COLOR_RED) break;
    }

   
    straight(8.5, 20);




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

    if(start == 1) {
        straight(7.5, 30);

        turn(90, -30, 30);
        straight(10, 40);
        straight_on(30);
        //白線見てから黒線で止まるストレート
        while (true){
            color_3 = ev3_color_sensor_get_color(EV3_PORT_3);
            if(color_3 == COLOR_WHITE) break;
        }
        while (true){
            now_reflect_3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
            if(now_reflect_3 <= 10) break;
        }
        straight_off(4, true);    

    }
    if(start == 2) {
        straight(12, -40);
        turn(90, -30, 30);
        arm_D(DOWN);
        straight_on(50);
        //白線見てから黒線で止まるストレート
        while (true){
            color_3 = ev3_color_sensor_get_color(EV3_PORT_3);
            if(color_3 == COLOR_WHITE) break;
        }
        straight_on(20);
        while (true){
            now_reflect_3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
            if(now_reflect_3 <= 10) break;
        }
        straight(3, 20);
        turn(90, 30, -30);
        linetrace_cm_rgb_pd_SP(6, 20, false);
        linetrace_color_pd_SP(LEFT, COLOR_BLACK, 20, false);
        straight(5, 30);
        turn(90, -30, 30);
        
    }

    linetrace_cm_rgb_pd_SP(8, 20, true);
    arm_A(CLOSE);
    arm_D(THREE);
    linetrace_cm_rgb_pd_SP(8, 20, true);
    tslp_tsk(100*MSEC);


   
    turn(40, 0, 29);
    arm_D(TWO);
    arm_A(OPEN);
    arm_D(FOUR);    
    arm_A(CLOSE);
    arm_D(ALLUP);
    straight(15, -30);
    /*speed_arm(EV3_PORT_D, DOWN, 60);
    tslp_tsk(300*MSEC);*/
    arm_D(DOWN);
}

void water(){
    straight(39, 50);
    tslp_tsk(100*MSEC);
    turn(90, 30, -30);
    straight(20, -60);
    straight_on(-30);
    tslp_tsk(300*MSEC);
    straight_on(-10);
    tslp_tsk(100*MSEC);
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
    tslp_tsk(100*MSEC);

    straight(27.5, 40);

    turn(90, -30, 30);
    straight(30, 60);
    arm_D(UP);

    speed_arm(EV3_PORT_D, DOWN, 25);
    straight(6, -30);
    arm_D(DOWN);
    arm_A(CLOSE);
    tslp_tsk(300*MSEC);

    straight(12, -30);
    turn(145, 30, -30);
    arm_A(GETDEBRIS);
    straight_on(80);
    straight_off(50, false);
    straight_on(40);
    arm_A(CLOSE);
    while (true){
        color_3 = ev3_color_sensor_get_color(EV3_PORT_3);
        if(color_3 == COLOR_WHITE) break;
    }
    while (true){
        color_3 = ev3_color_sensor_get_color(EV3_PORT_3);
        if(color_3 == COLOR_BLACK) break;
    }
    while (true){
        color_3 = ev3_color_sensor_get_color(EV3_PORT_3);
        if(color_3 == COLOR_WHITE) break;
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
    turn(35, 30, -30);
    


    linetrace_cm_rgb_pd_SP(15, 20, false);
    if(start == 2){
        linetrace_rgb_pd_SP(BOTH, COLOR_RED, 50, false);
        straight_on(70);
        straight_off(47, true);
        turn(120, 0, 80);
    }
    if(start == 1){
        linetrace_rgb_pd_SP(BOTH, COLOR_RED, 50, false);
        straight(40, 60);
        turn(40, -30, 30);
        speed_arm(EV3_PORT_A, OPEN, 8);
        straight(20, 50);
        arm_A(GETDEBRIS);
        straight(14, -50);
        turn(130, 30, -30);
        speed_arm(EV3_PORT_D, UP, 50);
        straight(15, -50);
        straight_on(-35);
        tslp_tsk(400*MSEC);
        straight_on(-10);
        arm_D(ALLUP);
        tslp_tsk(100*MSEC);
        ev3_motor_stop(EV3_PORT_B, true);
        ev3_motor_stop(EV3_PORT_C, true);
        straight(20, 40);
        turn(90, -30, 30);
        straight(31, 60);
        arm_D(UP);
    }

}

void take_house(){
    speed_arm(EV3_PORT_D, DOWN, 35);
    arm_A(SET);
    straight(4.5, 20);
    arm_D(DOWN);
    straight(2.5, 20);
    arm_A(CLOSE);
    tslp_tsk(300*MSEC);
    speed_arm(EV3_PORT_D, UP, 40);
    straight(7, -30);
    arm_D(UP);
}

void build_house(){
    straight(11, 10);
    speed_arm(EV3_PORT_D, DOWN, 30);
    straight(4.3, -20);
    arm_D(SETNEW);
    arm_A(GET_OBJ_2);
    arm_D(DOWN);
    straight_on(10);
    straight_off(0.5, false);
    arm_A(CLOSE);//armのところね
    straight_off(1.5, true);
    speed_arm(EV3_PORT_D, UP, 37);
    straight(8.7, -30);
    arm_D(UP);
}

void arm_task(intptr_t exinf) {
    now_arm_angle = ev3_motor_get_counts(EV3_PORT_D);
    fprintf(bt, "----TASKEND----\r\n");
    stp_cyc(ARM_CYC);
    /*if(now_arm_angle <= -659 && now_arm_angle >= -661 && now_mode == SETNEW) {
        ev3_motor_stop(EV3_PORT_D, true);
        fprintf(bt, "----TASKEND----\r\n");
        stp_cyc(ARM_CYC);
    }
    if(now_arm_angle <= -519 && now_arm_angle >= -521 && now_mode == SETNEW_2) {
        ev3_motor_stop(EV3_PORT_D, true);
        fprintf(bt, "----TASKEND----\r\n");
        stp_cyc(ARM_CYC);
    }
    if(now_arm_angle <= -339 && now_arm_angle >= -350 && now_mode == UP) {
        ev3_motor_stop(EV3_PORT_D, true);
        fprintf(bt, "----TASKEND----\r\n");
        stp_cyc(ARM_CYC);
    }
    if(now_arm_angle <= -789 && now_arm_angle >= -791 && now_mode == DOWN) {
        ev3_motor_stop(EV3_PORT_D, true);
        fprintf(bt, "----TASKEND----\r\n");
        stp_cyc(ARM_CYC);
    }
    if(now_arm_angle <= 1 && now_arm_angle >= -1 && now_mode == ALLUP) {
        ev3_motor_stop(EV3_PORT_D, true);
        fprintf(bt, "----TASKEND----\r\n");
        stp_cyc(ARM_CYC);
    }*/
}



void main_task(intptr_t unused) {

    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);

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
    ev3_sensor_config(PortSensorGyro, GYRO_SENSOR);
    
    fprintf(bt, "----GAME_START----\r\n");
    file=fopen(logfilename,"a");//ファイルをオープン(名前の指定)
    fprintf(file, "----GAME_START----\r\n");
    fclose(file);

    battery = ev3_battery_voltage_mV();
    fprintf(bt, "BATTERY:%d\r\n", battery);
    file=fopen(logfilename,"a");//ファイルをオープン(名前の指定)
    fprintf(file, "BATTERY:%d\r\n", battery);
    fclose(file);
    
    

    /* ここからコーディング */


    


    // OPEN UP の後、start 2
    /*arm_D(DOWN);
    arm_A(CLOSE);
    arm_D(UP);
    straight(10, 30);
    arm_A(OPEN);
    arm_D(DOWN);
    arm_A(CLOSE);
    straight(75, -30);
    straight_on(-20);
    while (true){
        color_2 = ev3_color_sensor_get_color(EV3_PORT_2);
        if(color_2 == COLOR_RED) break;
    }
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
    turn(90, -30 ,30);
    straight_on(-30);
    tslp_tsk(300*MSEC);
    straight_on(-10);
    tslp_tsk(300*MSEC);
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);*/

    


    
    tslp_tsk(300*MSEC);
   


    while(ev3_button_is_pressed(ENTER_BUTTON) == false){}    
    tslp_tsk(200*MSEC);




    ev3_motor_reset_counts(EV3_PORT_A);
    ev3_motor_reset_counts(EV3_PORT_D);



    
    yellow = 4;
    start = 2;

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
    tslp_tsk(500*MSEC);
    turn(93, 30, -30);
    tslp_tsk(100*MSEC);
    straight(10, 20);
    linetrace_cm_rgb_pd_SP(8, 20, false);
    linetrace_rgb_pd_SP(LEFT, COLOR_BLACK, 20, true);
    straight(2,20);
    turn(90, -30, 30);
    arm_D(THREE);
    arm_A(OPEN);
    straight(20,30);
    arm_D(DOWN);
    arm_A(CLOSE);
    straight(30, -40);
    straight_on(-30);
    tslp_tsk(700*MSEC);
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_C, true);
    straight(2.5, 20);
    turn(90, 30, -30);
    straight(4, -30);
    arm_A(OPEN);
    arm_D(ONE);
    arm_A(CLOSE);
    arm_D(UP);
    straight(10, -30);
    arm_A(OPEN);

    


    stopping();









}