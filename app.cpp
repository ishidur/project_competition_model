
#include "ev3api.h"
#include "app.h"

#include "Phase/PhaseExecuter.h"
#include "AppliedHardware/Communication/Communication.h"
#include "Balancer/balancer.h"

using namespace Phase;
using namespace AppliedHardware::Communication;

// #define DEBUG
#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

void initialize(void);

static const sensor_port_t
    touch_sensor      = EV3_PORT_1,
    color_sensor      = EV3_PORT_2,
    ultrasonic_sensor = EV3_PORT_3,
    gyro_sensor       = EV3_PORT_4;

static const motor_port_t
    tail_motor        = EV3_PORT_A,
    right_motor       = EV3_PORT_B,
    left_motor        = EV3_PORT_C;

void main_task(intptr_t unused){
    printf("app initialize\n");
    initialize();
    printf("app initialize done\n");

    ev3_led_set_color(LED_GREEN);

    ev3_speaker_play_tone(NOTE_C4, 100);
    tslp_tsk(100);
    ev3_speaker_play_tone(NOTE_C4, 100);

    printf("app PhaseExecuter\n");
    PhaseExecuter* phaseExecuter = new PhaseExecuter();
    phaseExecuter->ExecutePhases();
    printf("app PhaseExecuter dones\n");

    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    ext_tsk();
}

void com_task(intptr_t unused){
    printf("start COM_TASK\n");
    Communication* com = Communication::GetInstance();
    while(1){
        if(!com->GetTaskStop()){
            com->ReceiveTask();
        }   
        tslp_tsk(10);
    }
}

void initialize(){
    ev3_lcd_fill_rect(0,0,EV3_LCD_WIDTH,EV3_LCD_HEIGHT,EV3_LCD_WHITE);

    ev3_sensor_config(ultrasonic_sensor,ULTRASONIC_SENSOR);
    
    ev3_sensor_config(color_sensor,COLOR_SENSOR);
    ev3_color_sensor_get_reflect(color_sensor);

    ev3_sensor_config(touch_sensor,TOUCH_SENSOR);
    
    ev3_sensor_config(gyro_sensor,GYRO_SENSOR);
    
    ev3_motor_config(left_motor,LARGE_MOTOR);
    ev3_motor_config(right_motor,LARGE_MOTOR);
    ev3_motor_config(tail_motor,LARGE_MOTOR);
    
    ev3_motor_reset_counts(tail_motor);
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    
    ev3_gyro_sensor_reset(gyro_sensor);
    balance_init();
}
