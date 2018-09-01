#ifndef _ETROBO_BALANCER_H_INCLUDED
#define _ETROBO_BALANCER_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)        ((void*) 0)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)   ((void) 0)
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)      ((void*) 0)
#endif

#define CMD_MAX     100.0F         /* 前進/旋回命令絶対最大値 */
#define DEG2RAD     0.01745329238F /* 角度単位変換係数(=pi/180) */
#define EXEC_PERIOD 0.00500000000F /* バランス制御実行周期(秒) *//* sample_c4の処理時間考慮 */

extern void balance_init(void);
extern void balance_control(float args_cmd_forward, float args_cmd_turn,
  float args_gyro, float args_gyro_offset, float args_theta_m_l,
  float args_theta_m_r, float args_battery, signed char *ret_pwm_l, signed char
  *ret_pwm_r);

#ifdef __cplusplus
}
#endif

#endif 