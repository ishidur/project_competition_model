#include "balancer.h"
#include "balancer_private.h"

#define rt_SATURATE(sig,ll,ul)         (((sig) >= (ul)) ? (ul) : (((sig) <= (ll)) ? (ll) : (sig)) )

static float ud_err_theta;          /* 左右車輪の平均回転角度(θ)目標誤差状態値 */
static float ud_psi;                /* 車体ピッチ角度(ψ)状態値 */
static float ud_theta_lpf;          /* 左右車輪の平均回転角度(θ)状態値 */
static float ud_theta_ref;          /* 左右車輪の目標平均回転角度(θ)状態値 */
static float ud_thetadot_cmd_lpf;   /* 左右車輪の目標平均回転角速度(dθ/dt)状態値 */


void balance_control(float args_cmd_forward, float args_cmd_turn, float
                     args_gyro, float args_gyro_offset, float
                     args_theta_m_l, float args_theta_m_r, float
                     args_battery, signed char *ret_pwm_l, signed char *ret_pwm_r)
{
    float tmp_theta;
    float tmp_theta_lpf;
    float tmp_pwm_r_limiter;
    float tmp_psidot;
    float tmp_pwm_turn;
    float tmp_pwm_l_limiter;
    float tmp_thetadot_cmd_lpf;
    float tmp[4];
    float tmp_theta_0[4];
    long tmp_0;

    tmp_thetadot_cmd_lpf = (((args_cmd_forward / CMD_MAX) * K_THETADOT) * (1.0F - A_R)) + (A_R * ud_thetadot_cmd_lpf);
    tmp_theta = (((DEG2RAD * args_theta_m_l) + ud_psi) + ((DEG2RAD * args_theta_m_r) + ud_psi)) * 0.5F;
    tmp_theta_lpf = ((1.0F - A_D) * tmp_theta) + (A_D * ud_theta_lpf);
    tmp_psidot = (args_gyro - args_gyro_offset) * DEG2RAD;

    tmp[0] = ud_theta_ref;
    tmp[1] = 0.0F;
    tmp[2] = tmp_thetadot_cmd_lpf;
    tmp[3] = 0.0F;
    tmp_theta_0[0] = tmp_theta;
    tmp_theta_0[1] = ud_psi;
    tmp_theta_0[2] = (tmp_theta_lpf - ud_theta_lpf) / EXEC_PERIOD;
    tmp_theta_0[3] = tmp_psidot;
    tmp_pwm_r_limiter = 0.0F;
    for (tmp_0 = 0; tmp_0 < 4; tmp_0++) {
      tmp_pwm_r_limiter += (tmp[tmp_0] - tmp_theta_0[tmp_0]) * K_F[(tmp_0)];
    }

    tmp_pwm_r_limiter = (((K_I * ud_err_theta) + tmp_pwm_r_limiter) / ((BATTERY_GAIN * args_battery) - BATTERY_OFFSET)) * 100.0F;
    tmp_pwm_turn = (args_cmd_turn / CMD_MAX) * K_PHIDOT;

    tmp_pwm_l_limiter = tmp_pwm_r_limiter + tmp_pwm_turn;
    tmp_pwm_l_limiter = rt_SATURATE(tmp_pwm_l_limiter, -100.0F, 100.0F);
    (*ret_pwm_l) = (signed char)tmp_pwm_l_limiter;

    tmp_pwm_r_limiter -= tmp_pwm_turn;
    tmp_pwm_r_limiter = rt_SATURATE(tmp_pwm_r_limiter, -100.0F, 100.0F);
    (*ret_pwm_r) = (signed char)tmp_pwm_r_limiter;

    tmp_pwm_l_limiter = (EXEC_PERIOD * tmp_thetadot_cmd_lpf) + ud_theta_ref;
    tmp_pwm_turn = (EXEC_PERIOD * tmp_psidot) + ud_psi;
    tmp_pwm_r_limiter = ((ud_theta_ref - tmp_theta) * EXEC_PERIOD) + ud_err_theta;

    ud_err_theta = tmp_pwm_r_limiter;
    ud_theta_ref = tmp_pwm_l_limiter;
    ud_thetadot_cmd_lpf = tmp_thetadot_cmd_lpf;
    ud_psi = tmp_pwm_turn;
    ud_theta_lpf = tmp_theta_lpf;
}

void balance_init(void){
  ud_err_theta = 0.0F;
  ud_theta_ref = 0.0F;
  ud_thetadot_cmd_lpf = 0.0F;
  ud_psi = 0.0F;
  ud_theta_lpf = 0.0F;
}