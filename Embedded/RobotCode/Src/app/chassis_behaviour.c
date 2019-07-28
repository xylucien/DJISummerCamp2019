/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       chassis_behaviour.c/h
  * @brief      完成底盘行为任务。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#include "chassis_behaviour.h"
#include "chassis_task.h"
#include "arm_math.h"

#include "gimbal_behaviour.h"




extern void chassis_normal_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
extern void chassis_auto_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
  * @author         RM
  * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
  * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
  * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
  * @author         RM
  * @param[in]      vx_set前进的速度
  * @param[in]      vy_set左右的速度
  * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);



//底盘行为状态机
static chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;

void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

    //遥控器设置行为模式
    if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_AUTO;
    }
    else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_NO_MOVE;
    }
    else if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_NORMAL_MODE;
    }


    //根据行为状态机选择底盘状态机
    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {

        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW; //当行为是底盘无力，则设置底盘状态机为 raw，原生状态机。
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {

        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_SPEED; //当行为是底盘不移动，则设置底盘状态机为 底盘不跟随角度 状态机。
    }
    else if (chassis_behaviour_mode == CHASSIS_AUTO)
    {

        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_SPEED; //当行为是底盘不跟随角度，则设置底盘状态机为 底盘不跟随角度 状态机。
    }
    else if (chassis_behaviour_mode == CHASSIS_NORMAL_MODE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_SPEED; //当行为是底盘开环，则设置底盘状态机为 底盘原生raw 状态机。
    }
}
void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{

    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_zero_force_control(vx_set, vy_set, wz_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
        chassis_no_move_control(vx_set, vy_set, wz_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_NORMAL_MODE)
    {
        chassis_normal_control(vx_set, vy_set, wz_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_AUTO)
    {
        chassis_auto_control(vx_set, vy_set, wz_set, chassis_move_rc_to_vector);
    }

}
/**
  * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
  * @author         RM
  * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
  * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
  * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_can_set = 0.0f;
    *vy_can_set = 0.0f;
    *wz_can_set = 0.0f;
}
/**
  * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
  * @author         RM
  * @param[in]      vx_set前进的速度
  * @param[in]      vy_set左右的速度
  * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}
