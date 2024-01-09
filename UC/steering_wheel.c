//
// Created by tony on 23-10-24.
//

#include "steering_wheel.h"
#include "arm_math.h"
#include "steering_wheel_chassis.h"
#include <math.h>
#include <stdint.h>
//#define Delay_ms(x) HAL_Delay(x)

void _Swheel_get_rotationSpeed(steering_wheel_t *this);
void _Swheel_get_rotationPos(steering_wheel_t *this);
void _Swheel_rMotor_speedServo(steering_wheel_t *this, const float speed);
void _Swheel_rMotor_posServo(steering_wheel_t *this, const float pos);
uint8_t Swheel_aim(steering_wheel_t *this);
#ifdef USE_DEFAULT_MOTOR_PARAM
#include "DJI.h"
#include "Caculate.h"
#include "wtr_vesc.h"

//默认电机api (转向电机为M2006)
/**
 * @brief 内部函数，转向速度获取
 * @return 轮子转向速度，单位rpm
 */
void _Swheel_get_rotationSpeed(steering_wheel_t *this)
{
    this->rotation_speed = hDJI[this->id].FdbData.rpm/hDJI[this->id].reductionRate/STEERING_WHEEL_ROTATIOM_REDUCTION;
}
/**
 * @brief 内部函数，转向位置获取
 * @return 转过的总弧度，单位rad
 * */
void _Swheel_get_rotationPos(steering_wheel_t *this)
{
    this->rotation_pos= hDJI[this->id].Calculate.RotorAngle_all/hDJI[this->id].reductionRate*PI/180.f/STEERING_WHEEL_ROTATIOM_REDUCTION;
    // = fmod(range,2*M_PI);
}
/**
 * @brief 内部函数，转向速度设置
 * @param speed 轮子转向速度，单位rpm
 */
void _Swheel_rMotor_speedServo(steering_wheel_t *this, const float speed)
{
    speedServo(speed*hDJI[this->id].reductionRate*STEERING_WHEEL_ROTATIOM_REDUCTION,&hDJI[this->id]);
}
/**
 * @brief 内部函数，转向位置设置
 * @param pos 轮子转过的总弧度，单位rad
 */
void _Swheel_rMotor_posServo(steering_wheel_t *this, const float pos)
{
    positionServo(pos*180.f/PI*STEERING_WHEEL_ROTATIOM_REDUCTION,&hDJI[this->id]);
}
/**
 * @brief 转向电机复位
 */
void steeringWheel_rMotor_reset(steering_wheel_t *this)
{
    hDJI[this->id].Calculate.RotorAngle_all = 0;
    hDJI[this->id].Calculate.RotorAngle_0_360_Log[0] = 0;
    hDJI[this->id].Calculate.RotorAngle_0_360_Log[1] = 0;
    hDJI[this->id].Calculate.RotorRound = 0;
    hDJI[this->id].Calculate.RotorAngle_0_360_OffSet = hDJI[this->id].FdbData.RotorAngle_0_360;
}
void _Swheel_mMotor_speedServo(steering_wheel_t *this, const float speed)
{
    float erpm = 840*speed/STEERING_WHEEL_DIAMETER/PI;//n(rpm)=60f=60v/(pi*D) 840=14*60
    VESC_CAN_SET_ERPM(&hVESC[this->id],erpm);
}
#endif



/**
 * @brief 舵轮初始化
 * @param id
 * @param LS_GPIOx 光电开关GPIO
 * @param LS_GPIO_Pin 光电开关引脚
 * @return
 */
void steeringWheel_init(steering_wheel_t *this, const uint8_t id, GPIO_TypeDef* LS_GPIOx, const uint16_t LS_GPIO_Pin)
{
    this->id = id;
    this->_LS_GPIOx = LS_GPIOx;
    this->_LS_GPIO_Pin = LS_GPIO_Pin;
    this->_light_switch_flag = 0;
    this->direction = 0;
    //this->direction_offset = 0;
    //this->main_speed = 0;
    this->target_main_speed =0;
    this->rotation_speed = 0;
    //this->main_pos = 0;
    this->rotation_pos = 0;
    this->correcting_stage=0;
    this->state = STOP;
}
/**
 * @brief 转向轮启动校准
 * @param this
 */
void steeringWheel_startCorrect(steering_wheel_t *this)
{
    this->state = CORRECTING;
}
/**
 * @brief 转向轮限位开关外部中断回调函数
 * @param this
 * @param GPIO_Pin
 */
void steeringWheel_EXTI_Callback(steering_wheel_t *this, uint16_t GPIO_Pin)
{
    if(GPIO_Pin==this->_LS_GPIO_Pin)
        this->_light_switch_flag= 1;
}
/**
 * @brief 转向轮校准
 * @param this
 * @description
 * 1.转向轮转动到限位开关处 (stage=0)
 * 2.转向轮反向转动到限位开关处 (stage = 1~1000,如果限位开关触发则stage=-1,若一直不触发直到stage=1000则stage=0)
 * 3.转向轮停止,等待（stage = -1->-10）
 * 4.电机复位（stage = -10）
 */
static inline uint8_t _Swheel_correcting(steering_wheel_t *this)
{
    if(this->correcting_stage==0)
    {
        if(this->_light_switch_flag)
        {
            _Swheel_rMotor_speedServo(this, -2);
            this->_light_switch_flag = 0;
            this->correcting_stage=1;
        }
        else
        {
            _Swheel_rMotor_speedServo(this, 20);
        }
        return 0;
    }
    else if(this->correcting_stage>0&&this->correcting_stage<1000)
    {
        if(this->_light_switch_flag)
        {
            _Swheel_rMotor_speedServo(this, 0);
            this->_light_switch_flag = 0;
            this->correcting_stage=-1;
            return 0;
        }
        else
        {   this->correcting_stage++;
            _Swheel_rMotor_speedServo(this, -2);
            return 0;
        }
    }
    else if(this->correcting_stage>=1000){this->correcting_stage = 0;return 0;}//重头再来
    else if(this->correcting_stage<0&&this->correcting_stage>-100)
    {
        this->correcting_stage--;
        _Swheel_rMotor_speedServo(this, 0);
        return 0;
    }//延时
    else if(this->correcting_stage<=-100)
    {
        steeringWheel_rMotor_reset(this);
        this->correcting_stage=0;
        return 1;
    }

}
/**
 * @brief 舵轮执行器函数
 * @param this
 * @description
 * 1.如果状态为STOP，则电机停止
 * 2.如果状态为CORRECTING，则执行校准
 * 3.如果状态为RUNNING，则执行转向
 */
void steeringWheel_executor(steering_wheel_t *this)
{
    _Swheel_get_rotationPos(this);
    this->direction=fmod(this->rotation_pos,2*PI);//+this->direction_offset;
    if(this->direction>PI)
        this->direction-=2*PI;
    
    _Swheel_get_rotationSpeed(this);
    if(this->state==CORRECTING)
    {
        if(_Swheel_correcting(this))
            this->state = STOP;
        return;
    }
    if(this->state==STOP)
    {
        _Swheel_rMotor_speedServo(this, 0);
        _Swheel_mMotor_speedServo(this, 0);
        return;
    }
    if(this->state==RUNNING)
    {
       //_Swheel_rMotor_posServo(this, this->target_direction +
        //                                2 * M_PI * (int) ((this->rotation_pos - this->direction_offset) / (2 * M_PI)));
        if(Swheel_aim(this))
        {
            this->state=AIMMING;
            return;
        }
        _Swheel_mMotor_speedServo(this, this->target_main_speed);
        return;
    }
    if(this->state==AIMMING) 
    {
        if(!Swheel_aim(this))
        {
            this->state=STOP;
            return;
        }
        //_Swheel_rMotor_posServo(this, this->target_direction +2 * M_PI *(int) ((this->rotation_pos - this->direction_offset)/ (2 * M_PI)));
        return;
    }

}



uint8_t Swheel_aim(steering_wheel_t *this)
{
    float tgt_d[2];
    float d[2];
    float d_err;
    //int8_t is_invert;
    d[0] = this->direction;
    d[1] = d>=0 ? d[0]-PI:d[1]+PI;
    tgt_d[0] = this->target_direction;
    tgt_d[1] = tgt_d[0]>0 ? tgt_d[0]-2*PI:tgt_d[0]+2*PI;
    for(int8_t i=0;i<=1;i++)
    {
            d_err = tgt_d[i]-d[0];
            if(fabsf(d_err)<PI/2)
                break;
            d_err = tgt_d[i]-d[1];
            if(fabsf(d_err)<PI/2)
            {
                this->target_main_speed*=-1;
                break;
            }   
    }
    

    // if (fabsf(tgt_d1-d)<=PI/2) {
    //     d_err = tgt_d1-d;
    //     //this->target_main_speed*=1;
    // }
    // else if (fabsf(tgt_d2-d)<=PI/2) {
    //     d_err = tgt_d1-d;
    // }
    // else {
    //     d_err = this->target_direction-invert_d;        
    //     this->target_main_speed*=-1;
    // }
    _Swheel_rMotor_posServo(this, d_err+this->rotation_pos);
    if(fabsf(d_err)<0.1)
        return 0;
    else
        return 1;



}