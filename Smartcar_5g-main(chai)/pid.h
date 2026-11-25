/*
 * @Descripttion:
 * @Author: LHK
 * @Date: 2023-10-28 18:20:19
 * @LastEditors: LHK
 * @LastEditTime: 2023-10-29 10:11:17
 */
#ifndef PID_H
#define PID_H

namespace PID
{
    class Position
    {
    private:
        float kp;       // 比例系数
        float ki;       // 积分系数
        float kd;       // 微分系数
        float target;   // 目标值
        float actual;   // 实际值
        float e;        // 误差
        float e_pre;    // 上一次误差
        float integral; // 积分项
    public:
        Position();
        ~Position();
        Position(float p, float i, float d);
        float pid_control(float tar, float act, float max, float min); // 执行PID控制
        void pid_show();                                               // 显示PID控制器的内部参数
    };
}

#endif