/*
 * @Descripttion:
 * @Author: LHK
 * @Date: 2023-10-28 17:37:24
 * @LastEditors: LHK
 * @LastEditTime: 2023-10-29 10:07:48
 */
#include <iostream>
#include "pid.h"

// 位置PID
namespace PID
{

    Position::Position()
    {
        kp = 0.0;
        ki = 0.0;
        kd = 0.0;
        target = 0.0;
        actual = 0.0;
        e = 0.0;
        e_pre = 0.0;
        integral = 0.0;
    }

    Position::~Position() {}

    Position::Position(float p, float i, float d)
    {
        kp = p;
        ki = i;
        kd = d;
        target = 0.0;
        actual = 0.0;
        e = 0.0;
        e_pre = 0.0;
        integral = 0.0;
    }
    // 执行PID控制
    float Position::pid_control(float tar, float act, float MAX_OUTPUT, float MIN_OUTPUT)
    {
        float u;
        target = tar;
        actual = act;
        e = target - actual;
        integral += e;
        u = kp * e + ki * integral + kd * (e - e_pre);

        // 对控制器的输出进行限幅
        if (u > MAX_OUTPUT)
        {
            u = MAX_OUTPUT;
        }
        else if (u < MIN_OUTPUT)
        {
            u = MIN_OUTPUT;
        }
        e_pre = e;
        return u;
    }

    void Position::pid_show()
    {
        std::cout << "The information of this position PID controller is as follows:" << std::endl;
        std::cout << "       Kp=" << kp << std::endl;
        std::cout << "       Ki=" << ki << std::endl;
        std::cout << "       Kd=" << kd << std::endl;
        std::cout << " integral=" << integral << std::endl;
        std::cout << "   target=" << target << std::endl;
        std::cout << "   actual=" << actual << std::endl;
        std::cout << "        e=" << e << std::endl;
        std::cout << "    e_pre=" << e_pre << std::endl;
    }
}