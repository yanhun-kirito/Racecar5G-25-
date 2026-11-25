/*
 * @Descripttion:
 * @Author: LHK
 * @Date: 2023-10-26 16:56:10
 * @LastEditors: LHK
 * @LastEditTime: 2023-11-07 14:28:03
 */
#include <pigpio.h>
#include <unistd.h>
#include <cstdlib>
#include <iostream>

// 舵机角度转化
float angleToDutyCycle(float angle)
{
    return 25 + (angle / 180.0) * 100;
}

void Init()
{
    // 停止网络服务
    std::system("sudo systemctl stop network-rc.service");
    std::system("sudo killall pigpiod");
    std::system("sudo pigpiod");
    usleep(1000);

    // 电机初始化
    int pwm_pin = 13;
    gpioInitialise();
    gpioSetMode(pwm_pin, PI_OUTPUT);
    gpioSetPWMfrequency(pwm_pin, 200);
    gpioSetPWMrange(pwm_pin, 40000);
    gpioPWM(pwm_pin, 12000);
    usleep(1000);

    // 拉高此脚，速度快
    int pwm_pin_high = 17;
    gpioInitialise();
    gpioSetMode(pwm_pin_high, PI_OUTPUT);
    gpioWrite(pwm_pin_high, 1);
    usleep(1000);
    std::cout << "Motor initialization successful" << std::endl;

    // 转向舵机初始化
    int servo_pin = 12;
    gpioInitialise();
    gpioSetPWMfrequency(servo_pin, 50);
    gpioSetPWMrange(servo_pin, 1000);
    gpioPWM(servo_pin, angleToDutyCycle(98));
    std::cout << "steering engine initialization successful" << std::endl;
    usleep(1000);

    // 蜂鸣器初始化
    int buzzer_pin = 21;
    gpioInitialise();
    gpioSetMode(buzzer_pin, PI_OUTPUT);
    gpioWrite(buzzer_pin, 1);
    usleep(500000);
    gpioWrite(buzzer_pin, 0);
    std::cout << "Buzzer initialization successful" << std::endl;
    usleep(1000);
}

void buzzer_on(void)
{
    gpioWrite(21, 1);
}
void buzzer_off(void)
{
    gpioWrite(21, 0);
}
/*int main()
{
    if (gpioInitialise() < 0)
    {
        // 初始化失败，处理错误
        return -1;
    }
    Init();
    while (1)
    {
        gpioPWM(12, angleToDutyCycle(60));
        usleep(1000000);
        gpioPWM(12, angleToDutyCycle(120));
        usleep(1000000);
    }
    gpioTerminate();
    return 0;
}*/
