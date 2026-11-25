//======================================================================================================================================================================================#include <iostream>
//======================================================================================================================================================================================#include <iostream>
// 系统库头文件
#include <cstdlib>       // C标准库函数
#include <unistd.h>      // Unix标准函数（如sleep等）
#include <iostream>      // 输入输出流
#include <stdlib.h>      // 标准库函数（与cstdlib部分重叠）
#include <string>        // 字符串处理
#include <vector>        // 向量容器
#include <chrono>        // 时间相关功能
#include <csignal>       // 信号处理
#include <atomic>        // 原子操作

// OpenCV库头文件（注意：有些头文件可能重复包含）
#include <opencv2/opencv.hpp>               // OpenCV主要头文件
#include <opencv2/core/core.hpp>    // OpenCV核心功能
#include <opencv2/highgui.hpp>      // 高层GUI功能
#include <opencv2/imgproc/imgproc_c.h>  // 图像处理C风格函数

// pigpio库头文件，用于GPIO控制（树莓派等平台）
#include <pigpio.h>

// 多线程库
#include <thread>

//时间相关
#include <ctime>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <errno.h>
#include <cstring>

#include <termios.h> // 必须添加，用于终端控制
#include <unistd.h> // 必须添加，用于 read 函数
#include <fcntl.h> // 必须添加，用于文件控制

// 宏定义：打印程序用法信息
#define Usage()                                            \
    {                                                      \
        std::cerr << "usage: ./showpic FILE" << std::endl; \
    }

// 使用命名空间，简化代码编写
using namespace std;
using namespace cv;

// 全局变量定义

// OpenCV图像矩阵变量
cv::Mat openedImage;   // 已打开的图像
Mat bin_image2;        // 备用二值化图像
Mat frame;             // 当前摄像头帧
Mat img;               // 通用图像变量
Mat bin_image;         // 主要二值化图像
Mat change_frame;      // 转换后的图像（如HSV色彩空间）
Mat mask;              // 掩码图像
Mat final_img;         // 最终输出图像
Mat white_mask;        // 白色区域掩码
Mat hsv_image;         // HSV色彩空间图像
Mat delite_frame;      // 删除后的帧（可能用于中间处理）
Mat binaryImage;       // 二值化图像
Mat grayImage;         // 灰度图像
Mat binaryImage_1;     // 备用二值化图像

// 轮廓相关变量
vector<Point> contours;           // 单个轮廓点集
vector<vector<Point>> contours_all;  // 所有检测到的轮廓

// 计数和状态标志变量
int number1 = 0;          // 通用计数器1
int number2 = 0;          // 通用计数器2
int heighest = 100;       // 最高点位置（可能用于路径检测）
int count_change = 0;     // 变道计数
//int begin_sign = 1;       // 开始标志（1表示开始，0表示未开始）
int find_first = 0;       // 是否找到第一个目标（如蓝色卡片）的标志
int flag_yellow_finish = 0;  // 黄色目标处理完成标志
int flag_cross_finish = 0;   // 斑马线处理完成标志（0=未处理，1=处理中，2=处理完成）
int cross=0;                // 斑马线检测结果（0=未检测到，1=检测到）
int zebra_y = -1;          // 斑马线检测到的最高（最靠上的）Y坐标，-1表示未检测到
int zebra_y_center = -1;   // 可选：斑马线区域的中心Y坐标
int now_blue_max;         // 当前最大蓝色区域相关
int flag_cross = 0;       // 斑马线标志
int line_num = 0;         // 线条数量计数
int bluearea_min = 50;    // 蓝色区域最小面积阈值
int bluearea_max = 1000;  // 蓝色区域最大面积阈值
int count_count2 = 0;     // 嵌套计数器
int try_patching_line;    // 尝试修补线条的标志
int yellow_flag = 0;      // 黄色目标标志
int sigmaCenter;          // 中心点相关参数
int last_blue = 0;        // 上一次蓝色目标位置
int blue_ver_mid;         // 蓝色垂直中线位置
int times4 = 0;           // 计数器
int biandao = 0;          // 变道标志

// 电机和舵机控制参数
int speedlow = 11500;     // 低速PWM值
int speedhigh = 12000;    // 高速PWM值
int motorSpeed;           // 当前电机速度
int counterShift = 0;     // 移位计数器
int servo_pin = 12;       // 舵机GPIO引脚号
float servo_pwm_range = 10000;       // 舵机PWM范围
float servo_pwm_frequency = 50;      // 舵机PWM频率（Hz）
float servo_pwm_duty_cycle_unlock = 864;  // 756舵机解锁占空比
int motor_pin = 13;       // 电机GPIO引脚号
float motor_pwm_range = 40000.0;     // 电机PWM范围
float motor_pwm_frequency = 200.0;   // 电机PWM频率（Hz）
float motor_pwm_duty_cycle_unlock = 10500.0;  // 电机解锁占空比
float kp = 1.0;           // PID控制比例系数
float kd = 2.0;           // PID控制微分系数
float servo_pwm_now;      // 当前舵机PWM值
float servo_pwm_yanzhen;  // 舵机PWM验证值
int error_first = 0;      // 初始误差
int last_error = 0;       // 上一次误差
float servo_pwm_diff;     // 舵机PWM差值
float servo_pwm;          // 目标舵机PWM值
int speed_all = 9350;    //全局行驶车速（调车速直接修改）
int Servo_middle = 858;  // 858为中间位置 

enum State
{
    READY,//准备状态，等待发车
    START,//发车，正常循迹并寻找锥桶
    AVOID_CONE,//避开锥桶
    FIND_CROSS,//寻找十字路口
    CROSS_STOP,//十字路口停车
    LANE_CHANGE_1,//变道1
    FIND_CHANGE_LINE,//寻找变道线
    LANE_CHANGE_2,//变道2
    FIND_PARK_SIGN,//寻找停车标志
    PARKING,//停车
    END,//任务结束
    DEBUG//调试状态
};

State Running_State = READY;//当前运行状态，初始为准备状态
float center_x=0;  // 存储锥桶轮廓包围圆圆心的x坐标
float center_y=0;  // 存储锥桶轮廓包围圆圆心的y坐标
vector<vector<Point>> blue_contours;  // 普通变量，用于接收锥桶识别返回值
std::chrono::high_resolution_clock::time_point TimeRecord_Start_1;
std::chrono::high_resolution_clock::time_point TimeRecord_End_1;
std::chrono::high_resolution_clock::time_point start_time;
std::chrono::high_resolution_clock::time_point end_time;
int blue_check = 0; // 确认蓝色锥桶计数器
int blue_disapear = 0;//检测锥桶消失计数器
int Keep_Avoid = 0; // 保持避开锥桶状态
int cross_check = 0; // 确认十字路口计数器
int Left_check = 0; // 确认左变道箭头计数器
int Right_check = 0; // 确认右变道箭头计数器
int change_direction = -1; // 变道方向，-1=左转，1=右转

std::chrono::high_resolution_clock::time_point last_capture_time;//上一次抓拍时间
bool first_capture = true;      //启用第一次抓拍
const int CAPTURE_INTERVAL = 30; // 抓拍30秒间隔


// 路径点集（用于存储检测到的路径边界点）
std::vector<cv::Point> left_line;     // 左边界点集
std::vector<cv::Point> right_line;    // 右边界点集
std::vector<cv::Point> mid;           // 中间路径点集
std::vector<cv::Point> left_line_blue;    // 蓝色路径左边界点集
std::vector<cv::Point> right_line_blue;   // 蓝色路径右边界点集
std::vector<cv::Point> mid_blue;          // 蓝色路径中间点集

// 变道线参数数组
int patching_line1[2] = {1, 0};  // 修补线1参数
int patching_line2[2] = {0, 1};  // 修补线2参数

// 点位置变量
Point end_point;               // 终点
Point vertebral_barrel_Point;  // 锥桶位置

// 时间相关变量
long long total_duration = 0;  // 总运行时间（毫秒）
int crosstime = 0;             // 十字路口处理时间阈值
std::string save_dir = "/home/pi/5Gcode4/captured_frames"; // 保存目录

// 确保在全局变量区域定义了 park_place 
int park_place = 0; 
int biandao2 = 0; //用于 FIND_CHANGE_LINE 状态

/**
 * 在二值图像上绘制一条指定宽度的白色直线
 * 
 * @param binaryImage 输入的二值图像（将在其副本上进行绘制）
 * @param start 直线的起点坐标
 * @param end 直线的终点坐标
 * @param lineWidth 直线的宽度（像素）
 * @return 返回绘制了白色直线后的图像副本
 */
cv::Mat drawWhiteLine(const cv::Mat &binaryImage, cv::Point start, cv::Point end, int lineWidth)
{
    // 创建输入图像的副本，避免直接修改原图
    cv::Mat resultImage = binaryImage.clone();

    // 提取起点和终点的坐标
    int x1 = start.x, y1 = start.y;
    int x2 = end.x, y2 = end.y;

    // 处理垂直线的情况（x坐标相同）
    if (x1 == x2)
    {
        // 从上到下遍历垂直线上的所有点
        for (int y = std::min(y1, y2); y <= std::max(y1, y2); ++y)
        {
            // 为每个点扩展指定宽度，确保线条粗细
            for (int i = -lineWidth / 2; i <= lineWidth / 2; ++i)
            {
                // 将对应位置的像素设置为白色（255）
                resultImage.at<uchar>(cv::Point(x1 + i, y)) = 255;
            }
        }
    }
    // 处理非垂直线的情况
    else
    {
        // 计算直线的斜率和截距
        double slope = static_cast<double>(y2 - y1) / (x2 - x1);
        double intercept = y1 - slope * x1;

        // 从左到右遍历直线上的所有点
        for (int x = std::min(x1, x2); x <= std::max(x1, x2); ++x)
        {
            // 使用直线方程计算对应的y坐标
            int y = static_cast<int>(slope * x + intercept);
            // 为每个点扩展指定宽度，确保线条粗细
            for (int i = -lineWidth / 2; i <= lineWidth / 2; ++i)
            {
                // 确保y坐标在图像范围内，防止越界访问
                int newY = std::max(0, std::min(y + i, resultImage.rows - 1));
                // 将对应位置的像素设置为白色（255）
                resultImage.at<uchar>(cv::Point(x, newY)) = 255;
            }
        }
    }

    // 返回绘制了白色直线后的图像
    return resultImage;
}
/**
 * 初始化舵机和电机的PWM控制
 * 
 * 该函数负责初始化GPIO系统，并配置舵机和电机的PWM参数，
 * 包括设置引脚模式、PWM频率、PWM范围和初始占空比。
 * 初始化成功后，舵机和电机将处于解锁状态，准备接收控制信号。
 */
void servo_motor_pwmInit(void)
{
    // 初始化GPIO系统
    if (gpioInitialise() < 0)
    {
        // 初始化失败时输出错误信息并返回
        std::cout << "GPIO failed" << std::endl;
        return;
    }
    else
        // 初始化成功时输出成功信息
        std::cout << "GPIO ok" << std::endl;
    
    // 配置舵机引脚
    gpioSetMode(servo_pin, PI_OUTPUT);               // 设置舵机引脚为输出模式
    gpioSetPWMfrequency(servo_pin, servo_pwm_frequency); // 设置舵机PWM频率
    gpioSetPWMrange(servo_pin, servo_pwm_range);      // 设置舵机PWM范围
    gpioPWM(servo_pin, servo_pwm_duty_cycle_unlock);  // 设置舵机初始占空比（解锁状态）

    // 配置电机引脚
    gpioSetMode(motor_pin, PI_OUTPUT);               // 设置电机引脚为输出模式
    gpioSetPWMfrequency(motor_pin, motor_pwm_frequency); // 设置电机PWM频率
    gpioSetPWMrange(motor_pin, motor_pwm_range);      // 设置电机PWM范围
    gpioPWM(motor_pin, motor_pwm_duty_cycle_unlock);  // 设置电机初始占空比（解锁状态）

//	// 配置云台Y舵机引脚
//    gpioSetMode(22, PI_OUTPUT);               // 设置舵机引脚为输出模式
//    gpioSetPWMfrequency(22, servo_pwm_frequency); // 设置舵机PWM频率
//    gpioSetPWMrange(22, servo_pwm_range);      // 设置舵机PWM范围
//    gpioPWM(22, 285);  // 设置舵机初始占空比（解锁状态）
//
//    // 配置云台X舵机引脚
//    gpioSetMode(23, PI_OUTPUT);               // 设置舵机引脚为输出模式
//    gpioSetPWMfrequency(23, servo_pwm_frequency); // 设置舵机PWM频率
//    gpioSetPWMrange(23, servo_pwm_range);      // 设置舵机PWM范围
//    gpioPWM(23, 333);  // 设置舵机初始占空比（解锁状态）
    
}

/**
 * 对输入图像进行去畸变处理
 * 
 * 该函数使用预计算的相机内参和畸变系数，对输入图像进行去畸变操作，
 * 消除由于相机镜头光学特性导致的图像畸变，使图像中的直线在处理后保持直线。
 * 
 * @param frame 输入的原始图像帧（可能包含畸变）
 * @return 返回去畸变后的图像帧
 */
cv::Mat undistort(const cv::Mat &frame)
{
    // 定义相机畸变系数
    double k1 = -0.340032906324299;  // 径向畸变系数1
    double k2 = 0.101344757327394;   // 径向畸变系数2
    double p1 = 0.0;                 // 切向畸变系数1
    double p2 = 0.0;                 // 切向畸变系数2
    double k3 = 0.0;                 // 径向畸变系数3（此处未使用）

    // 定义相机内参矩阵K（3x3矩阵）
    // 包含焦距(fx, fy)和光心( cx, cy )等相机内部参数
    cv::Mat K = (cv::Mat_<double>(3, 3) << 162.063205442089, 0.0, 154.707845362265,
                 0.0, 162.326264903804, 129.914361509615,
                 0.0, 0.0, 1.0);

    // 创建畸变系数矩阵D
    cv::Mat D = (cv::Mat_<double>(1, 5) << k1, k2, p1, p2, k3);
    
    // 声明用于存储映射表的矩阵
    cv::Mat mapx, mapy;
    // 声明用于存储去畸变后图像的矩阵
    cv::Mat undistortedFrame;

    // 计算从原始图像到去畸变图像的映射关系
    // 参数：相机内参矩阵、畸变系数矩阵、空矩阵、新相机内参矩阵、输出图像大小、映射表数据类型
    cv::initUndistortRectifyMap(K, D, cv::Mat(), K, frame.size(), CV_32FC1, mapx, mapy);
    
    // 应用映射表，执行去畸变操作
    // 使用线性插值方法(INTER_LINEAR)提高图像质量
    cv::remap(frame, undistortedFrame, mapx, mapy, cv::INTER_LINEAR);

    // 返回去畸变后的图像
    return undistortedFrame;
}
/**
 * 自定义直方图均衡化函数，允许控制对比度增强的程度
 * 
 * 该函数先对输入图像应用标准的直方图均衡化，然后通过线性插值
 * 在原始图像和均衡化后的图像之间进行混合，以减弱可能过于强烈的对比度增强效果。
 * 适用于需要适度增强图像对比度，但又不希望细节丢失的场景。
 * 
 * @param inputImage 输入的灰度图像（单通道）
 * @param alpha 混合系数，范围[0,1]，控制均衡化效果的强度
 *              alpha=1.0: 完全使用直方图均衡化后的图像
 *              alpha=0.0: 完全保留原始图像
 *              0<alpha<1: 混合两种图像，减弱均衡化效果
 * @return 返回对比度增强后的图像
 */
Mat customEqualizeHist(const Mat &inputImage, float alpha)
{
    Mat enhancedImage;
    // 应用标准的直方图均衡化，增强图像对比度
    equalizeHist(inputImage, enhancedImage);

    // 通过线性插值混合原始图像和均衡化后的图像，减弱对比度增强的效果
    // 公式：输出图像 = alpha*均衡化图像 + (1-alpha)*原始图像
    return alpha * enhancedImage + (1 - alpha) * inputImage;
}
Mat ImagePreprocessing(const cv::Mat &frame_a)
{
    int width = 320;//图像宽度
    int height = 240;//图像高度

    binaryImage_1 = cv::Mat::zeros(height, width, CV_8U);//创建一个与输入图像大小相同的零矩阵，用于存储二值化后的图像
    //--------------------------------------------这一段代码中包含了canny算子和一些图像处理的方案--------------------------------------------------
    // cv::Mat grayImage;//灰度图像
    // cv::cvtColor(frame, grayImage, cv::COLOR_BGR2GRAY);
    // cv::Mat edgeImage;//边缘图像
    // cv::Canny(grayImage, edgeImage, 50, 150);//Canny算子检测边缘
    // vector<Vec4i> lines;//检测到的直线
    // HoughLinesP(edgeImage, lines, 1, CV_PI / 180, 180, 100, 10);

    // for (size_t i = 0; i < lines.size(); i++) {
    //     Vec4i l = lines[i];
    //     line(binaryImage, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 4);
    // }
    // // GaussianBlur(frame, img, cv::Size(7, 7), 0);
    // cvtColor(img, hsv_image, cv::COLOR_BGR2HSV);
    // // Scalar lower_white(10, 43, 46);
    // // Scalar upper_white(180, 255, 255);
    // // inRange(hsv_image, lower_white, upper_white, white_mask);
    // // bitwise_not(white_mask, final_img);
    // // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1));

    // // cv::morphologyEx(final_img, delite_frame, cv::MORPH_OPEN, kernel);
    // Mat img_edges;
    // Canny(frame, img_edges, 100, 150);
    // cv::Mat blurred;
    // cv::Mat edges;
    // cv::GaussianBlur(frame_a, blurred, cv::Size(5, 5), 0);
    // int roiYStart = 100;
    // int roiYEnd = 155;
    // // 创建感兴趣区域（ROI）
    // cv::Mat roiImage = blurred(cv::Range(roiYStart, roiYEnd), cv::Range(0, frame_a.cols));

    // // Canny边缘检测
    // cv::Canny(roiImage, edges, 100, 150);

    // // 创建全黑图像
    // cv::Mat blackImage = cv::Mat::zeros(frame_a.size(), CV_8UC1);

    // // 将Canny边缘叠加到全黑图像的感兴趣区域中
    // edges.copyTo(blackImage(cv::Range(roiYStart, roiYEnd), cv::Range(0, frame_a.cols)));
    // imshow("canny1",edges);
    // imshow("canny2",blackImage);
    //----------------------------------------------------------------------------------------------------------------
    //-------------------------------------------------赛道条件好的情况-------------------------------------------------
    // int kernelSize = 5;
    // double sigma = 1.0;
    // cv::Mat blurredImage;
    // cv::GaussianBlur(frame_a, blurredImage, cv::Size(kernelSize, kernelSize), sigma);
    // cv::Mat grad_x, grad_y;
    // cv::Mat sobel_x,sobel_y;
    // cv::Mat abs_grad_x, abs_grad_y;
    // int xWeight = 1;
    // int yWeight = 1;
    // cv::Sobel(blurredImage, grad_x, CV_16S, 1, 0, 3, xWeight);
    // cv::Sobel(blurredImage, grad_y, CV_16S, 0, 1, 3, yWeight);
    // cv::convertScaleAbs(grad_x, abs_grad_x);
    // cv::convertScaleAbs(grad_y, abs_grad_y);
    // cv::Mat edges;
    // cv::addWeighted(abs_grad_x, 0.5, abs_grad_y,0.5, 0, edges);
    // imshow("edges",edges);
    // cv::threshold(edges, binaryImage, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
    // imshow("binaryImage",binaryImage);
    //--------------------------------------------------赛道条件差的情况----------------------------------------------
    // Sobel边缘检测
    Mat originalImage;//原始图像
    cvtColor(frame, originalImage, cv::COLOR_BGR2GRAY);//将输入图像从BGR颜色空间转换为灰度图像
     //float alpha = 0.5;  // 调整这个值以控制对比度增强的强度
     //Mat enhancedImage = customEqualizeHist(originalImage, alpha);//对比度增强后的图像
    Mat enhancedImage = originalImage;//对比度增强后的图像
    // Sobel边缘检测
    Mat sobelx, sobely;//Sobel算子的x和y方向梯度
    Sobel(enhancedImage, sobelx, CV_64F, 1, 0, 3);//Sobel算子的x方向梯度
    Sobel(enhancedImage, sobely, CV_64F, 0, 1, 3);//Sobel算子的y方向梯度

    Mat gradientMagnitude = abs(sobelx) + abs(sobely);//梯度幅值图像
    convertScaleAbs(gradientMagnitude, gradientMagnitude);//将梯度幅值图像转换为8位无符号整数图像

    // 调整阈值
    Mat binaryImage12 = Mat::zeros(enhancedImage.size(), CV_8U);//二值化后的图像
    // threshold(gradientMagnitude, binaryImage12, 50, 255, THRESH_BINARY);
    cv::threshold(gradientMagnitude, binaryImage12, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);//Otsu阈值分割
    //-------------------------------------------------上新方案----------------------------------------------
    cv::Mat kernel1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));//膨胀核
    cv::dilate(binaryImage12, binaryImage, kernel1, cv::Point(-1, -1), 1); // 这个地方也要修改，膨胀操作
    // cv::dilate(binaryImage, binaryImage, kernel1, cv::Point(-1, -1), 1);//膨胀操作
    int x_roi = 1;//感兴趣区域的x坐标
    int y_roi = 120;//感兴趣区域的y坐标
    int width_roi = 318;;//感兴趣区域的宽度
    int height_roi = 60;//感兴趣区域的高度
    cv::Rect roi(x_roi, y_roi, width_roi, height_roi);//感兴趣区域的矩形框

    cv::Mat croppedObject = binaryImage(roi);//裁剪出感兴趣区域
    vector<Vec4i> lines;//检测到的直线
    HoughLinesP(croppedObject, lines, 1, CV_PI / 180, 25, 15, 10);//Hough变换检测直线
    for (size_t i = 0; i < lines.size(); i++)//遍历检测到的直线
    {
        Vec4i l = lines[i];//当前检测到的直线
        double angle = atan2(l[3] - l[1], l[2] - l[0]) * 180.0 / CV_PI;//计算直线的角度
        double length = sqrt(pow(l[3] - l[1], 2) + pow(l[2] - l[0], 2));//计算直线的长度
        double aspect_ratio = length / abs(l[3] - l[1]);//计算直线的纵横比
        if (abs(angle) > 15) // 过滤掉接近水平的线段
        {
            Vec4i l = lines[i];
            l[0] += x_roi;
            l[1] += y_roi;
            l[2] += x_roi;
            l[3] += y_roi;
            line(binaryImage_1, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 2, LINE_AA);
                static int frame_count = 0;
//    if (frame_count < 2) {
//        std::string filename = "frame_" + std::to_string(frame_count) + ".jpg";
//        cv::imwrite(filename,binaryImage_1);
//        frame_count++;
//    }
        }
    }
    return binaryImage_1;//返回二值化后的图像
}
//***************************************************************************************************************** */
//   // 步骤1：转为灰度图
//     Mat gray;
//     cvtColor(frame_a, gray, COLOR_BGR2GRAY);  // 修正：使用frame_a
    
//     // 步骤2：光照均衡化
//     Mat clahe_img;
//     Ptr<CLAHE> clahe = createCLAHE(2.0, Size(8, 8));
//     clahe->apply(gray, clahe_img);
    
//     // 步骤3：多方法融合
//     Mat binary1, binary2;
    
//     // 方法1：自适应阈值
//     adaptiveThreshold(clahe_img, binary1, 255, ADAPTIVE_THRESH_GAUSSIAN_C, 
//                      THRESH_BINARY, 15, 5);
    
//     // 方法2：Canny边缘检测 + 膨胀连接
//     Mat edges;
//     GaussianBlur(clahe_img, clahe_img, Size(3, 3), 0);
//     Canny(clahe_img, edges, 30, 90);
    
//     // 连接边缘断点
//     Mat kernel_dilate = getStructuringElement(MORPH_RECT, Size(3, 3));
//     dilate(edges, binary2, kernel_dilate);
    
//     // 步骤4：融合两种结果
//     Mat binaryResult;
//     bitwise_or(binary1, binary2, binaryResult);
    
//     // 步骤5：形态学优化
//     Mat kernel_close = getStructuringElement(MORPH_RECT, Size(5, 5));
//     morphologyEx(binaryResult, binaryResult, MORPH_CLOSE, kernel_close);
    
//     // 将结果复制到输出图像
//     binaryResult.copyTo(binaryImage_1);
    
//     return binaryImage_1;
// }
// --------------------------------------------------------新方案---------------------------------------------------


/**
 * 在二值化膨胀图像中跟踪左右车道线，并计算中心线
 * 
 * 该函数通过从图像底部向上逐行扫描的方式，寻找左右车道线的边界点，
 * 并计算每一行的中心点，从而生成完整的左右车道线和中心线轨迹。
 * 这是智能车视觉导航系统中的核心函数，用于确定车辆在道路上的位置和行驶方向。
 * 
 * @param dilated_image 输入的二值化膨胀图像，通常是经过预处理（二值化、膨胀）后的车道线图像
 *                      白色像素(255)表示车道线，黑色像素(0)表示背景
 */
void Tracking(const cv::Mat &dilated_image)
{
/***************************************************** 初版 *********************************************************************/
    // // 设置初始扫描起点（图像宽度的中间位置附近）
    // int begin = 160;
    
    // // 清空之前存储的车道线点集合
    // left_line.clear();   // 左车道线点集合
    // right_line.clear();  // 右车道线点集合
    // mid.clear();         // 中心线点集合
    
    // // 从图像底部向上扫描（从y=153到y=110）
    // for (int i = 150; i >= 120; i--)
    // {
    //     // 初始化左右车道线发现标志
    //     int find_l = 0;  // 左车道线是否找到
    //     int find_r = 0;  // 右车道线是否找到
        
    //     // 设置向左和向右搜索的起始位置
    //     int to_left = begin;
    //     int to_right = begin;

    //     // 向左搜索左车道线边界（从起始点向左移动）
    //     while (to_left != 1)  // 直到到达图像左边界
    //     {
    //         // 判断是否连续两个像素都是白色（即车道线）
    //         if (dilated_image.at<uchar>(i, to_left) == 255 && dilated_image.at<uchar>(i, to_left + 1) == 255)
    //         {
    //             find_l = 1;  // 找到左车道线
    //             left_line.push_back(cv::Point(to_left, i));  // 记录左车道线点
    //             break;
    //         }
    //         else
    //         {
    //             to_left--;  // 继续向左搜索
    //         }
    //     }

    //     // 如果未找到左车道线（到达左边界），则使用左边界作为左车道线点
    //     if (to_left == 1)
    //     {
    //         left_line.push_back(cv::Point(1, i));
    //     }

    //     // 向右搜索右车道线边界（从起始点向右移动）
    //     while (to_right != 318)  // 直到到达图像右边界
    //     {
    //         // 判断当前像素和左侧第二个像素是否都是白色（即车道线）
    //         if (dilated_image.at<uchar>(i, to_right) == 255 && dilated_image.at<uchar>(i, to_right - 2) == 255)
    //         {
    //             find_r = 1;  // 找到右车道线
    //             right_line.push_back(cv::Point(to_right, i));  // 记录右车道线点
    //             break;
    //         }
    //         else
    //         {
    //             to_right++;  // 继续向右搜索
    //         }
    //     }

    //     // 如果未找到右车道线（到达右边界），则使用右边界作为右车道线点
    //     if (to_right == 318)
    //     {
    //         right_line.push_back(cv::Point(318, i));
    //     }
        
    //     // 获取当前行的左右车道线点
    //     cv::Point midx1 = left_line.back();
    //     cv::Point midx2 = right_line.back();
        
    //     // 计算当前行的中心点并记录
    //     mid.push_back(cv::Point(int((midx1.x + midx2.x) / 2), i));
        
    //     // 更新下一行的扫描起始位置（当前行左右边界的中点）
    //     begin = (to_right + to_left) / 2;

    // }
/***************************************************** 第二版 *********************************************************************/
    // ================== 参数区 (根据实际跑道调整) ==================
    int search_start_row = 150; // 扫描起始行
    int search_end_row = 110;   // 扫描结束行
    int default_width = 145;    // 赛道大概的像素宽度 (320宽度的图，赛道约占200-260)
    int min_track_width = 100;  // 最小允许的赛道宽度 (防止把两条很近的噪点当赛道)
    int max_track_width = 310;  // 最大允许的赛道宽度
    // ============================================================

    left_line.clear();
    right_line.clear();
    mid.clear();

    int width = dilated_image.cols; // 320
    int center = width / 2;         // 160
    
    // 记录上一行的中点，用于下一行搜索的起点 (保持连续性)
    static int last_mid = width / 2; 
    int current_mid = last_mid;     // 当前行的中点

    // 记录当前的赛道宽度，用于单边补线 (动态更新)
    static int current_track_width = default_width; 

    // 从下往上扫描
    for (int i = search_start_row; i >= search_end_row; i--)
    {
        int left_point = -1;  // -1 表示未找到
        int right_point = -1; // -1 表示未找到

        // 1. 确定搜索起点 (基于上一行中点，防止突变)
        // 限制搜索起点不要太靠边，留出余量
        int search_center = std::max(20, std::min(current_mid, width - 20));

        // 2. 向左搜索
        for (int j = search_center; j > 2; j--)
        {
            // 连续两个白点，且再往左是黑点 (外黑内白边缘检测，更抗噪)
            // 或者沿用你原来的逻辑：连续两点白
            if (dilated_image.at<uchar>(i, j) == 255 && dilated_image.at<uchar>(i, j - 1) == 255)
            {
                left_point = j;
                break;
            }
        }

        // 3. 向右搜索
        for (int j = search_center; j < width - 2; j++)
        {
            if (dilated_image.at<uchar>(i, j) == 255 && dilated_image.at<uchar>(i, j + 1) == 255)
            {
                right_point = j;
                break;
            }
        }

        // 4. 决策逻辑 (核心优化部分)
        int final_left = 0;
        int final_right = width;
        int calculated_mid = 0;

        bool left_found = (left_point != -1);
        bool right_found = (right_point != -1);

        // 情况A: 两边都找到了
        if (left_found && right_found)
        {
            int dist = right_point - left_point;
            // 宽度合理性检查
            if (dist > min_track_width && dist < max_track_width)
            {
                final_left = left_point;
                final_right = right_point;
                calculated_mid = (final_left + final_right) / 2;
                
                // 更新记忆的赛道宽度 (加权平均，缓慢更新，防抖)
                current_track_width = int(current_track_width * 0.6 + dist * 0.4);
            }
            else
            {
                // 宽度异常（可能是斑马线干扰或横向噪点），沿用上一行中点
                calculated_mid = current_mid;
                final_left = calculated_mid - current_track_width / 2;
                final_right = calculated_mid + current_track_width / 2;
            }
        }
        // 情况B: 只找到左边 (右边丢线/出界)
        else if (left_found && !right_found)
        {
            final_left = left_point;
            // 用记忆的宽度补出右边
            final_right = left_point + current_track_width; 
            // 防止补出图像外太远
            if (final_right >= width) final_right = width - 1;
            
            calculated_mid = final_left + current_track_width / 2;
        }
        // 情况C: 只找到右边 (左边丢线/出界)
        else if (!left_found && right_found)
        {
            final_right = right_point;
            // 用记忆的宽度补出左边
            final_left = right_point - current_track_width;
            if (final_left < 0) final_left = 0;

            calculated_mid = final_right - current_track_width / 2;
        }
        // 情况D: 两边都没找到 (完全丢失，或者进入全白区域)
        else
        {
            // 极度危险情况，保持上一行的趋势或者直接沿用上一行
            calculated_mid = current_mid;
            final_left = calculated_mid - current_track_width / 2;
            final_right = calculated_mid + current_track_width / 2;
        }

        // 5. 防抖滤波 (限制单行突变幅度)
        // 如果当前行计算出的中点和上一行差距过大(例如 > 30像素)，限制变化幅度
        int diff = calculated_mid - current_mid;
        int max_diff = 20; // 允许的最大跳变
        if (abs(diff) > max_diff)
        {
            calculated_mid = current_mid + (diff > 0 ? max_diff : -max_diff);
        }

        // 6. 存储数据
        left_line.push_back(cv::Point(final_left, i));
        right_line.push_back(cv::Point(final_right, i));
        mid.push_back(cv::Point(calculated_mid, i));

        // 更新 current_mid 供下一行循环使用
        //std::cout << "defult_width: " << right_point - left_point << std::endl;
        current_mid = calculated_mid;
    }

    // 更新静态变量 last_mid，供下一帧图像使用 (帧间平滑)
    // 这里给 mid 的最底下一行 (也就是 i=150 那个点) 较高的权重
    if (!mid.empty()) {
        // 简单的帧间滤波
        last_mid = int(last_mid * 0.5 + mid[0].x * 0.5);
    }
}
/**
 * 专用于蓝色车道线或蓝色区域的跟踪函数
 * 
 * 该函数通过从图像底部向上逐行扫描的方式，寻找蓝色车道线的左右边界点，
 * 并计算每一行的中心点，从而生成完整的左右车道线和中心线轨迹。
 * 主要用于智能车在蓝色引导线或蓝色标识区域的导航。
 * 
 * @param dilated_image 输入的二值化膨胀图像，通常是经过预处理（颜色阈值分割、膨胀）后的蓝色车道线图像
 *                      白色像素(255)表示蓝色车道线/区域，黑色像素(0)表示背景
 */
void Tracking_blue(const cv::Mat &dilated_image)
{
    // 设置初始扫描起点（图像宽度的中间位置附近）
    int begin = 160;
    
    // 清空之前存储的蓝色车道线点集合
    left_line_blue.clear();   // 蓝色左车道线点集合
    right_line_blue.clear();  // 蓝色右车道线点集合
    mid_blue.clear();         // 蓝色车道中心线点集合
    
    // 从图像底部向上扫描（从y=153到y=heighest，heighest为可调整的上边界）
    for (int i = 153; i >= heighest; i--)
    {
        // 初始化左右蓝色车道线发现标志
        int find_l = 0;  // 左蓝色车道线是否找到
        int find_r = 0;  // 右蓝色车道线是否找到
        
        // 设置向左和向右搜索的起始位置
        int to_left = begin;
        int to_right = begin;

        // 向左搜索左蓝色车道线边界（从起始点向左移动）
        while (to_left != 1)  // 直到到达图像左边界
        {
            // 判断是否连续两个像素都是白色（即蓝色车道线）
            if (dilated_image.at<uchar>(i, to_left) == 255 && dilated_image.at<uchar>(i, to_left + 1) == 255)
            {
                find_l = 1;  // 找到左蓝色车道线
                left_line_blue.push_back(cv::Point(to_left, i));  // 记录左蓝色车道线点
                break;
            }
            else
            {
                to_left--;  // 继续向左搜索
            }
        }

        // 如果未找到左蓝色车道线（到达左边界），则使用左边界作为左蓝色车道线点
        if (to_left == 1)
        {
            left_line_blue.push_back(cv::Point(1, i));
        }

        // 向右搜索右蓝色车道线边界（从起始点向右移动）
        while (to_right != 318)  // 直到到达图像右边界
        {
            // 判断当前像素和左侧第二个像素是否都是白色（即蓝色车道线）
            if (dilated_image.at<uchar>(i, to_right) == 255 && dilated_image.at<uchar>(i, to_right - 2) == 255)
            {
                find_r = 1;  // 找到右蓝色车道线
                right_line_blue.push_back(cv::Point(to_right, i));  // 记录右蓝色车道线点
                break;
            }
            else
            {
                to_right++;  // 继续向右搜索
            }
        }

        // 如果未找到右蓝色车道线（到达右边界），则使用右边界作为右蓝色车道线点
        if (to_right == 318)
        {
            right_line_blue.push_back(cv::Point(318, i));
        }
        
        // 获取当前行的左右蓝色车道线点
        cv::Point midx1 = left_line_blue.back();
        cv::Point midx2 = right_line_blue.back();
        
        // 计算当前行的中心点并记录
        mid_blue.push_back(cv::Point(int((midx1.x + midx2.x) / 2), i));
        
        // 更新下一行的扫描起始位置（当前行左右边界的中点）
        begin = (to_right + to_left) / 2;
    }
}
/**
 * @brief 蓝色锥桶跟踪算法1 - 从图像底部向上逐行扫描检测蓝色区域边界并计算中间点
 * @param dilated_image 膨胀处理后的二值图像，其中蓝色锥桶区域已被标记为白色(255)
 * 
 * 该函数实现了一种蓝色锥桶检测跟踪算法，通过从图像底部向上逐行扫描的方式，检测每行中蓝色锥桶区域的左右边界，并计算中间点。
 * 检测结果分别存储在全局变量left_line(左边界点集合)、right_line(右边界点集合)和mid(中间点集合)中，供后续路径规划和控制决策使用。
 * 算法采用连续扫描策略，动态更新下一行的搜索起始位置，以提高跟踪稳定性和准确性。
 */
void Tracking_blue_zhuitong1(const cv::Mat &dilated_image)//寻找蓝色锥桶1
{
    int begin = 210;  // 初始搜索起始列位置，设置在图像水平中心附近(320像素宽度的图像约66%位置)
    left_line.clear();  // 清空左边界点集合，为新一轮检测做准备
    right_line.clear(); // 清空右边界点集合
    mid.clear();        // 清空中间点集合
    
    // 从图像底部(第153行)向上遍历到heighest行(全局变量，控制扫描的最高位置)
    for (int i = 153; i >= heighest; i--)
    {
        int find_l = 0;  // 左边界找到标记，0表示未找到，1表示已找到
        int find_r = 0;  // 右边界找到标记
        int to_left = begin;  // 向左搜索的当前列位置
        int to_right = begin; // 向右搜索的当前列位置

        // 向左搜索左边界：寻找连续两个白色像素点作为有效边界
        while (to_left != 1) // 未到达图像左边缘(第1列)
        {
            // 检测当前位置和右侧相邻位置是否均为白色像素(蓝色锥桶区域)
            if (dilated_image.at<uchar>(i, to_left) == 255 && dilated_image.at<uchar>(i, to_left + 1) == 255)
            {
                find_l = 1;  // 找到左边界
                left_line.push_back(cv::Point(to_left, i));  // 记录左边界点坐标(x,y)
                break;  // 跳出搜索循环
            }
            else
            {
                to_left--;  // 继续向左搜索
            }
        }

        // 边界情况处理：如果搜索到图像左边缘仍未找到有效边界，使用左边缘作为左边界
        if (to_left == 1)
        {
            left_line.push_back(cv::Point(1, i));
        }

        // 向右搜索右边界：寻找当前位置为白色且左侧第二个点也为白色的连续区域
        while (to_right != 318) // 未到达图像右边缘(第318列)
        {
            // 检测当前位置和左侧间隔一个位置是否均为白色像素，作为右边界判断条件
            if (dilated_image.at<uchar>(i, to_right) == 255 && dilated_image.at<uchar>(i, to_right - 2) == 255)
            {
                find_r = 1;  // 找到右边界
                right_line.push_back(cv::Point(to_right, i));  // 记录右边界点坐标(x,y)
                break;  // 跳出搜索循环
            }
            else
            {
                to_right++;  // 继续向右搜索
            }
        }

        // 边界情况处理：如果搜索到图像右边缘仍未找到有效边界，使用右边缘作为右边界
        if (to_right == 318)
        {
            right_line.push_back(cv::Point(318, i));
        }
        
        // 计算并记录中间点：取左右边界点的水平中点坐标
        cv::Point midx1 = left_line.back();  // 获取当前行的左边界点
        cv::Point midx2 = right_line.back(); // 获取当前行的右边界点
        mid.push_back(cv::Point(int((midx1.x + midx2.x) / 2), i)); // 计算中点并存储
        
        // 更新下一行的搜索起始位置为当前行左右边界的中点，实现连续跟踪，提高稳定性
        begin = (to_right + to_left) / 2;
    }
}
/**
 * @brief 蓝色锥桶跟踪算法2 - 从图像底部向上逐行扫描检测蓝色区域边界并计算中间点（左侧区域优化版）
 * @param dilated_image 膨胀处理后的二值图像，其中蓝色锥桶区域已被标记为白色(255)
 * 
 * 该函数是蓝色锥桶检测跟踪的第二个版本，与第一个版本算法流程相同，但针对左侧区域检测进行了优化。
 * 通过从图像底部向上逐行扫描的方式，检测每行中蓝色锥桶区域的左右边界，并计算中间点。
 * 检测结果存储在全局变量left_line(左边界点集合)、right_line(右边界点集合)和mid(中间点集合)中，供后续路径规划使用。
 * 算法采用连续扫描策略，动态更新下一行的搜索起始位置，提高跟踪稳定性和准确性。
 * 主要区别在于初始搜索位置设置在图像左侧区域(120像素)，更适合特定场景下的蓝色锥桶检测。
 */
void Tracking_blue_zhuitong2(const cv::Mat &dilated_image)//寻找蓝色锥桶2
{
    int begin = 120;  // 初始搜索起始列位置，设置在图像左侧区域(320像素宽度的图像约37.5%位置)
    left_line.clear();  // 清空左边界点集合，为新一轮检测做准备
    right_line.clear(); // 清空右边界点集合
    mid.clear();        // 清空中间点集合
    
    // 从图像底部(第153行)向上遍历到heighest行(全局变量，控制扫描的最高位置)
    for (int i = 153; i >= heighest; i--)
    {
        int find_l = 0;  // 左边界找到标记，0表示未找到，1表示已找到
        int find_r = 0;  // 右边界找到标记
        int to_left = begin;  // 向左搜索的当前列位置
        int to_right = begin; // 向右搜索的当前列位置

        // 向左搜索左边界：寻找连续两个白色像素点作为有效边界
        while (to_left != 1) // 未到达图像左边缘(第1列)
        {
            // 检测当前位置和右侧相邻位置是否均为白色像素(蓝色锥桶区域)
            if (dilated_image.at<uchar>(i, to_left) == 255 && dilated_image.at<uchar>(i, to_left + 1) == 255)
            {
                find_l = 1;  // 找到左边界
                left_line.push_back(cv::Point(to_left, i));  // 记录左边界点坐标(x,y)
                break;  // 跳出搜索循环
            }
            else
            {
                to_left--;  // 继续向左搜索
            }
        }

        // 边界情况处理：如果搜索到图像左边缘仍未找到有效边界，使用左边缘作为左边界
        if (to_left == 1)
        {
            left_line.push_back(cv::Point(1, i));
        }

        // 向右搜索右边界：寻找当前位置为白色且左侧第二个点也为白色的连续区域
        while (to_right != 318) // 未到达图像右边缘(第318列)
        {
            // 检测当前位置和左侧间隔一个位置是否均为白色像素，作为右边界判断条件
            if (dilated_image.at<uchar>(i, to_right) == 255 && dilated_image.at<uchar>(i, to_right - 2) == 255)
            {
                find_r = 1;  // 找到右边界
                right_line.push_back(cv::Point(to_right, i));  // 记录右边界点坐标(x,y)
                break;  // 跳出搜索循环
            }
            else
            {
                to_right++;  // 继续向右搜索
            }
        }

        // 边界情况处理：如果搜索到图像右边缘仍未找到有效边界，使用右边缘作为右边界
        if (to_right == 318)
        {
            right_line.push_back(cv::Point(318, i));
        }
        
        // 计算并记录中间点：取左右边界点的水平中点坐标
        cv::Point midx1 = left_line.back();  // 获取当前行的左边界点
        cv::Point midx2 = right_line.back(); // 获取当前行的右边界点
        mid.push_back(cv::Point(int((midx1.x + midx2.x) / 2), i)); // 计算中点并存储
        
        // 更新下一行的搜索起始位置为当前行左右边界的中点，实现连续跟踪，提高稳定性
        begin = (to_right + to_left) / 2;
    }
}
/**
 * @brief 蓝色锥桶跟踪的舵机PD控制器 - 计算舵机PWM值以实现蓝色锥桶的跟踪控制
 * @param target 目标位置的x坐标，通常是期望的图像中心点
 * @return float 计算得到的舵机PWM控制值，范围在500-1000之间
 * 
 * 该函数实现了一个PD(比例-微分)控制器，用于控制舵机转向以跟踪蓝色锥桶。
 * 控制器通过计算蓝色锥桶中心位置与目标位置的偏差，并根据偏差及其变化率生成控制信号。
 * 主要用于智能车在蓝色锥桶引导下的路径跟踪和方向控制。
 */
float servo_pd_blue(int target)
  //锥桶舵机
{
    // 获取蓝色锥桶中间点集合mid_blue中中间位置点的x坐标，作为当前实际位置
    int pidx = mid_blue[(int)(mid_blue.size() / 2)].x;
    cout << "servo_pd_blue" << pidx << endl;  // 输出调试信息，显示当前检测到的x坐标
    
    // 计算当前误差：目标位置减去实际位置
    error_first = target - pidx;
    
    // PD控制算法计算舵机PWM差值：Kp*error + Kd*(error-prev_error)，其中Kp=3.0，Kd=3.0
    servo_pwm_diff = 3.0 * error_first + 3.0 * (error_first - last_error);
    
    // 更新上一次误差，供下次计算微分项使用
    last_error = error_first;
    
    // 注意：以下条件判断没有实际作用，无论条件如何，servo_pwm_diff都被赋值为自身
    if (servo_pwm_diff < 0)
    {
        servo_pwm_diff = servo_pwm_diff;
    }
    else
    {
        servo_pwm_diff = servo_pwm_diff;
    }
    
    // 计算最终的舵机PWM值：以700为中点，加上计算得到的差值
    servo_pwm = 720 + servo_pwm_diff;
    
    // 限制舵机PWM值在有效范围内(500-1000)，防止超出硬件限制
    if (servo_pwm > 1020)
    {
        servo_pwm = 1020;  // 上限保护
    }
    else if (servo_pwm < 520)
    {
        servo_pwm = 520;  // 下限保护
    }
    
    // 返回计算得到的舵机PWM控制值
    return servo_pwm;
}
/**
 * @brief 变道后的舵机PD控制器 - 计算变道完成后舵机的PWM控制值以保持稳定行驶
 * @param target 目标位置的x坐标，通常是期望的图像中心点
 * @return float 计算得到的舵机PWM控制值，范围在550-786之间
 * 
 * 该函数实现了变道完成后使用的PD(比例-微分)控制器，专门用于变道后的路径跟踪和方向控制。
 * 与普通跟踪控制器相比，该函数使用了特定的中间点加权策略和不同的PWM输出范围限制，
 * 旨在提供更平稳的变道后行驶体验。
 */
float servo_pd_after(int target)
  //变道后
{
    int size = int(mid.size());  // 获取中间点集合mid的大小

    // cout << "size:" << size << endl;  // 调试信息，已注释
    
    // 计算当前实际位置：取中间点集合中特定三个点(索引23、20和25)的x坐标平均值
    // 这种设计可能是为了在变道后获得更稳定的路径中心点估计
    int pidx = int((mid[23].x + mid[20].x + mid[25].x) / 3);
    
    // 计算当前误差：目标位置减去实际位置
    error_first = target - pidx;
    
    // PD控制算法计算舵机PWM差值：使用全局变量kp和kd作为比例和微分系数
    servo_pwm_diff = kp * error_first + kd * (error_first - last_error);
    
    // cout << "servo_pwm_diff:" << servo_pwm_diff << endl;  // 调试信息，已注释
    
    // 更新上一次误差，供下次计算微分项使用
    last_error = error_first;
    
    // 计算最终的舵机PWM值：以756为中点，加上计算得到的差值
    servo_pwm = 720 + servo_pwm_diff;
    
    // 限制舵机PWM值在变道后特定的有效范围内(550-786)，比常规控制范围更窄，以保证变道后的稳定性
    if (servo_pwm > 750)
    {
        servo_pwm = 750;  // 上限保护，比常规控制更严格
    }
    else if (servo_pwm < 520)
    {
        servo_pwm = 520;  // 下限保护，比常规控制更严格
    }
    
    // 返回计算得到的舵机PWM控制值
    return servo_pwm;
}

/**
 * @brief 左侧舵机PD控制器 - 计算左侧路径跟踪的舵机PWM控制值
 * @param target 目标位置的x坐标，通常是期望的图像中心点
 * @return float 计算得到的舵机PWM控制值，范围在500-826之间
 * 
 * 该函数实现了针对左侧路径跟踪的PD(比例-微分)控制器，用于智能车在左侧路径行驶时的方向控制。
 * 与其他控制器相比，该函数具有更大的PWM输出上限(826)，允许更大幅度的左转控制，
 * 适用于需要向左侧转向的场景，如左侧车道保持或左转弯控制。
 */
float servo_pd_left(int target)
{
    int size = int(mid.size());  // 获取中间点集合mid的大小

    // cout << "size:" << size << endl;  // 调试信息，已注释
    
    // 计算当前实际位置：取中间点集合中特定三个点(索引23、20和25)的x坐标平均值
    // 这种设计可以获得更稳定的路径中心点估计，减少单个点波动的影响
    int pidx = int((mid[23].x + mid[20].x + mid[25].x) / 3);
    
    // 计算当前误差：目标位置减去实际位置
    error_first = target - pidx;
    
    // PD控制算法计算舵机PWM差值：使用全局变量kp和kd作为比例和微分系数
    servo_pwm_diff = kp * error_first + kd * (error_first - last_error);
    
    // cout << "servo_pwm_diff:" << servo_pwm_diff << endl;  // 调试信息，已注释
    
    // 更新上一次误差，供下次计算微分项使用
    last_error = error_first;
    
    // 计算最终的舵机PWM值：以700为中点，加上计算得到的差值
    servo_pwm = 720 + servo_pwm_diff;
    
    // 限制舵机PWM值在特定的有效范围内(500-826)，其中上限826较大，允许更大幅度的左转
    if (servo_pwm > 790)
    {
        servo_pwm = 790;  // 上限保护，允许比常规控制更大的左转角度
    }
    else if (servo_pwm < 470)
    {
        servo_pwm = 470;  // 下限保护
    }
    
    // 返回计算得到的舵机PWM控制值
    return servo_pwm;
}

/**
 * @brief 左侧路径跟踪的PD控制器实现
 * @param target 目标位置（通常是期望的中心线x坐标）
 * @return 计算得到的舵机PWM控制值，范围在550-900之间
 * @note 这个函数与servo_pd_left类似，但允许更大范围的转向控制（上限扩展到900）
 */
float servo_pd_left1(int target)
{
    // 获取mid向量的大小，mid向量存储了检测到的路径中心点
    int size = int(mid.size());

    // 计算实际位置：取mid数组中索引23、20和25的x坐标平均值
    // 这三个点可能对应于车辆前方不同距离的路径中心点，通过平均提高稳定性
    int pidx = int((mid[23].x + mid[20].x + mid[25].x + mid[27].x + mid[29].x + mid[31].x) / 6);
    
    // 计算当前偏差：目标位置与实际位置之差
    error_first = target - pidx;
    
    // PD控制算法：比例项(P) + 微分项(D)
    // kp为比例系数，kd为微分系数
    servo_pwm_diff = kp * error_first + kd * (error_first - last_error);
    
    // 更新上一次偏差，用于下一次微分计算
    last_error = error_first;
    
    // 计算最终舵机PWM值：基准值700加上PD控制量
    // 766可能是车辆直线行驶时的基准PWM值
    servo_pwm = 720 + servo_pwm_diff;
    
    // PWM输出限幅：确保值不超过870（允许更大幅度的左转）
    if (servo_pwm > 870)
    {
        servo_pwm = 870;
    }
    // PWM输出限幅：确保值不低于520（防止过度右转）
    else if (servo_pwm < 520)
    {
        servo_pwm = 520;
    }
    
    // 返回计算得到的舵机PWM控制值
    return servo_pwm;
}

/**
 * @brief 智能车正常循迹的PD控制器实现
 * @param target 目标位置（通常是期望的中心线x坐标）
 * @return 计算得到的舵机PWM控制值，范围在600-900之间
 * @note 这是智能车最基本的路径跟踪控制器，用于常规道路场景下的稳定行驶
 */
float servo_pd(int target)
  //正常循迹
{
    // 获取mid向量的大小，mid向量存储了检测到的路径中心点
    int size = int(mid.size());

    // 1. 安全检查：如果没有找到线，保持上一次的舵机值或回正 
    if (size == 0) return servo_pwm;  
    
    // 2. 动态选择索引，防止越界 
    // 我们希望取远处（索引大）的点，但不能超过 size-1 
    int idx1 = std::min(23, size - 1);  
    int idx2 = std::min(30, size - 1); 
    int idx3 = std::min(25, size - 1); 
    int idx4 = std::min(27, size - 1);
    int idx5 = std::min(28, size - 1);
    int idx6 = std::min(31, size - 1);

    // 3. 计算实际位置 (使用受保护的索引) 
    int pidx = int((mid[idx1].x + mid[idx2].x + mid[idx3].x + mid[idx4].x + mid[idx5].x + mid[idx6].x) / 6); 

    
    float filter = 0.7;// 低通滤波系数，用于平滑路径跟踪
    float kp_term;
    float kd_term;
    // 计算实际位置：取mid数组中索引23、20和25的x坐标平均值
    // 这三个点可能对应于车辆前方不同距离的路径中心点，通过平均提高稳定性
//    int pidx = int((mid[15].x + mid[17].x + mid[19].x + mid[21].x + mid[23].x + mid[24].x) / 6);//近
    //int pidx = int((mid[23].x + mid[25].x + mid[27].x + mid[28].x + mid[19].x + mid[21].x) / 6);//远
    // 计算当前偏差：目标位置与实际位置之差
    error_first = target - pidx;

    const int ALPHA = 0.3; // 调整这个值：0.1-0.5

    // 在控制循环中
    error_first = target - pidx;
    // 对误差进行限幅，防止过大的控制输出
    if(abs(error_first)<8)
    {
        error_first = 0;        
    }
    if(error_first>39)
    {
    		error_first = 39;
    }
    	if(error_first<-39)
    {
    		error_first = -39;
    }
    // PD控制算法：比例项(P) + 微分项(D)
    // kp为比例系数，kd为微分系数
    // 低通滤波：平滑PD控制输出，减少高频波动
    kp_term = kp * error_first;
    kd_term = kd * (error_first - last_error);
    float kd_filter = filter * kd_filter + (1 - filter) * kd_term;
    servo_pwm_diff = kp_term + kd_filter;
    // 更新上一次偏差，用于下一次微分计算
    last_error = error_first;
    error_first = ALPHA * error_first + (1 - ALPHA) * last_error;
    // 可选的调试输出，用于监控控制量变化
    // cout << "servo_pwm_diff:" << servo_pwm_diff << endl;
    
    // 计算最终舵机PWM值：基准值720加上PD控制量
    // 756是车辆直线行驶时的基准PWM值
    servo_pwm = 858 - servo_pwm_diff;
    //756
    // PWM输出限幅：确保值不超过900（防止过度左转）
    if (servo_pwm >980)
    {
        servo_pwm = 980;
    }//900
    // PWM输出限幅：确保值不低于570（防止过度右转）
    else if (servo_pwm < 740)
    {
        servo_pwm = 740;
    }
    
    // 返回计算得到的舵机PWM控制值
    return servo_pwm;
}

/**
 * @brief 智能车快速循迹的PD控制器实现
 * @param target 目标位置（通常是期望的中心线x坐标）
 * @return 计算得到的舵机PWM控制值，范围在500-1000之间
 * @note 这个控制器专为高速行驶场景设计，具有更大的转向范围和特定的PD参数
 */
float servo_pd_fast(int target)
  //快速循迹
{
    // 获取mid向量的大小，mid向量存储了检测到的路径中心点
    int size = int(mid.size());
    
    // 可选的调试输出，用于监控mid向量大小
    // cout << "size:" << size << endl;
    
    // 计算实际位置：取mid数组中索引23、20和25的x坐标平均值
    // 这三个点对应车辆前方不同距离的路径中心点，通过平均提高高速行驶时的稳定性
    int pidx = int((mid[23].x + mid[20].x + mid[25].x) / 3);
    
    // 备用的实际位置计算方法（当前已注释掉）：使用line_mid及其偏移点
    // int pidx = int((mid[line_mid].x + mid[line_mid - 5].x + mid[line_mid - 10].x) / 3);
    
    // 计算当前偏差：目标位置与实际位置之差
    error_first = target - pidx;
    
    // PD控制算法：使用固定的比例系数(Kp=1.0)和微分系数(Kd=2.0)
    // 这些参数专为高速行驶场景优化，提供更灵敏的响应
    servo_pwm_diff = 1.0 * error_first + 2.0 * (error_first - last_error);
    
    // 可选的调试输出，用于监控控制量变化
    // cout << "servo_pwm_diff:" << servo_pwm_diff << endl;
    
    // 更新上一次偏差，用于下一次微分计算
    last_error = error_first;
    
    // 计算最终舵机PWM值：基准值700加上PD控制量
    // 756是车辆直线行驶时的基准PWM值
    servo_pwm = 720 + servo_pwm_diff;
    
    // PWM输出限幅：确保值不超过1000（允许更大幅度的左转，适应高速场景）
    if (servo_pwm > 970)
    {
        servo_pwm = 970;
    }
    // PWM输出限幅：确保值不低于500（允许更大幅度的右转，适应高速场景）
    else if (servo_pwm < 470)
    {
        servo_pwm = 470;
    }
    
    // 返回计算得到的舵机PWM控制值
    return servo_pwm;
}

/**
 * @brief 计算整数向量的平均值
 * @param vec 输入的整数向量
 * @return 向量中所有元素的平均值（双精度浮点数），如果向量为空则返回-1
 * @note 该函数在智能车系统中可能用于计算多个数据点的平均值，如传感器数据滤波或路径中心点统计
 */
double average(vector<int> vec)
{
    // 边界检查：如果向量为空，返回-1作为错误标志
    if (vec.size() < 1)
        return -1;

    // 初始化总和变量
    double sum = 0;
    
    // 遍历向量，累加所有元素的值
    for (int i = 0; i < vec.size(); i++)
    {
        sum += vec[i];
    }

    // 计算并返回平均值：总和除以元素个数
    // 使用double类型转换确保结果精度
    return (double)sum / vec.size();
}

/**
 * @brief 计算整数向量的方差（均方差）
 * @param vec 输入的整数向量
 * @return 向量中所有元素的方差值（双精度浮点数），如果向量为空则返回0
 * @note 该函数在智能车系统中主要用于数据分析，如评估传感器数据的稳定性或路径跟踪的偏差程度
 */
double sigma(vector<int> vec)
{
    // 边界检查：如果向量为空，返回0作为默认值
    if (vec.size() < 1)
        return 0;

    // 调用average函数计算向量的平均值
    double aver = average(vec);
    
    // 初始化方差累加变量
    double sigma = 0;
    
    // 遍历向量，计算每个元素与平均值差的平方并累加
    // 这是方差计算的第一步：求和所有(xi-μ)²
    for (int i = 0; i < vec.size(); i++)
    {
        sigma += (vec[i] - aver) * (vec[i] - aver);
    }
    
    // 完成方差计算：将累加和除以元素个数，得到平均平方差
    sigma /= (double)vec.size();
    
    // 返回计算得到的方差值
    return sigma;
}


// ==================== 键盘监听模块开始 ====================
// 辅助函数：检测是否有按键被按下 (非阻塞)
int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO); // 禁用缓冲和回显
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}
// 辅助函数：读取一个字符 (不需回车)
char getch(void)
{
    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0)
        perror("tcsetattr()");
        old.c_lflag &= ~ICANON; // 关闭缓冲区
        old.c_lflag &= ~ECHO; // 关闭回显
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0)
        perror("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
        perror("tcsetattr ~ICANON");
    return buf;
}

atomic<int> key(8); // 使用 atomic 以便在线程间安全访问
atomic<bool> is_set(false); // 标志变量，确保未知数只能被修改一次
atomic<int> set_count(0); // 计数器，跟踪未知数被修改的次数

/** 
 * @brief 用户输入处理线程函数 (修改版) 
 * @details 支持输入 "+" "-" 控制变道和停车位，同时也支持输入数字设置 key 
 */ 
void input_thread() { 
    std::string input_str; // 使用字符串接收，以便处理 "+" "-" 和数字 
    int value; 
 
    while (true) { 
    // 使用 getch() 读取一个字符，无需回车
// 注意：这里是阻塞读取一个字符，但因为是在独立线程中，所以不会阻塞主程序
    char ch = getch();
    // 1. 处理 '+' 号
    if (ch == '+' || ch == '=') // 键盘上+通常和=是一个键，兼容一下
    {
        if (Running_State == CROSS_STOP)
        {
            biandao = 1;
            std::cout << "\n>>> [即时生效] 状态:CROSS_STOP, biandao = 1 (右)" << std::endl;
        }
        else if (Running_State == FIND_CHANGE_LINE)
        {
            biandao2 = 1;
            std::cout << "\n>>> [即时生效] 状态:FIND_CHANGE_LINE, biandao2 = 1" << std::endl;
        }
        else if (Running_State == FIND_PARK_SIGN)
        {
            //parking = 1; // 或者是 park_place = 1，根据你逻辑定
            park_place = 1; // 同时赋值给旧变量以防万一
            std::cout << "\n>>> [即时生效] 状态:FIND_PARK_SIGN, parking = 1" << std::endl;
        }
        else
        {
            std::cout << "\n>>> [无效操作] 当前状态不支持 '+'" << std::endl;
        }
    }
    // 2. 处理 '-' 号
    else if (ch == '-')
    {
        if (Running_State == CROSS_STOP)
        {
            biandao = -1;
            std::cout << "\n>>> [即时生效] 状态:CROSS_STOP, biandao = -1 (左)" << std::endl;
        }
        else if (Running_State == FIND_CHANGE_LINE)
        {
            biandao2 = -1;
            std::cout << "\n>>> [即时生效] 状态:FIND_CHANGE_LINE, biandao2 = -1" << std::endl;
        }
        else if (Running_State == FIND_PARK_SIGN)
        {
            //parking = -1;
            park_place = -1;
            std::cout << "\n>>> [即时生效] 状态:FIND_PARK_SIGN, parking = -1" << std::endl;
        }
        else
        {
            std::cout << "\n>>> [无效操作] 当前状态不支持 '-'" << std::endl;
        }
    }
    // 3. 处理数字按键 (保留你原来的 key 逻辑)
    else if (ch >= '0' && ch <= '9')
    {
    int value = ch - '0'; // 字符转数字
        // 简单的防抖逻辑：如果 set_count 满了就不处理了
        if (set_count.load() < 10)
        {
            key.store(value);
            std::cout << "\n>>> [按键设置] 未知数 key = " << key.load() << std::endl;
            if (key.load() == 2 || key.load() == 3)
            {
                try_patching_line = 1;
                std::cout << "变线值为: " << try_patching_line << std::endl;
            }
            else if (key.load() == 0 || key.load() == 1)
            {
                try_patching_line = 2;
                std::cout << "变线值为: " << try_patching_line << std::endl;
            }
            set_count.fetch_add(1);
            if (set_count.load() == 10) {
                is_set.store(true);
            }
        }
    }
    // 稍微休眠释放 CPU
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
// ==================== 键盘监听模块结束 ====================


/**
 * @brief 电机与舵机控制函数
 * @param flag_yellow_cond 黄色条件标志
 * @param cross 斑马线检测标志（1表示检测到斑马线）
 * @note 该函数实现智能车在不同场景下的运动控制，包括锥桶跟踪、斑马线前停车、变道控制等功能
 */
void motor_servo_contral(int flag_yellow_cond, int cross)
{
    // 读取当前按键状态（原子操作）
    int current_key3 = key.load();
    
    //-----------------------------------------锥桶方案-----------------------------------------------------
    // 条件1：按33键停车，或黄色标志完成且十字标志完成且计数超过8
    if (current_key3 == 33 || flag_yellow_finish == 0 && flag_cross_finish == 1 && count_change > 8)
    {
        // 设置黄色标志为完成状态
        flag_yellow_finish = 1;
        // 设置舵机为中间位置
        gpioPWM(12, 720);
        // 停止电机（设置PWM为最大值）
        gpioPWM(13, 10000);
        // 短暂延时
        usleep(250000);//延时250ms
        // 再次停止电机
        gpioPWM(13, 10000);
        gpioPWM(13, 10000);

        motorSpeed = 10000;
        // 短暂延时
        usleep(250000);
        // 退出程序
        _exit(0);
    }
    // 条件2：检测到斑马线且十字标志未完成（斑马线前停车）
    else if (cross == 1 && flag_cross_finish == 0)
    {
        // 读取当前按键状态
        int current_key = key.load();
        // 设置十字标志为完成状态
        flag_cross_finish = 1;
        // 停止电机
        gpioPWM(13, 10000);
        gpioPWM(13, 10000);
        // 设置舵机为中间位置
        gpioPWM(12, 720);

        motorSpeed = 10000;
        // 延时1.05秒
        usleep(1050000);
        // 播放声音提示
        system("aplay /home/pi/Smartcar_5g-mainchai/Smartcar_5g-mainchai/say.wav");
        // 再次设置舵机和电机状态
        gpioPWM(12, 720);
        gpioPWM(13, 10000);
        
        // 根据不同按键执行不同的变道操作
        if(current_key == 0)  // 左变道停A点（左左）
        {
            std::cout << "zuo" << std::endl;
            // 当未完成变道时，持续向左变道
            while (biandao == 0) 
            {
                gpioPWM(12, 740);  // 舵机稍向左转
                gpioPWM(13, 9200); // 设置电机速度
                motorSpeed = 9200;
            }
        }
        if(current_key == 1)  // 左变道停B点（左右）
        {
            std::cout << "zuo" << std::endl;
            // 当未完成变道时，持续向左变道
            while (biandao == 0) 
            {
                gpioPWM(12, 740);  // 舵机稍向左转
                gpioPWM(13, 9200); // 设置电机速度
                motorSpeed = 9200;
            }
        }
        if(current_key == 2)  // 右变道停A点（右左）
        {
            std::cout << "you" << std::endl;
            // 当未完成变道时，持续向右变道
            while (biandao == 0) 
            {
                gpioPWM(12, 660);  // 舵机稍向右转
                gpioPWM(13, 9200); // 设置电机速度
                motorSpeed = 9200;
            }
        }
        if(current_key == 3)  // 右变道停B点（右右）
        {
            std::cout << "you" << std::endl;
            // 当未完成变道时，持续向右变道
            while (biandao == 0) 
            {
                gpioPWM(12, 660);  // 舵机稍向右转
                gpioPWM(13, 9200); // 设置电机速度
                motorSpeed = 9200;
            }
        }
        std::cout << "程序已从休眠中恢复。" << std::endl;
    }
    // 条件3：检测到轮廓且计数小于3且数量1大于等于7且十字标志已完成（锥桶方案，实际未使用）
    else if (contours_all.size() != 0 && count_change < 3 && number1 >= 7 && flag_cross_finish == 1)
    {
        // 根据变道标志执行不同的蓝色锥桶跟踪
        if (try_patching_line == 2 && count_change < 3 && count_change >= 1)
        {
            float servo_pwm_chayan = servo_pd_blue(160);
            servo_pwm_now = servo_pwm_chayan;
        }
        else if (try_patching_line == 1 && count_change >= 1 && count_change < 3)
        {
            float servo_pwm_chayan = servo_pd_blue(170);
            servo_pwm_now = servo_pwm_chayan;
        }
        else
        {
            servo_pwm_now = servo_pd_blue(160);
        }
        // 输出调试信息
        cout << "bin_imagepwm" << servo_pwm_now << endl;
        cout << "------------------------" << endl;
        // 首次执行时设置电机速度
        if (times4 == 0)
        {
            times4 = 1;
            gpioPWM(13, 10000);//12550
            motorSpeed = 10000;
        }
        // 应用舵机控制信号
        gpioPWM(12, servo_pwm_now);
    }
    // 条件4：其他情况（斑马线前和锥桶后+锥桶）
    else
    {
        // 静态变量，记录总持续时间
        static int recorded_total_duration = 0;
        // 静态布尔变量，确保只记录一次时间
        static bool is_recorded = false;
        long current_total = 0;
        
        // 子条件1：计数小于1或大于2（锥桶前或锥桶后）
        if (count_change < 1 || count_change > 2)
        {
            cout << 1 << endl;
            // 读取当前按键状态
            int current_key2 = key.load();
            // 获取总持续时间
            current_total = total_duration;
            
            // 子条件1.1：刚走完锥桶并且还没记录时间，记录此时时间
            if (count_change == 3 && !is_recorded)
            {
                // 记录当前总持续时间
                recorded_total_duration = current_total;
                std::cout << "3xainzaiTotal Duration: " << recorded_total_duration << std::endl;
                is_recorded = true; // 确保只记录一次
            }
            
            // 子条件1.2：计数大于2且十字标志已完成（锥桶后）
            if (count_change > 2 && flag_cross_finish == 1)
            {
                // 根据不同按键状态执行不同的路径跟踪策略
                if(current_key2 == 0)  // 0是靠左向A走
                {
                    // 锥桶后4秒的动作执行，即靠左走A
                    if(current_total - recorded_total_duration > 4000)
                    {
                        servo_pwm_now = servo_pd(183);  // 目标值设为183（靠左）
                        std::cout << "tc-A" << endl;
                        cout << "pwm" << servo_pwm_now << endl;
                        gpioPWM(13, 9100);  // 设置电机速度
                        gpioPWM(12, servo_pwm_now);  // 应用舵机控制
                        motorSpeed = 9100;
                    }
                    else  // 在到4秒之前寻左线
                    {
                        servo_pwm_now = servo_pd(123);
                        std::cout << "tcq-you" << endl;  // 停车前-右
                        cout << "pwm" << servo_pwm_now << endl;
                        gpioPWM(13, 9100);  // 设置电机速度
                        gpioPWM(12, servo_pwm_now);  // 应用舵机控制
                        motorSpeed = 9100;
                    }
                }
                if(current_key2 == 1)  // 1是靠右走B
                {
                    // 锥桶后4秒的动作执行
                    if(current_total - recorded_total_duration > 4000)
                    {
                        servo_pwm_now = servo_pd(123);
                        std::cout << "tc-B" << endl;
                        cout << "pwm" << servo_pwm_now << endl;
                        gpioPWM(13, 9100);  // 设置电机速度
                        gpioPWM(12, servo_pwm_now);  // 应用舵机控制
                        motorSpeed = 9100;
                    }
                    else  // 在到4秒之前寻右线
                    {
                        servo_pwm_now = servo_pd(123);
                        std::cout << "tcq-you" << endl;
                        cout << "pwm" << servo_pwm_now << endl;
                        gpioPWM(13, 9100);  // 设置电机速度
                        gpioPWM(12, servo_pwm_now);  // 应用舵机控制
                        motorSpeed = 9100;
                    }
                }
                if(current_key2 == 2)  // 2是靠左走A
                {
                    // 锥桶后4秒的动作执行
                    if(current_total - recorded_total_duration > 4000)
                    {
                        servo_pwm_now = servo_pd(183);
                        std::cout << "tc-A" << endl;
                        cout << "pwm" << servo_pwm_now << endl;
                        gpioPWM(13, 9100);  // 设置电机速度
                        gpioPWM(12, servo_pwm_now);  // 应用舵机控制
                        motorSpeed = 9100;
                    }
                    else  // 在到4秒之前寻左线
                    {
                        servo_pwm_now = servo_pd(183);
                        std::cout << "tcq-zuo" << endl;
                        cout << "pwm" << servo_pwm_now << endl;
                        gpioPWM(13, 9100);  // 设置电机速度
                        gpioPWM(12, servo_pwm_now);  // 应用舵机控制
                        motorSpeed = 9100;
                    }
                }
                if(current_key2 == 3)  // 3是靠右走B
                {
                    // 锥桶后4秒的动作执行
                    if(current_total - recorded_total_duration > 4000)
                    {
                        servo_pwm_now = servo_pd(123);
                        std::cout << "tc-B" << endl;
                        cout << "pwm" << servo_pwm_now << endl;
                        gpioPWM(13, 9100);  // 设置电机速度
                        gpioPWM(12, servo_pwm_now);  // 应用舵机控制
                        motorSpeed = 9100;
                    }
                    else  // 在到4秒之前寻左线
                    {
                        servo_pwm_now = servo_pd(183);
                        std::cout << "tcq-zuo" << endl;
                        cout << "pwm" << servo_pwm_now << endl;
                        gpioPWM(13, 9100);  // 设置电机速度
                        gpioPWM(12, servo_pwm_now);  // 应用舵机控制
                        motorSpeed = 9100;
                    }
                }
            }
            // 子条件1.3：计数小于1且十字标志未完成（斑马线前开始巡线）
            else if (count_change < 1 && flag_cross_finish == 0)
            {
                // 对于交叉线的速度控制，快了会走出去
                if(current_key2 == 22)
                {
                    servo_pwm_now = servo_pd(160);
                    cout << "pwm" << servo_pwm_now << endl;
                    gpioPWM(13, 9250);  // 交叉线速度
                    gpioPWM(12, servo_pwm_now);
                    std::cout << "jcx" << endl;
                    motorSpeed = 9250;
                }
                else  // 正常巡线速度                                                           正常巡线速度                  
                {
                    servo_pwm_now = servo_pd(160);
                    cout << "pwm" << servo_pwm_now << endl;
                    gpioPWM(13, 10000);  // 正常速度
                    gpioPWM(12, servo_pwm_now);
                    motorSpeed = 10000;
                }
            }
            // 子条件1.4：其他情况（既不是斑马线前又不是锥桶后，即锥桶运控+转弯后运控）
            else
            {
                // 2，3就是右变道，是指在完全右变道后按回车后的循迹
                if(current_key2 == 2 || current_key2 == 3)
                {
                    cout << "afteryou" << endl;
                    servo_pwm_now = servo_pd_after(183);  // 循左边
                    cout << "pwm" << servo_pwm_now << endl;
                    gpioPWM(13, 9250);  // 转弯后速度
                    gpioPWM(12, servo_pwm_now);
                    motorSpeed = 9250;
                }
                // 0，1就是左变道，是指在完全左变道后按回车后的循迹
                else if(current_key2 == 0 || current_key2 == 1)
                {
                    cout << "afterzuo" << endl;
                    servo_pwm_now = servo_pd_after(125);  // 循右边
                    cout << "pwm" << servo_pwm_now << endl;
                    gpioPWM(13, 9250);  // 转弯后速度
                    gpioPWM(12, servo_pwm_now);
                    motorSpeed = 9250;
                }
            }
        }
        // 子条件2：其他情况（绕锥桶部分）
        else
        {
            // try_patching_line == 2 表示靠左走
            if (try_patching_line == 2)
            {
                cout << "f" << endl;
                // 静态变量记录进入此分支的时间
                static int yanchif = total_duration;
                // 延时后执行左道巡线
                if(total_duration - yanchif > 00)
                {
                    servo_pwm_now = servo_pd_left(183);  // 巡左道
                    cout << "190" << endl;
                }
                else  // 延时前的控制
                {
                    servo_pwm_now = servo_pd_left(123);
                    cout << "136" << endl;
                }
                gpioPWM(13, 9300);  // 绕锥桶速度
                motorSpeed = 9300;
                gpioPWM(12, servo_pwm_now);
            }
            // try_patching_line == 1 表示靠右走
            else if (try_patching_line == 1)
            {
                cout << "h" << endl;
                // 静态变量记录进入此分支的时间
                static int yanchih = total_duration;
                // 延时后执行右道巡线
                if(total_duration - yanchih > 00)
                {
                    servo_pwm_now = servo_pd_left1(123);  // 巡右道
                    cout << "136" << endl;
                }
                else  // 延时前的控制
                {
                    servo_pwm_now = servo_pd_left(183);
                    cout << "190" << endl;
                }
                gpioPWM(13, 9300);  // 绕锥桶速度
                motorSpeed = 9300;
                gpioPWM(12, servo_pwm_now);
            }
            // 输出当前PWM值
            cout << "pwm" << servo_pwm_now << endl;
        }
    }
    // -----------------------------------------锥桶方案-----------------------------------------------------

    // ------------------------------------纯竞速（注释掉的代码）---------------------------------
    /*
    if (flag_yellow_cond != 0 && flag_yellow_finish == 0 && flag_cross_finish == 1)
    {
        flag_yellow_finish = 1;
        gpioPWM(12, 730);
        gpioPWM(13, 10000);
        usleep(250000);
        gpioPWM(13, 9400);
        gpioPWM(13, 8800);
        usleep(250000);
        cout << "停止" << endl;
        _exit(0);
    }
    else if (cross == 1 && flag_cross_finish == 0)
    {
        flag_cross_finish = 1;
        cout << "11" << endl;
        gpioPWM(13, 9400);
        gpioPWM(13, 8900);
        usleep(550000);
        gpioPWM(12, 730);
        gpioPWM(13, 10000);
        sleep(3);
    }
    else if (contours_all.size() != 0 && count_change < 3 && number1 >= 7)
    {
        if (try_patching_line == 2 && count_change < 3 && count_change >= 1)
        {
            float servo_pwm_chayan = servo_pd_blue(160);
            servo_pwm_now = servo_pwm_chayan;
        }
        else if (try_patching_line == 1 && count_change >= 1 && count_change < 3)
        {
            float servo_pwm_chayan = servo_pd_blue(160);
            servo_pwm_now = servo_pwm_chayan;
        }
        else
        {
            servo_pwm_now = servo_pd_blue(160);
        }
        if (times4 == 0)
        {
            times4 = 1;
            gpioPWM(13, 9400);
            gpioPWM(13, 8700);
            usleep(100000);
        }
        gpioPWM(13, 10000);
        gpioPWM(12, servo_pwm_now);
    }
    else
    {
        if (flag_cross_finish == 0)
        {
            servo_pwm_now = servo_pd(160);
            gpioPWM(13, 12000);
            gpioPWM(12, servo_pwm_now);

        }
        else
        {
            servo_pwm_now = servo_pd(160);
            gpioPWM(13, 12500);
            gpioPWM(12, servo_pwm_now);

        }
    }
    */
}
/**
 * @brief 绘制路径跟踪点函数
 * @note 该函数在摄像头采集的图像帧上绘制检测到的路径特征点，包括左边界、右边界和中间路径点，用于可视化和调试
 */
void draw_point()
{
    // 遍历左侧边界点集合，用红色圆形标记每个点
    for (const cv::Point &point : left_line)
    {
        // 在图像帧上绘制红色实心圆：圆心为point，半径5像素，颜色(0,0,255)表示红色，-1表示实心
        cv::circle(frame, point, 5, cv::Scalar(0, 0, 255), -1);
    }
    
    // 遍历右侧边界点集合，用绿色圆形标记每个点
    for (const cv::Point &point : right_line)
    {
        // 在图像帧上绘制绿色实心圆：圆心为point，半径5像素，颜色(0,255,0)表示绿色，-1表示实心
        cv::circle(frame, point, 5, cv::Scalar(0, 255, 0), -1);
    }
    
    // 遍历中间路径点集合，用蓝色圆形标记每个点
    for (const cv::Point &point : mid)
    {
        // 在图像帧上绘制蓝色实心圆：圆心为point，半径5像素，颜色(255,0,0)表示蓝色，-1表示实心
        cv::circle(frame, point, 5, cv::Scalar(255, 0, 0), -1);
    }
}
/**
 * @brief 绘制蓝色路径跟踪点函数
 * @note 该函数在摄像头采集的图像帧上绘制检测到的蓝色路径特征点，包括左边界、右边界和中间路径点，用于蓝色锥桶或蓝色路径的可视化和调试
 */
void draw_point_blue()
{
    // 遍历蓝色路径左侧边界点集合，用红色圆形标记每个点
    for (const cv::Point &point : left_line_blue)
    {
        // 在图像帧上绘制红色实心圆：圆心为point，半径5像素，颜色(0,0,255)表示红色，-1表示实心
        cv::circle(frame, point, 5, cv::Scalar(0, 0, 255), -1);
    }
    
    // 遍历蓝色路径右侧边界点集合，用绿色圆形标记每个点
    for (const cv::Point &point : right_line_blue)
    {
        // 在图像帧上绘制绿色实心圆：圆心为point，半径5像素，颜色(0,255,0)表示绿色，-1表示实心
        cv::circle(frame, point, 5, cv::Scalar(0, 255, 0), -1);
    }
    
    // 遍历蓝色路径中间点集合，用蓝色圆形标记每个点
    for (const cv::Point &point : mid_blue)
    {
        // 在图像帧上绘制蓝色实心圆：圆心为point，半径5像素，颜色(255,0,0)表示蓝色，-1表示实心
        cv::circle(frame, point, 5, cv::Scalar(255, 0, 0), -1);
    }
}

//------------------------------------------------------------------------------------------------------------------------------------

/**
 * @brief 计算轮廓的包围圆并在图像上标记圆心
 * @param contours_all 轮廓点集合的向量
 * @note 该函数遍历所有输入轮廓，为每个轮廓计算最小包围圆，并在原图像上用红色实心圆标记圆心位置
 */
void computer_area(vector<vector<Point>> contours_all)
{
    Point2f center; // 轮廓包围圆的圆心坐标
    float radius;  // 轮廓包围圆的半径
    
    // 遍历所有轮廓
    for (int i = 0; i < contours_all.size(); i++)
    {
        // 计算当前轮廓的最小包围圆
        minEnclosingCircle(contours_all[i], center, radius);
        // 在原图像上绘制红色实心圆标记圆心：半径8像素，颜色(0,0,255)表示红色，-1表示实心
        circle(frame, center, 8, Scalar(0, 0, 255), -1);
    }
}

/**
 * @brief 比较两个轮廓面积大小的函数
 * @param contour1 第一个轮廓点集合
 * @param contour2 第二个轮廓点集合
 * @return 如果第一个轮廓面积大于第二个则返回true，否则返回false
 * @note 该函数用于轮廓排序，按照面积从大到小排列
 */
bool Contour_Area(vector<Point> contour1, vector<Point> contour2)
{
    // 比较两个轮廓的面积并返回比较结果
    return contourArea(contour1) > contourArea(contour2);
}

/**
 * @brief 查找蓝色锥桶的主轮廓函数
 * @param mask 二值掩码图像，蓝色区域为白色，其他区域为黑色
 * @return 返回面积最大的轮廓点集合，对应检测到的蓝色锥桶
 * @note 该函数通过查找轮廓并按面积排序，返回最大的轮廓（假设为蓝色锥桶）
 */
vector<Point> blue_vertebral_barrel_find(Mat mask)
{
    vector<vector<Point>> contours;  // 存储检测到的所有轮廓
    vector<Vec4i> hierarcy;          // 存储轮廓的层次结构
    
    // 在二值掩码图像中查找所有外部轮廓
    findContours(mask, contours, hierarcy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    
    // 按照轮廓面积从大到小排序
    sort(contours.begin(), contours.end(), Contour_Area);
    
    // 返回面积最大的轮廓（假设为蓝色锥桶）
    return contours[0];
}

/**
 * 超鲁棒版蓝色锥桶检测与过滤（室外终极版 + 极致性能优化）
 * 
 * 核心优势：
 * 1. 只在 visualize==true 时才 clone 原图和执行任何绘图操作 → 比赛关闭可视化时几乎零额外开销
 * 2. 所有关键阈值集中到参数区，现场调参只需改这里
 * 3. 上部裁剪参数彻底解决远处天空蓝/广告牌干扰
 * 4. 路径边界容差可调
 * 
 * @param input_mask         输入的蓝色二值mask（白色255为蓝色区域）
 * @param source_frame       原始摄像头图像，用于可视化（传 frame）
 * @param visualize          是否打开可视化（比赛时务必 false，节省算力）
 * @param crop_top_pixels    从图像顶部裁掉的像素数（忽略远处干扰）
 *                           - 远处蓝天/广告牌多 → 80~120
 *                           - 想早点看到远处锥桶 → 40~60
 *                           - 默认60（240高约裁掉1/4）
 * @return                   过滤后的真实锥桶轮廓集合
 */
std::vector<std::vector<cv::Point>> blue_vertebral_barrel_find_all(
    cv::Mat input_mask,
    cv::Mat& source_frame,
    bool visualize = false,
    int crop_top_pixels = 110)/************************修改以减少天空影响****************************** */
{
    // ====================== 可调参数区（现场只改这里！）======================
    const int MIN_AREA            = 8;     // 最小面积：近大锥桶防漏 → 8~15         /***  8  600  80  200  0.25  15  140
    const int MAX_AREA            = 600;    // 最大面积：只关注3~6米内 → 400~800
    const int MIN_Y_MIN           = 100;     // Y下限（越小越远）：远处干扰 → 80~100
    const int MIN_Y_MAX           = 200;    // Y上限（越近越大）：近处防漏 → 180~210
    const double MIN_CIRCULARITY  = 0.25;   // 圆度：锥桶碎了 → 0.2；噪声多 → 0.35
    const int PATH_TOLERANCE      = 15;     // 路径容差：路径飘 → 20~30
    const int LARGE_BLUE_THRESHOLD = 100;   // 大锥桶阈值：触发Keep_Avoid用 → 200~300
    // ====================== 参数区结束 ======================

    // 1. 上部裁剪（直接操作ROI，不clone整张图）
    cv::Rect valid_roi(0, crop_top_pixels, input_mask.cols, input_mask.rows - crop_top_pixels);
    cv::Mat mask = input_mask(valid_roi);  // 只取ROI视图，不clone（零拷贝）

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        now_blue_max = 0;
        return contours;
    }

    std::vector<std::vector<cv::Point>> filtered;
    now_blue_max = 0;

    // 只有在需要可视化时才clone原图（节省大量内存+时间）
    cv::Mat vis_img;
    if (visualize) {
        vis_img = source_frame.clone();
    }

    for (const auto& cnt : contours)
    {
        double area = cv::contourArea(cnt);
        if (area < MIN_AREA || area > MAX_AREA) continue;

        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(cnt, center, radius);

        float real_y = center.y + crop_top_pixels;  // 换算回原图坐标
        if (real_y < MIN_Y_MIN || real_y > MIN_Y_MAX) continue;

        double perimeter = cv::arcLength(cnt, true);
        double circularity = (4 * CV_PI * area) / (perimeter * perimeter);
        if (circularity < MIN_CIRCULARITY) continue;

        // 路径边界检查
        int y_idx = static_cast<int>(real_y) - 108;
        y_idx = std::max(0, std::min(y_idx, (int)left_line.size() - 1));

        int left_bound  = left_line[y_idx].x + PATH_TOLERANCE;
        int right_bound = right_line[y_idx].x - PATH_TOLERANCE;

        if (center.x > left_bound && center.x < right_bound)
        {
            filtered.push_back(cnt);

            if (area > now_blue_max)
            {
                now_blue_max = static_cast<int>(area);
                center_x = center.x;
                center_y = real_y;
            }

            // === 只在 visualize==true 时才画图（零开销优化）===
            if (visualize)
            {
                cv::circle(vis_img, cv::Point(center.x, real_y), (int)radius + 2, cv::Scalar(0, 255, 0), 3);
                cv::circle(vis_img, cv::Point(center.x, real_y), 6, cv::Scalar(0, 255, 0), -1);
                cv::putText(vis_img, "CONE", cv::Point(center.x + 10, real_y - 10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            }
        }
    }

    // Keep_Avoid 触发逻辑
    bool has_large_blue = (now_blue_max > LARGE_BLUE_THRESHOLD);
    if (filtered.empty() && !contours.empty() && !has_large_blue && last_blue == 0)
    {            
        if(Running_State == AVOID_CONE)
        {
            Keep_Avoid = 1;
            TimeRecord_Start_1 = std::chrono::high_resolution_clock::now();
            std::cout << "【锥桶检测】大蓝块消失 → 触发Keep_Avoid" << std::endl;
        }
        
    }
    last_blue = has_large_blue ? 1 : 0;

    // 可视化窗口（只在需要时创建）
    if (visualize)
    {
        cv::imshow("Blue Cones Detection", vis_img);
        cv::waitKey(1);
    }

    return filtered;
}

/**
 * @brief 对图像上的圆进行整数化处理
 * @param frame 输入的图像矩阵，用于绘制圆
 * @param center 圆的中心坐标，Point2f类型
 * @param radius 圆的半径，整数类型
 * @return 返回整数化后的圆中心坐标，Point类型
 * @note 该函数主要用于在图像上绘制圆时，确保圆的中心坐标为整数，避免绘制模糊
 */
Point pic_frame(Mat &frame, Point center, int radius)
{
    // circle(frame, center, radius, Scalar(0, 255, 0), 2);//绘制绿色圆
    // circle(frame, center,8, Scalar(0, 0, 255), -1);//绘制红色圆心
    //    cout << "x : "<<int(center.x) << endl;//输出整数化后的x坐标
    //    cout << "y : "<<int(center.y) << endl;//输出整数化后的y坐标
    return Point(int(center.x), int(center.y));//返回整数化后的圆心点
}

/**
 * @brief 蓝色锥桶模型处理函数
 * @details 该函数用于处理检测到的蓝色锥桶，并根据不同的变道标志在图像上绘制白色引导线
 * @note 此函数依赖于全局变量contours_all、mask、try_patching_line、count_change、bin_image2、right_line、left_line等
 */
void blue_vertebral_model(void)
{
    // 调用blue_vertebral_barrel_find_all函数获取图像中所有蓝色锥桶的轮廓
    contours_all = blue_vertebral_barrel_find_all(mask, frame, false, 70);
    
    // 检查是否检测到蓝色锥桶轮廓
    if (contours_all.size() != 0)
    {
        Point2f center;  // 轮廓中心点坐标
        float radius;    // 轮廓外接圆半径
        
        // 计算第一个轮廓(面积最大的轮廓)的最小外接圆
        minEnclosingCircle(contours_all[0], center, radius);
        
        // 记录当前检测到的锥桶的y坐标(高度信息)
        heighest = center.y;
        
        // 如果变道标志为2(向右侧变道)
        if (try_patching_line == 2)
        {
            // 在二值图像上绘制从锥桶中心到右侧路径平均点的白色引导线
            // 引导线宽度为8像素
            bin_image2 = drawWhiteLine(bin_image2, Point(int(center.x), int(center.y)), 
                                      Point(int((right_line[0].x + right_line[1].x + right_line[2].x) / 3), 155), 8);
            
            // 打印调试信息：锥桶中心点坐标和标志
            cout << "center.x:" << center.x << endl;
            cout << "center.y:" << center.y << endl;
            cout << "1" << endl;
        }
        // 如果变道标志为1(向左侧变道)
        else if (try_patching_line == 1)
        {
            // 如果count_change不等于2，使用锥桶实际中心点
            if (count_change != 2)
            {
                // 在二值图像上绘制从锥桶中心到左侧路径平均点的白色引导线
                bin_image2 = drawWhiteLine(bin_image2, Point(int(center.x), int(center.y)), 
                                          Point(int((left_line[0].x + left_line[1].x + left_line[2].x) / 3), 155), 8);
            }
            // 如果count_change等于2，使用锥桶中心点向左偏移20像素的点
            else
            {
                // 在二值图像上绘制从偏移后的锥桶中心到左侧路径平均点的白色引导线
                bin_image2 = drawWhiteLine(bin_image2, Point(int(center.x - 20), int(center.y)), 
                                          Point(int((left_line[0].x + left_line[1].x + left_line[2].x) / 3), 155), 8);
            }
            
            // 打印调试信息：锥桶中心点坐标和标志
            cout << "center.x:" << center.x << endl;
            cout << "center.y:" << center.y << endl;
            cout << "2" << endl;
        }
    }
}
/** 
 * @brief 极简蓝色区域检测 (只数像素，不看形状) 
 * @param frame 输入图像 (BGR) 
 * @param side 检测哪一侧？ -1:查左边, 1:查右边 
 * @param debug 是否显示检测区域方便调试 
 * @return true: 这一侧有蓝色锥桶; false: 这一侧没有 
 */ 
bool check_blue_exist_simple(cv::Mat frame, int side, bool debug = false) 
{ 
	 // 1. 转换为HSV 
	 cv::Mat hsv; 
	 cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV); 
	 
	 // 2. 蓝色阈值 (根据你的代码里最稳的那个范围调整) 
	 // 你的代码里用了多种阈值，这里取一个比较宽容的范围 
	 // H: 90-170 (涵盖浅蓝到深蓝) 
	 // S: 43-255 (只要不是灰白) 
	 // V: 46-255 (只要不是死黑) 
	 cv::Scalar lower_blue(106, 25, 130); 
	 cv::Scalar upper_blue(135, 255, 255); 
	  
	 cv::Mat mask;  cv::inRange(hsv, lower_blue, upper_blue, mask); 
	 
	 // 3. 形态学去噪 (关键：去掉地面上的蓝色反光噪点) 
	 cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)); 
	 cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel); // 开运算去噪 
	 
	 // 4. 设定感兴趣区域 (ROI) 
	 // 这一步非常重要！我们只看车头两侧的特定区域，避开天空和远处 
	 int h = frame.rows; // 240 
	 int w = frame.cols; // 320 
	  
	 cv::Rect roi_rect; 
	 
	 // 下面的坐标参数你可以现场调： 
	 // y从 100 开始，避开远处天空 
	 // height 取 140，一直看到车底 
	 if (side == -1) {  
	 // 【查左边】只看左半边 (0 ~ 160) 
	 roi_rect = cv::Rect(0, 40, w/2, h - 40);  
	 } else {  
	 // 【查右边】只看右半边 (160 ~ 320) 
	 roi_rect = cv::Rect(w/2, 100, w/2, h - 100); 
	 } 
	 
	 // 确保ROI不越界 
	 roi_rect = roi_rect & cv::Rect(0, 0, w, h); 
	  
	 // 截取Mask 
	 cv::Mat roi = mask(roi_rect); 
	 
	 // 5. 数像素 (核心一步) 
	 int blue_pixel_count = cv::countNonZero(roi); 
	 
	 // 6. 判断是否存在 
	 // 阈值设定：如果这一块区域里蓝色像素超过 200 个，就认为有锥桶 
	 // 这个 200 你可以根据实际情况改，比如 100 或 500 
	 int threshold = 30;  
	 bool exists = (blue_pixel_count > threshold); 
	 
	 // --- 调试显示 --- 
//	 if (debug) { 
//	 cv::Mat debug_img = frame.clone(); 
//	 // 画出搜索框：绿色代表有，红色代表无  cv::Scalar color = exists ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255); 
//	 //cv::rectangle(debug_img, roi_rect, cvXor, 2); 
//	  
//	 // 写上像素数量 
//	 std::string text = std::to_string(blue_pixel_count); 
//	 cv::putText(debug_img, text, cv::Point(roi_rect.x + 10, roi_rect.y + 30),  
//	 cv::FONT_HERSHEY_SIMPLEX, 1.0, color, 2); 
//	  
//	 cv::imshow("Blue Check Debug", debug_img); 
//	 cv::waitKey(1); 
//	 } 
	 
	 return exists; 
}
/**
 * @brief 斑马线检测函数
 * @param frame 输入的图像帧
 * @return int 返回是否检测到斑马线(1表示检测到，0表示未检测到)
 * @details 该函数通过图像处理和像素分析来识别图像中的斑马线特征
 * @note 函数使用全局变量flag_cross来存储检测结果状态
 */
int crossroad(Mat frame)
{
    flag_cross = 0;
    zebra_y = -1;
    zebra_y_center = -1;

    int height = frame.rows;
    int width  = frame.cols;

    Mat hsv;
    cvtColor(frame, hsv, COLOR_BGR2HSV);

    // 修复：声明局部 mask1
    Mat mask1;
    //inRange(hsv, Scalar(0, 0, 220), Scalar(180, 40, 255), mask1);
    inRange(hsv, Scalar(109, 0, 165), Scalar(179, 255, 250), mask1);
    // 形态学
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(mask1, mask1, MORPH_OPEN,  kernel, Point(-1,-1), 2);
    morphologyEx(mask1, mask1, MORPH_CLOSE, kernel, Point(-1,-1), 2);

    // ROI
    int roi_y_start = height * 40 / 100;
    int roi_height  = height * 35 / 100;
    Mat src(mask1, Rect(width*1/8, roi_y_start, width*3/4, roi_height));

    // 4. 严格的逐行跳变检测
    int continuous_valid_lines = 0;
    const int MIN_LINES_NEEDED = 3;        // 至少连续7行才算（斑马线通常8~12条）
    const int MIN_JUMPS_PER_LINE = 6;     // 每行至少12次有效跳变（一条白条大概2次跳变，6条白条≈12次）

    for (int i = 0; i < src.rows; i++)
    {
        int jumps = 0;
        uchar last = src.at<uchar>(i, 10);

        for (int j = 11; j < src.cols - 10; j++)
        {
            uchar curr = src.at<uchar>(i, j);
            // 只要相邻像素从 <100 到 >200（或反过来）就算一次有效跳变
            if ((last < 100 && curr > 200) || (last > 200 && curr < 100))
            {
                jumps++;
            }
            last = curr;
        }

        // 如果这一行跳变足够多，连续计数+1；否则清零
        if (jumps >= MIN_JUMPS_PER_LINE)
        {
            continuous_valid_lines++;
        }
        else
        {
            continuous_valid_lines = 0;  // 必须真正连续，不能断
        }

        // 实时记录最早出现的位置（最上方 = 离车最远）
        if (continuous_valid_lines >= MIN_LINES_NEEDED && zebra_y == -1)
        {
            zebra_y = roi_y_start + i - MIN_LINES_NEEDED + 1;  // 斑马线上边界
            zebra_y_center = roi_y_start + i - MIN_LINES_NEEDED/2;
        }

        // 一旦满足条件就直接判定成功并退出（不需要继续扫完）
        if (continuous_valid_lines >= MIN_LINES_NEEDED)
        {
            flag_cross = 1;
            cout << "=== 检测到斑马线！ zebra_y = " << zebra_y << " ===" << endl;
            break;
        }
    }

    return flag_cross;
}
//                    int count = 0;
////                     int height = frame.rows;
////                     int width = frame.cols;
////
////    // 定义一个矩形区域，只取图像的下半部分
////                     cv::Rect lower_half(0, height / 2, width, height / 2);
////
////    // 从图像中裁剪出下半部分
////                      cv::Mat lower_half_image = frame(lower_half);
////
////    // 调整下半部分图像的大小
//                                  
//                      cv::resize(frame, frame, cv::Size(800, 400));
//                    // 图像灰度化
//                    cv::Mat gray_zebra;
//                    cv::cvtColor(frame, gray_zebra, cv::COLOR_BGR2GRAY);
//
//                    // 高斯滤波
//                    cv::Mat img_zebra;
//                    cv::GaussianBlur(gray_zebra, img_zebra, cv::Size(5, 5), 5);
//                    // Canny 边缘检测
//                     cv::Mat edges;
//                     cv::Canny(img_zebra, edges, 100, 200); // 低阈值100，高阈值200
//
//// 腐蚀
//                     cv::Mat kernel_Ero = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 1));
//                     cv::Mat img_Ero;
//                     cv::erode(edges, img_Ero, kernel_Ero, cv::Point(-1, -1), 3);
//
//                    // 膨胀
//                    cv::Mat kernel_Dia = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 1));
//                    cv::Mat img_Dia;
//                    cv::dilate(img_Ero, img_Dia, kernel_Dia, cv::Point(-1, -1), 1);
//
//                    // 轮廓检测
//                    std::vector<std::vector<cv::Point>> contours;
//                    std::vector<cv::Vec4i> hierarchy;
//                    cv::findContours(img_Dia, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
//
//                    // 克隆图片作输出
//                    // cv::Mat out = cropped_image.clone();
//                    for (size_t i = 0; i < contours.size(); ++i)
//                    {
//                        // 获取轮廓矩形
//                        cv::Rect rect = cv::boundingRect(contours[i]);
//                        // 绘制符合条件的轮廓
//                        if (rect.width > 10 && rect.height > 20)
//                        {
//                            count++; // 符合条件的轮廓数量加1
//                        }
//                    }
//                    if (count >= 7)
//                    {
//                         cout << "斑马线" << endl;
//                         flag_cross = 1;
//                    }
//        
//    
//                  cout << "flag_cross" << flag_cross << endl;
//                   return flag_cross;
//                  }
//
//
/**
 * @brief 黄色边缘检测函数
 * @param img 输入的RGB图像
 * @param visual_flag 是否显示处理结果的可视化标志
 * @return int 返回检测到的符合条件的黄色边缘线条数量
 * @details 该函数通过图像处理技术识别图像中接近水平的黄色边缘线条，主要用于检测道路上的黄色标识线
 */
int yellow_edge(Mat img, bool visual_flag)
{
    // 声明所需的图像变量
    Mat cropped_image, canvas, image;
    // 初始化检测到的黄色线条计数
    int yellow_num = 0;
    
    // 获取输入图像的高度和宽度
    int height = img.rows;
    int width = img.cols;
    // 计算图像高度的一半
    int half_height = int(height / 2);
    // 计算高度的1/20作为单位高度
    int per_height = int(height / 20);
    
    // 克隆输入图像，避免修改原图像
    image = img.clone();
    
    // 裁剪图像的下半部分区域（从半高+2个单位高度到半高+5个单位高度）
    // 这是假设黄色线条更可能出现在图像的下半部分
    cropped_image = image(Rect(0, half_height+per_height*2, width,per_height*3));
    
    // 如果需要可视化结果，创建画布用于绘制结果
    if (visual_flag == true)
    {
        canvas = cropped_image.clone();
    }
    
    // 将裁剪区域转换为灰度图像，为边缘检测做准备
    cvtColor(cropped_image, cropped_image, COLOR_BGR2GRAY);
    
    // 使用Canny边缘检测算法提取图像中的边缘
    // 参数：输入图像，输出图像，低阈值，高阈值，Sobel算子大小
    Canny(cropped_image, cropped_image, 50, 150, 3);
    
    // 声明存储检测到的线段的向量
    vector<Vec4i> lines;
    
    // 使用霍夫变换检测图像中的直线
    // 参数：输入图像，输出线段集合，距离分辨率，角度分辨率，阈值，线段最短长度，线段最大间隙
    HoughLinesP(cropped_image, lines, 1, CV_PI / 180, 25, 1, 70);
    
    // 在处理后的图像上绘制检测到的所有线条（用红色标记）
    for (size_t i = 0; i < lines.size(); i++) {
        Vec4i l = lines[i];
        line(cropped_image, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2, LINE_AA);
    }

    // 将标注后的图像保存到文件
    imwrite("output_image.jpg", cropped_image);
    
    // 检查是否检测到任何线条
    if (lines.size() == 0)
    {
        // 如果没有检测到线条，打印提示信息
        printf("No yellow edge detected!\n");
        return 0;
    }
    else
    {
        // 遍历所有检测到的线条
        for (size_t i = 0; i < lines.size(); i++)
        {
            // 获取当前线条的端点坐标
            Vec4i l = lines[i];
            // 计算线条的角度（弧度转角度）
            double angle = atan2(l[3] - l[1], l[2] - l[0]) * 180.0 / CV_PI;
            // 计算线条的长度
            double length = sqrt(pow(l[3] - l[1], 2) + pow(l[2] - l[0], 2));
            // 计算线条的宽高比
            double aspect_ratio = length / abs(l[3] - l[1]);
            
            // 筛选接近水平的线条（角度绝对值小于15度）
            if (abs(angle) < 15)
            {
                // 如果需要可视化结果
                if (visual_flag == true)
                {
                    // 在画布上用蓝色绘制符合条件的线条
                    line(canvas, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 0, 0), 2, LINE_AA);
                    // 打印线条的详细信息（端点坐标、角度、长度、宽高比）
                    printf("x: %d, y: %d, x1: %d, y1: %d, angle: %f, length: %f, aspect_ratio: %f\n", l[0], l[1], l[2],
                           l[3], angle, length, aspect_ratio);
                }
                
                // 打印线条信息和图像尺寸（调试用）
                cout << "Line " << i + 1 << ": (" << l[0] << ", " << l[1] << "), (" << l[2] << ", " << l[3] << ")" << endl;
                cout << "Height: " << height << endl;
                cout << "Width: " << width << endl;

                // 增加符合条件的黄色线条计数
                yellow_num += 1;
            }
        }
    }
    
    // 如果需要可视化结果，显示处理后的图像
    if (visual_flag == true)
    {
        imshow("Image_edge", canvas);
        waitKey(0);  // 等待用户按键继续
    }
    
    // 返回检测到的符合条件的线条数量
    return yellow_num;
}
    
/**
 * @brief 基于HSV颜色空间的黄色区域检测函数
 * @param img 输入的RGB图像
 * @param visual_flag 是否显示处理结果的可视化标志
 * @return int 返回检测到的符合条件的黄色区域数量
 * @details 该函数通过HSV颜色空间阈值分割和形态学操作，识别图像中符合特定面积和形状条件的黄色区域
 */
int yellow_hsv(Mat img, bool visual_flag)
{
    // 声明所需的图像变量
    Mat cropped_image, canvas, image;
    
    // 定义黄色在HSV颜色空间中的下界和上界阈值
    // 这些值经过调优，用于检测特定场景下的黄色物体
    Scalar lowerb = Scalar(18, 43, 152);   // 黄色下界：H=18, S=43, V=152
    Scalar upperb = Scalar(40, 150, 250);  // 黄色上界：H=40, S=150, V=250
    
    // 初始化检测到的黄色区域计数
    int yellow_num = 0;
    
    // 获取输入图像的高度和宽度
    int height = img.rows;
    int width = img.cols;
    // 计算图像高度的一半
    int half_height = int(height / 2);
    // 计算高度的1/20作为单位高度
    int per_height = int(height / 20);
    
    // 计算面积筛选的上下界（基于图像尺寸的相对值）
    // 小面积阈值：图像总面积的0.2775%
    int area_1 = int(0.002775 * height * width);
    // 大面积阈值：图像总面积的2.5%
    int area_2 = int(0.025 * height * width);
    
    // 克隆输入图像，避免修改原图像
    image = img.clone();
    
    // 裁剪图像的中间偏下区域（从半高-1个单位高度到半高+2个单位高度）
    // 这是假设黄色物体更可能出现在这个区域
    cropped_image = image(Rect(0, half_height - per_height, width, per_height * 3));
    
    // 如果需要可视化结果，创建画布用于绘制结果
    if (visual_flag == true)
    {
        canvas = cropped_image.clone();
    }
    
    // 将裁剪区域转换为HSV颜色空间，便于颜色阈值分割
    cvtColor(cropped_image, cropped_image, COLOR_BGR2HSV);
    
    // 对HSV图像进行形态学开操作（先腐蚀后膨胀）
    // 目的是去除小的噪声点，同时保留较大的黄色区域
    morphologyEx(cropped_image, cropped_image, MORPH_OPEN, getStructuringElement(MORPH_RECT, Size(3, 3)));
    
    // 根据HSV颜色阈值生成二值掩码图像
    // 黄色区域为255（白色），其他区域为0（黑色）
    inRange(cropped_image, lowerb, upperb, cropped_image);
    
    // 声明存储轮廓和层次结构的向量
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    
    // 查找图像中的轮廓
    // 参数：输入图像，输出轮廓，输出层次结构，轮廓检索模式，轮廓近似方法
    findContours(cropped_image, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
    // 遍历所有检测到的轮廓
    for (size_t i = 0; i < contours.size(); i++)
    {
        // 计算当前轮廓的面积
        double area = contourArea(contours[i]);
        
        // 筛选面积在指定范围内的轮廓
        if (area > area_1 && area < area_2)
        {
            // 计算轮廓的最小外接矩形
            Rect rect = boundingRect(contours[i]);
            // 计算矩形的宽高比
            double aspect_ratio = rect.width / rect.height;
            
            // 筛选宽高比大于10的轮廓（表示长条形物体）
            if (aspect_ratio > 10)
            {
                // 如果需要可视化结果
                if (visual_flag == true)
                {
                    // 在画布上用蓝色矩形标记符合条件的区域
                    rectangle(canvas, rect, Scalar(255, 0, 0), 2, LINE_AA);
                    // 打印矩形的详细信息（位置、尺寸、宽高比）
                    printf("x: %d, y: %d, width: %d, height: %d, aspect_ratio: %f\n", rect.x, rect.y, rect.width,
                           rect.height, aspect_ratio);
                }
                
                // 增加符合条件的黄色区域计数
                yellow_num += 1;
            }
        }
    }
    
    // 如果需要可视化结果，显示处理后的图像
    if (visual_flag == true)
    {
        imshow("Image_hsv", canvas);
        waitKey(0);  // 等待用户按键继续
    }
    
    // 返回检测到的符合条件的黄色区域数量
    return yellow_num;
}

/**
 * 变道箭头检测函数（完全复刻 Python 版 biandao 逻辑）
 * 返回值：
 *   -1 : 左变道箭头 ←
 *    0 : 未检测到有效箭头
 *    1 : 右变道箭头 →
 */
int detectLaneChangeArrow(cv::Mat& frame, bool debug = false)
{
    //*************************************初版************************************** */
    // // ========================== 1. 透视变换（鸟瞰） ==========================
    // // 原 Python 源点（600x400 图像）
    // // pts1 = [[150,120], [450,120], [150,250], [450,250]]
    // // 我们 320x240 按比例缩放 ≈ 0.533x
    // cv::Point2f src_pts[4] = {
    //     cv::Point2f(80,  72),   // 左上
    //     cv::Point2f(240, 72),   // 右上
    //     cv::Point2f(80,  150),  // 左下
    //     cv::Point2f(240, 150)   // 右下
    // };

    // // 目标矩形（和原 Python 一样拉平成矩形）
    // cv::Point2f dst_pts[4] = {
    //     cv::Point2f(80,  0),
    //     cv::Point2f(240, 0),
    //     cv::Point2f(80,  239),
    //     cv::Point2f(240, 239)
    // };

    // cv::Mat M = cv::getPerspectiveTransform(src_pts, dst_pts);
    // cv::Mat warped;
    // cv::warpPerspective(frame, warped, M, cv::Size(320, 240));

    // if (debug) {
    //     cv::imshow("warped (bird view)", warped);
    // }

    // // ========================== 2. 裁剪 ROI ==========================
    // // 原 Python: x_roi=150, y_roi=25, w=200, h=350
    // // 320x240 按比例缩放
    // int x_roi = 80;
    // int y_roi = 15;
    // int width_roi = 160;
    // int height_roi = 210;

    // cv::Rect roi_rect(x_roi, y_roi, width_roi, height_roi);
    // if (roi_rect.x + roi_rect.width > warped.cols || roi_rect.y + roi_rect.height > warped.rows) {
    //     return 0;
    // }
    // cv::Mat roi = warped(roi_rect).clone();

    // // ========================== 3. resize 到 400x400（和 Python 完全一致） ==========================
    // cv::Mat resized;
    // cv::resize(roi, resized, cv::Size(400, 400));

    // if (debug) {
    //     cv::imshow("biandao resized 400x400", resized);
    // }

    // // ========================== 4. HSV 提取蓝色底 ==========================
    // cv::Mat hsv;
    // cv::cvtColor(resized, hsv, cv::COLOR_BGR2HSV);

    // cv::Scalar lower_blue(63, 100, 130);
    // cv::Scalar upper_blue(179, 255, 255);

    // cv::Mat blue_mask;
    // cv::inRange(hsv, lower_blue, upper_blue, blue_mask);

    // std::vector<std::vector<cv::Point>> contours;
    // cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // int flag = 0;

    // if (!contours.empty())
    // {
    //     // 找最大蓝色轮廓
    //     auto largest = std::max_element(contours.begin(), contours.end(),
    //         [](const auto& a, const auto& b) { return cv::contourArea(a) < cv::contourArea(b); });

    //     cv::Rect blue_rect = cv::boundingRect(*largest);
    //     int blue_area = blue_rect.width * blue_rect.height;

    //     if (debug) {
    //         std::cout << "blue area: " << blue_area << std::endl;
    //         cv::rectangle(resized, blue_rect, cv::Scalar(0, 0, 255), 2);
    //     }
    //     std::cout << " blue_area:" << blue_area << std::endl;
    //     if (blue_area < 500) {
    //         return 0;   // 和 Python 一样，小于1000直接认为无效
    //     }

    //     // ========================== 5. BGR 空间提取白色箭头（不是HSV！） ==========================
    //     // 上半部
    //     cv::Mat upper_half = resized(cv::Rect(blue_rect.x, blue_rect.y,
    //                                           blue_rect.width, blue_rect.height / 2)).clone();
    //     cv::Mat upper_white;
    //     cv::inRange(upper_half, cv::Scalar(220, 220, 220), cv::Scalar(255, 255, 255), upper_white);

    //     // 下半部
    //     cv::Mat lower_half = resized(cv::Rect(blue_rect.x, blue_rect.y + blue_rect.height / 2,
    //                                           blue_rect.width, blue_rect.height - blue_rect.height / 2)).clone();
    //     cv::Mat lower_white;
    //     cv::inRange(lower_half, cv::Scalar(220, 220, 220), cv::Scalar(255, 255, 255), lower_white);

    //     // ========================== 6. 分别求上下半部的白色重心 ==========================
    //     int up_cp = blue_rect.x + blue_rect.width / 2;    // 默认中间
    //     int down_cp = up_cp;

    //     // 下半部
    //     std::vector<std::vector<cv::Point>> lower_contours;
    //     cv::findContours(lower_white, lower_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    //     if (!lower_contours.empty())
    //     {
    //         auto max_lower = std::max_element(lower_contours.begin(), lower_contours.end(),
    //             [](const auto& a, const auto& b) { return cv::contourArea(a) < cv::contourArea(b); });
    //         cv::Moments m = cv::moments(*max_lower);
    //         if (m.m00 > 0)
    //         {
    //             int cx = static_cast<int>(m.m10 / m.m00) + blue_rect.x;
    //             int cy = static_cast<int>(m.m01 / m.m00) + blue_rect.y + blue_rect.height / 2;
    //             down_cp = cx;

    //             if (debug) {
    //                 cv::circle(resized, cv::Point(cx, cy), 8, cv::Scalar(0, 255, 0), -1);  // 绿色
    //                 std::cout << "down_cp: " << down_cp << std::endl;
    //             }
    //         }
    //     }

    //     // 上半部
    //     std::vector<std::vector<cv::Point>> upper_contours;
    //     cv::findContours(upper_white, upper_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    //     if (!upper_contours.empty())
    //     {
    //         auto max_upper = std::max_element(upper_contours.begin(), upper_contours.end(),
    //             [](const auto& a, const auto& b) { return cv::contourArea(a) < cv::contourArea(b); });
    //         cv::Moments m = cv::moments(*max_upper);
    //         if (m.m00 > 0)
    //         {
    //             int cx = static_cast<int>(m.m10 / m.m00) + blue_rect.x;
    //             int cy = static_cast<int>(m.m01 / m.m00) + blue_rect.y;
    //             up_cp = cx;

    //             if (debug) {
    //                 cv::circle(resized, cv::Point(cx, cy), 8, cv::Scalar(255, 0, 0), -1);  // 蓝色
    //                 std::cout << "up_cp: " << up_cp << std::endl;
    //             }
    //         }
    //     }

    //     // ========================== 7. 判断箭头方向 ==========================
    //     if (up_cp > down_cp + 10)        // 右箭头（上半部偏右）
    //     {
    //         flag = 1;
    //         std::cout << "=== 右变道箭头 ===" << std::endl;
    //     }
    //     else if (up_cp < down_cp - 10)   // 左箭头（上半部偏左）
    //     {
    //         flag = -1;
    //         std::cout << "=== 左变道箭头 ===" << std::endl;
    //     }
    //     else
    //     {
    //         flag = 0;
    //     }
    // }

    // if (debug)
    // {
    //     cv::imshow("biandao final result", resized);
    //     cv::waitKey(1);
    // }

    // return flag;
    /*************************************************第二版***************************************************************/

    int crop_top = 60; // 裁掉原始图像最上面的多少行（避开天空/树影），建议 60~100
    // ========================== 0. 忽略图像上半部分（避开天空、树影） ==========================
    if (crop_top > 0 && crop_top < frame.rows) {
        frame = frame(cv::Rect(0, crop_top, frame.cols, frame.rows - crop_top));
    }

    // ========================== 1. 透视变换（鸟瞰）- 参数已针对躺牌大幅优化 ==========================
    cv::Point2f src_pts[4] = {
        cv::Point2f(60,  60),    // 左上
        cv::Point2f(260, 60),    // 右上
        cv::Point2f(40,  180),   // 左下（大幅下移）
        cv::Point2f(280, 180)    // 右下
    };

    cv::Point2f dst_pts[4] = {
        cv::Point2f(0,   0),
        cv::Point2f(320, 0),
        cv::Point2f(0,   240),
        cv::Point2f(320, 240)
    };

    cv::Mat M = cv::getPerspectiveTransform(src_pts, dst_pts);
    cv::Mat warped;
    cv::warpPerspective(frame, warped, M, cv::Size(320, 240));

    if (debug) cv::imshow("warped", warped);

    // ========================== 2. 统一resize到400x400（保持和原来Python一致的尺度） ==========================
    cv::Mat resized;
    cv::resize(warped, resized, cv::Size(400, 400));

    if (debug) cv::imshow("resized 400x400", resized);

    // ========================== 3. HSV提取蓝色底（阈值已放宽 + 强闭操作） ==========================
    cv::Mat hsv;
    cv::cvtColor(resized, hsv, cv::COLOR_BGR2HSV);

    cv::Mat blue_mask;
    cv::inRange(hsv, cv::Scalar(90, 35,197), cv::Scalar(179,255,255), blue_mask);///////////////////////////////////////////////////////////////////////////

    // 强闭操作，把躺牌打碎的蓝底粘回去
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15,15));
    cv::morphologyEx(blue_mask, blue_mask, cv::MORPH_CLOSE, kernel, cv::Point(-1,-1), 2);

    // ========================== 4. 找最大蓝色轮廓 + 面积过滤 ==========================
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) return 0;

    auto largest = std::max_element(contours.begin(), contours.end(),
        [](const auto& a, const auto& b) { return cv::contourArea(a) < cv::contourArea(b); });

    cv::Rect blue_rect = cv::boundingRect(*largest);
    int blue_area = blue_rect.width * blue_rect.height;

    if (blue_area < 60) return 0;   // 躺牌时面积会很小，150是150就够了

    if (debug) {
        cv::rectangle(resized, blue_rect, cv::Scalar(0,0,255), 3);
    }

    // ========================== 5. 提取白色箭头（HSV方式，最抗光照） ==========================
    cv::Mat white_mask;
    // V>200（亮） 且 S<60（不彩）
    cv::inRange(hsv, cv::Scalar(0, 0, 245), cv::Scalar(158, 50, 255), white_mask);////////////////////////////////////////////////////////////////
    cv::dilate(white_mask, white_mask, cv::Mat(), cv::Point(-1,-1), 2);

    // ========================== 6. 计算白色整体重心 ==========================
    cv::Moments m_white = cv::moments(white_mask, true);
    if (m_white.m00 == 0) return 0;  // 没白就退出
    int white_cx = static_cast<int>(m_white.m10 / m_white.m00);

    // ========================== 7. 判断方向（白色重心 vs 蓝底中心） ==========================
    int blue_center_x = blue_rect.x + blue_rect.width / 2;
    int offset = white_cx - blue_center_x;

    int flag = 0;
    if (offset > 8) {
        flag = 1;
        std::cout << "=== 检测到 右变道箭头 ===" << std::endl;
    }
    else if (offset < -8) {
        flag = -1;
        std::cout << "=== 检测到 左变道箭头 ===" << std::endl;
    }
    else {
        std::cout << "未检测到有效箭头" << std::endl;
    }

    // ========================== 8. debug显示 ==========================
    if (debug) {
        cv::circle(resized, cv::Point(white_cx, 150), 10, cv::Scalar(0,255,0), -1);  // 绿色白重心
        cv::circle(resized, cv::Point(blue_center_x, 150), 10, cv::Scalar(255,0,0), -1); // 蓝色蓝中心
        cv::imshow("biandao final result", resized);
        cv::waitKey(1);
    }

    return flag;
}

//左右箭头识别方案2/************************************************************************************ */
// 辅助函数：对4个顶点进行排序（左上，右上，右下，左下）
// 确保透视变换后的图像方向正确
void orderPoints(std::vector<cv::Point> pts, std::vector<cv::Point2f>& ordered) {
    // 按x坐标排序
    std::sort(pts.begin(), pts.end(), [](cv::Point a, cv::Point b) {
        return a.x < b.x;
    });

    // 此时 pts[0] 和 pts[1] 是左边的两个点，pts[2] 和 pts[3] 是右边的两个点
    std::vector<cv::Point> leftPts = {pts[0], pts[1]};
    std::vector<cv::Point> rightPts = {pts[2], pts[3]};

    // 按y坐标排序，区分上下
    std::sort(leftPts.begin(), leftPts.end(), [](cv::Point a, cv::Point b) {
        return a.y < b.y;
    });
    std::sort(rightPts.begin(), rightPts.end(), [](cv::Point a, cv::Point b) {
        return a.y < b.y;
    });

    ordered.clear();
    ordered.push_back(leftPts[0]);  // 左上
    ordered.push_back(rightPts[0]); // 右上
    ordered.push_back(rightPts[1]); // 右下
    ordered.push_back(leftPts[1]);  // 左下
}

/**
 * @brief 高鲁棒性变道箭头检测函数
 * @param frame 输入的原始图像
 * @param debug 是否显示调试窗口 (默认 false)
 * @return int 0:未检测到/无效, -1:左转, 1:右转
 */
int detectLaneChangeArrow2(cv::Mat& frame, bool debug = false) {
    if (frame.empty()) return 0;

    cv::Mat hsv, mask_blue, mask_white;
    
    // 1. 预处理与颜色分割
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    // 蓝色阈值 (根据你的环境微调，这里给的是一个宽泛的鲁棒范围)
    // H: 100-124 (蓝色), S: 43-255 (去除灰白), V: 46-255 (去除暗黑)
    cv::Scalar lower_blue(90, 35,197);        ////////////////////////////////////////////////////////////////////////////////////////////
    cv::Scalar upper_blue(179,255,255);
    cv::inRange(hsv, lower_blue, upper_blue, mask_blue);

    // 形态学操作：闭运算填充蓝色区域内部的白色箭头造成的空洞，连接断开的区域
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(mask_blue, mask_blue, cv::MORPH_CLOSE, kernel);
    // 稍微腐蚀一点去掉边缘噪声
    // cv::erode(mask_blue, mask_blue, kernel);

    if (debug) cv::imshow("Debug: Blue Mask", mask_blue);

    // 2. 寻找最大的蓝色轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_blue, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) return 0;

    // 找到面积最大的轮廓
    auto max_it = std::max_element(contours.begin(), contours.end(),
                                   [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                                       return cv::contourArea(a) < cv::contourArea(b);
                                   });
    
    double area = cv::contourArea(*max_it);
    if (area < 350) return 0; // 面积太小认为是噪声

    // 3. 多边形拟合寻找4个顶点
    std::vector<cv::Point> approx;
    double epsilon = 0.04 * cv::arcLength(*max_it, true);
    cv::approxPolyDP(*max_it, approx, epsilon, true);

    // 如果拟合出来的不是4边形（可能是梯形畸变），或者形状太奇怪，则尝试用凸包或者外接矩形修正
    // 为了简单鲁棒，如果点数!=4，我们直接退化为使用最小外接矩形
    std::vector<cv::Point2f> src_pts(4);
    if (approx.size() == 4) {
        orderPoints(approx, src_pts);
    } else {
        // 鲁棒性降级方案：使用最小外接矩形（带旋转）
        cv::RotatedRect rect = cv::minAreaRect(*max_it);
        cv::Point2f v[4];
        rect.points(v);
        std::vector<cv::Point> rect_pts;
        for(int i=0; i<4; i++) rect_pts.push_back(v[i]);
        orderPoints(rect_pts, src_pts);
    }

    // 4. 透视变换 (Warp Perspective)
    // 将蓝色区域强行拉伸成 200x200 的正方形
    int card_size = 200;
    std::vector<cv::Point2f> dst_pts = {
        cv::Point2f(0, 0),
        cv::Point2f(card_size - 1, 0),
        cv::Point2f(card_size - 1, card_size - 1),
        cv::Point2f(0, card_size - 1)
    };

    cv::Mat M = cv::getPerspectiveTransform(src_pts, dst_pts);
    cv::Mat warped_card;
    cv::warpPerspective(frame, warped_card, M, cv::Size(card_size, card_size));

    if (debug) cv::imshow("Debug: Warped Card", warped_card);

    // 5. 在拉伸后的图中提取白色箭头
    cv::Mat warped_hsv;
    cv::cvtColor(warped_card, warped_hsv, cv::COLOR_BGR2HSV);
    
    // 白色阈值：低饱和度，高亮度
    // 注意：在蓝色底板上，箭头边缘可能会偏蓝，S设宽松点
    cv::inRange(warped_hsv, cv::Scalar(0, 0, 245), cv::Scalar(158, 50, 255), mask_white);  /////////////////////////////////////////////////////////////////////////////////////////

    if (debug) cv::imshow("Debug: Arrow Mask", mask_white);

    // 6. 判别方向 (基于上半部分的重心/像素分布)
    // 逻辑：转弯箭头的“头部”决定了方向。头部通常在图像的上半部分。
    // 我们只统计上半部分的像素。
    
    cv::Rect roi_top(0, 0, card_size, card_size / 2); // 取上半截
    cv::Mat mask_top = mask_white(roi_top);

    // 计算上半部分的矩 (Moments)
    cv::Moments m = cv::moments(mask_top, true);
    
    if (m.m00 == 0) return 0; // 没看见白色

    int cx = static_cast<int>(m.m10 / m.m00); // 重心X坐标

    // 图像宽度是200，中心是100
    // 如果重心在左边 (<100)，说明箭头头指向左边 -> 左转
    // 如果重心在右边 (>100)，说明箭头头指向右边 -> 右转
    // 加上一个阈值死区防止抖动 (例如 +/- 10)
    
    int center_threshold = card_size / 2; 
    int dead_zone = 10; 

    int result = 0;
    if (cx < center_threshold - dead_zone) {
        result = -1; // 左转
        if(debug) std::cout << ">>> LEFT Turn Detected (cx: " << cx << ")" << std::endl;
    } else if (cx > center_threshold + dead_zone) {
        result = 1;  // 右转
        if(debug) std::cout << ">>> RIGHT Turn Detected (cx: " << cx << ")" << std::endl;
    }

    return result;
}

/**
 * @brief 停车标志检测 (A/B 识别)
 * @param frame 输入图像
 * @param debug 是否开启调试窗口
 * @return int 0:未检测到, 1:检测到'A', 2:检测到'B'
 */
int detectStopSign(cv::Mat& frame, bool debug = false) {
    // if (frame.empty()) return 0;

    // // 1. 寻找蓝色底板 (复用之前的逻辑)
    // cv::Mat hsv, mask_blue;
    // cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    
    // // 蓝色阈值 (根据实际环境调整)
    // cv::Scalar lower_blue(90, 35,197);
    // cv::Scalar upper_blue(179,255,255);
    // cv::inRange(hsv, lower_blue, upper_blue, mask_blue);
    
    // // 形态学闭运算，填补蓝色区域空洞
    // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    // cv::morphologyEx(mask_blue, mask_blue, cv::MORPH_CLOSE, kernel);

    // // 寻找最大蓝色轮廓
    // std::vector<std::vector<cv::Point>> contours;
    // cv::findContours(mask_blue, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // if (contours.empty()) return 0;

    // auto max_it = std::max_element(contours.begin(), contours.end(),
    //                                [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
    //                                    return cv::contourArea(a) < cv::contourArea(b);
    //                                });

    // if (cv::contourArea(*max_it) < 1000) return 0; // 面积过小忽略

    // // 2. 透视变换 (将蓝色区域拉伸为正方形)
    // std::vector<cv::Point> approx;
    // double epsilon = 0.04 * cv::arcLength(*max_it, true);
    // cv::approxPolyDP(*max_it, approx, epsilon, true);

    // std::vector<cv::Point2f> src_pts(4);
    // if (approx.size() == 4) {
    //     orderPoints(approx, src_pts);
    // } else {
    //     cv::RotatedRect rect = cv::minAreaRect(*max_it);
    //     cv::Point2f v[4];
    //     rect.points(v);
    //     std::vector<cv::Point> rect_pts;
    //     for(int i=0; i<4; i++) rect_pts.push_back(v[i]);
    //     orderPoints(rect_pts, src_pts);
    // }

    // int card_h = 100; // 不需要太大，100x100足够识别
    // int card_w = 100;
    // std::vector<cv::Point2f> dst_pts = {
    //     cv::Point2f(0, 0),
    //     cv::Point2f(card_w - 1, 0),
    //     cv::Point2f(card_w - 1, card_h - 1),
    //     cv::Point2f(0, card_h - 1)
    // };

    // cv::Mat M = cv::getPerspectiveTransform(src_pts, dst_pts);
    // cv::Mat warped_card;
    // cv::warpPerspective(frame, warped_card, M, cv::Size(card_w, card_h));

    // if (debug) cv::imshow("Debug: StopSign Warped", warped_card);

    // // 3. 提取白色字符
    // cv::Mat gray_card, mask_white;
    // cv::cvtColor(warped_card, gray_card, cv::COLOR_BGR2GRAY);
    // // 使用Otsu自动阈值，或者固定阈值提取白色
    // cv::threshold(gray_card, mask_white, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    
    // // 再次清理噪点
    // cv::morphologyEx(mask_white, mask_white, cv::MORPH_OPEN, kernel);

    // // 4. 提取字符的最小外接矩形 (ROI)
    // // 这一步非常关键：它可以去除蓝色边缘的留白，让我们只分析字母本身
    // std::vector<std::vector<cv::Point>> char_contours;
    // cv::findContours(mask_white, char_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // if (char_contours.empty()) return 0;
    
    // // 找最大的白色轮廓（即字母）
    // auto max_char = std::max_element(char_contours.begin(), char_contours.end(),
    //                                  [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
    //                                      return cv::contourArea(a) < cv::contourArea(b);
    //                                  });
    
    // cv::Rect char_rect = cv::boundingRect(*max_char);
    
    // // 安全检查：如果识别到的白块太小（比如只是反光噪点），直接返回
    // if (char_rect.height < 20 || char_rect.width < 10) return 0;

    // // 裁剪出字母图像
    // cv::Mat roi = mask_white(char_rect);
    // if (debug) cv::imshow("Debug: Character ROI", roi);

    // // 5. 核心算法：计算 上半部分宽度 vs 下半部分宽度
    // // A: 上窄下宽
    // // B: 上下同宽
    
    // int h = roi.rows;
    // int w = roi.cols;
    
    // // 统计 上部区域(Top 30%) 和 下部区域(Bottom 30%) 的白色像素平均宽度
    // // 这里的“宽度”我们简化为：该行最左白点到最右白点的距离
    
    // double sum_top_width = 0;
    // int top_lines = 0;
    // double sum_bottom_width = 0;
    // int bottom_lines = 0;

    // for (int r = 0; r < h; r++) {
    //     const uchar* row_ptr = roi.ptr<uchar>(r);
    //     int first = -1;
    //     int last = -1;
        
    //     // 找这一行的最左和最右白点
    //     for (int c = 0; c < w; c++) {
    //         if (row_ptr[c] == 255) {
    //             if (first == -1) first = c;
    //             last = c;
    //         }
    //     }

    //     if (first != -1) {
    //         int line_width = last - first;
            
    //         // 如果是顶部 1/3
    //         if (r < h / 4) {
    //             sum_top_width += line_width;
    //             top_lines++;
    //         }
    //         // 如果是底部 1/3
    //         else if (r > h * 2 / 3) {
    //             sum_bottom_width += line_width;
    //             bottom_lines++;
    //         }
    //     }
    // }
    
    // // 防止除以0
    // if (top_lines == 0 || bottom_lines == 0) return 0;

    // double avg_top_width = sum_top_width / top_lines;
    // double avg_bottom_width = sum_bottom_width / bottom_lines;
    
    // // 计算比率
    // double ratio = avg_top_width / avg_bottom_width;

    // if (debug) {
    //     std::cout << "Top Width: " << avg_top_width 
    //               << " Bottom Width: " << avg_bottom_width 
    //               << " Ratio: " << ratio << std::endl;
    // }

    // // 6. 判别逻辑
    // // 'A' 的顶部很尖，平均宽度很小，底部很宽。Ratio 通常在 0.3 ~ 0.6 之间。
    // // 'B' 的顶部和底部差不多宽。Ratio 通常在 0.85 ~ 1.1 之间。
    // // 我们取一个中间阈值，比如 0.75

    // if (ratio < 0.75) {
    //     if (debug) std::cout << ">>> Detected: A" << std::endl;
    //     return 1; // A
    // } else {
    //     if (debug) std::cout << ">>> Detected: B" << std::endl;
    //     return 2; // B
    // }
    //******************************方案2******************************** */
    // if (frame.empty()) return 0;

    // // ================= 1. 蓝色区域提取与矫正 (保持不变) =================
    // cv::Mat hsv, mask_blue;
    // cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    
    // // 蓝色阈值 (根据环境微调)
    // cv::Scalar lower_blue(90, 35,197);
    // cv::Scalar upper_blue(179,255,255);
    // cv::inRange(hsv, lower_blue, upper_blue, mask_blue);
    
    // // 闭运算填补空洞
    // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    // cv::morphologyEx(mask_blue, mask_blue, cv::MORPH_CLOSE, kernel);

    // // 找最大蓝色轮廓
    // std::vector<std::vector<cv::Point>> contours;
    // cv::findContours(mask_blue, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // if (contours.empty()) return 0;

    // auto max_it = std::max_element(contours.begin(), contours.end(),
    //                                [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
    //                                    return cv::contourArea(a) < cv::contourArea(b);
    //                                });

    // if (cv::contourArea(*max_it) < 800) return 0; // 面积太小过滤

    // // 透视变换矫正
    // std::vector<cv::Point> approx;
    // double epsilon = 0.04 * cv::arcLength(*max_it, true);
    // cv::approxPolyDP(*max_it, approx, epsilon, true);

    // std::vector<cv::Point2f> src_pts(4);
    // if (approx.size() == 4) {
    //     orderPoints(approx, src_pts);
    // } else {
    //     cv::RotatedRect rect = cv::minAreaRect(*max_it);
    //     cv::Point2f v[4];
    //     rect.points(v);
    //     std::vector<cv::Point> rect_pts;
    //     for(int i=0; i<4; i++) rect_pts.push_back(v[i]);
    //     orderPoints(rect_pts, src_pts);
    // }

    // // 矫正为 100x100 的正方形
    // int card_size = 100;
    // std::vector<cv::Point2f> dst_pts = {
    //     cv::Point2f(0, 0), cv::Point2f(card_size-1, 0),
    //     cv::Point2f(card_size-1, card_size-1), cv::Point2f(0, card_size-1)
    // };
    // cv::Mat M = cv::getPerspectiveTransform(src_pts, dst_pts);
    // cv::Mat warped_card;
    // cv::warpPerspective(frame, warped_card, M, cv::Size(card_size, card_size));

    // //if (debug) cv::imshow("Debug: 1. Warped", warped_card);

    // // ================= 2. 精准字符提取 =================
    // cv::Mat gray_card, mask_white;
    // cv::cvtColor(warped_card, gray_card, cv::COLOR_BGR2GRAY);
    
    // // 使用 Otsu 二值化，自动适应光照变化
    // cv::threshold(gray_card, mask_white, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    
    // // 清理噪点
    // cv::morphologyEx(mask_white, mask_white, cv::MORPH_OPEN, kernel);

    // // 提取字母的 ROI (Region of Interest)
    // std::vector<std::vector<cv::Point>> char_contours;
    // cv::findContours(mask_white, char_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // if (char_contours.empty()) return 0;

    // auto max_char = std::max_element(char_contours.begin(), char_contours.end(),
    //                                  [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
    //                                      return cv::contourArea(a) < cv::contourArea(b);
    //                                  });
    
    // cv::Rect box = cv::boundingRect(*max_char);
    
    // // [关键过滤]：长宽比检查
    // // A和B的长宽比通常接近，大约 0.6 ~ 1.2 之间
    // // 如果识别到一个扁长的条或者细长的线，那肯定不是字母，直接过滤
    // float aspect_ratio = (float)box.width / (float)box.height;
    // if (box.height < 20 || aspect_ratio < 0.3 || aspect_ratio > 2.0) {
    //     if (debug) std::cout << "Ignored noise, aspect ratio: " << aspect_ratio << std::endl;
    //     return 0;
    // }

    // // 裁剪出字母，并给它加一点边框（padding），防止resize时边缘特征丢失
    // int pad = 2;
    // box.x = std::max(0, box.x - pad);
    // box.y = std::max(0, box.y - pad);
    // box.width = std::min(mask_white.cols - box.x, box.width + 2*pad);
    // box.height = std::min(mask_white.rows - box.y, box.height + 2*pad);
    
    // cv::Mat roi = mask_white(box);

    // // ================= 3. 核心识别算法：归一化网格检测 =================
    
    // // 将字母强制缩放到 20x20 像素
    // // 这一步让我们可以忽略字母的大小、远近，只看结构
    // cv::Mat norm_roi;
    // cv::resize(roi, norm_roi, cv::Size(20, 20));
    
    // // 再次二值化确保只有黑白（0或255）
    // cv::threshold(norm_roi, norm_roi, 127, 255, cv::THRESH_BINARY);

    // if (debug) {
    //     cv::Mat debug_roi;
    //     cv::resize(norm_roi, debug_roi, cv::Size(200, 200), 0, 0, cv::INTER_NEAREST);
    //     //cv::imshow("Debug: 2. Normalized Char", debug_roi);
    // }

    // // 定义两个关键检测区域 (基于20x20的图像)
    
    // // Zone 1: 左上角 (Top-Left Corner)
    // // 区域：x(0~7), y(0~7)
    // // 'A' 在这里是黑的 (斜边之外)
    // // 'B' 在这里是白的 (竖线+横杠)
    // cv::Mat zone_tl = norm_roi(cv::Rect(0, 0, 6, 6));
    // int pixel_tl = cv::countNonZero(zone_tl);
    
    // // Zone 2: 底部中间 (Bottom-Center)
    // // 区域：x(7~13), y(14~20) -> 也就是两腿之间
    // // 'A' 在这里是黑的 (两腿之间的空隙)
    // // 'B' 在这里是白的 (底部横杠)
    // cv::Mat zone_bc = norm_roi(cv::Rect(7, 14, 6, 6));
    // int pixel_bc = cv::countNonZero(zone_bc);

    // // ================= 4. 逻辑判决 =================
    
    // // 阈值判定：
    // // Zone 1 (8x8=64像素): 如果超过 20 个白点，说明这里有内容
    // // Zone 2 (6x6=36像素): 如果超过 15 个白点，说明这里有内容
    
    // bool is_tl_filled = pixel_tl > 16;
    // bool is_bc_filled = pixel_bc > 13;

    // if (debug) {
    //     std::cout << "Pixel TL: " << pixel_tl << " (Filled? " << is_tl_filled << ")" << std::endl;
    //     std::cout << "Pixel BC: " << pixel_bc << " (Filled? " << is_bc_filled << ")" << std::endl;
    // }

    // // B 的特征：左上角满 + 底部中间满
    // if (is_tl_filled && is_bc_filled) {
    //     if (debug) std::cout << ">>> Result: B" << std::endl;
    //     return 2;
    // }
    // // A 的特征：左上角空 + 底部中间空 (或者只有一点点噪点)
    // // 只要左上角是空的，大概率就是 A
    // else if (!is_tl_filled) {
    //     if (debug) std::cout << ">>> Result: A" << std::endl;
    //     return 1;
    // }
    
    // // 如果情况不明（比如左上角满但底部空，可能是P？），为了鲁棒性，
    // // 我们可以倾向于认为它是B（因为A的左上角极难被填满，除非严重误识别）
    // // 或者返回0表示未识别。这里为了赛道容错，倾向于最接近的特征。
    // if (is_tl_filled && !is_bc_filled) {
    //      // 这种情况极少见，可能是 'B' 的底部反光看不清，或者 'A' 倒了
    //      // 可以根据赛道实际情况修改，这里暂定为 0 或 2
    //      return 0; 
    // }

    // return 0;
    /***************************************************方案三*****************************************************************/
    /**
 * @brief 极简高鲁棒性 A/B 识别 (基于拓扑与凸包特征)
 * 不需要透视变换，不依赖角度，抗干扰能力极强
 * @param frame 输入图像
 * @param debug 是否开启调试窗口
 * @return int 0:未检测到, 1:检测到'A', 2:检测到'B'
 */
    if (frame.empty()) return 0;

    // ================= 1. 快速定位蓝色区域 =================
    cv::Mat hsv, mask_blue;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    
    // 蓝色阈值 (宽松一点，保证能圈住底板)
    cv::Scalar lower_blue(90, 35,197);     /////////////////////////////////////////////////////////////////////////////////////////
    cv::Scalar upper_blue(179,255,255);
    cv::inRange(hsv, lower_blue, upper_blue, mask_blue);
    
    // 形态学闭操作：把蓝色底板连成一片
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
    cv::morphologyEx(mask_blue, mask_blue, cv::MORPH_CLOSE, kernel);

    // 找蓝色外轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_blue, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return 0;

    // 找最大的蓝色块
    auto max_it = std::max_element(contours.begin(), contours.end(),
                                   [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                                       return cv::contourArea(a) < cv::contourArea(b);
                                   });
    
    // 面积太小直接忽略
    if (cv::contourArea(*max_it) < 1000) return 0;

    // 获取蓝色区域的边界框 (ROI)
    cv::Rect blue_rect = cv::boundingRect(*max_it);
    
    // 安全边界检查，防止越界
    int padding = 5;
    blue_rect.x = std::max(0, blue_rect.x + padding);
    blue_rect.y = std::max(0, blue_rect.y + padding);
    blue_rect.width = std::min(frame.cols - blue_rect.x - padding, blue_rect.width - 2*padding);
    blue_rect.height = std::min(frame.rows - blue_rect.y - padding, blue_rect.height - 2*padding);
    
    if (blue_rect.width <= 0 || blue_rect.height <= 0) return 0;

    // 截取 ROI，只在蓝色底板范围内找白色
    cv::Mat roi = frame(blue_rect);
    
    // ================= 2. 提取白色字符 =================
    cv::Mat roi_hsv, mask_white;
    cv::cvtColor(roi, roi_hsv, cv::COLOR_BGR2HSV);
    
    // 白色阈值：低饱和度，高亮度 (适应蓝色背景上的白色)
    cv::inRange(roi_hsv, cv::Scalar(0, 0, 160), cv::Scalar(180, 60, 255), mask_white);  //////////////////////////////////////////////////////////////////////

    // 稍微膨胀一点，确保孔洞闭合，且噪点不会断开字符
    cv::Mat kernel_char = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask_white, mask_white, cv::MORPH_CLOSE, kernel_char);

    if (debug) cv::imshow("Debug: White Mask", mask_white);

    // ================= 3. 拓扑特征分析 (关键步骤) =================
    // 使用 RETR_TREE 模式，它可以建立轮廓的层级关系 (父子关系)
    // 这样我们就能数出这个轮廓里面有几个"洞"
    std::vector<std::vector<cv::Point>> char_contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask_white, char_contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    if (char_contours.empty()) return 0;

    // 遍历所有轮廓，找到面积最大的那个（认为是字母本身）
    int max_idx = -1;
    double max_area = 0;
    for (size_t i = 0; i < char_contours.size(); i++) {
        // 过滤掉太小的噪点
        double area = cv::contourArea(char_contours[i]);
        // 还需要确保这个轮廓有父轮廓是-1（说明它是最外层，不是孔洞本身）或者它是最大的
        if (area > max_area) {
            max_area = area;
            max_idx = i;
        }
    }

    if (max_idx == -1 || max_area < 200) return 0; // 没找到有效字符

    // === 特征 1: 孔洞数量 (Holes) ===
    int holes = 0;
    // hierarchy[i] = [Next, Previous, First_Child, Parent]
    // 我们从最大的轮廓 (max_idx) 开始，看它的 First_Child (第一个子轮廓/孔洞)
    int child_idx = hierarchy[max_idx][2]; 
    while (child_idx != -1) {
        double hole_area = cv::contourArea(char_contours[child_idx]);
        // 只有当孔洞面积足够大才算，防止噪点干扰
        if (hole_area > 20) { 
            holes++;
        }
        child_idx = hierarchy[child_idx][0]; // 找下一个兄弟孔洞
    }

    // === 特征 2: 实心度 (Solidity) ===
    // Solidity = 轮廓面积 / 凸包面积
    std::vector<cv::Point> hull;
    cv::convexHull(char_contours[max_idx], hull);
    double hull_area = cv::contourArea(hull);
    // 防止除0错误
    if (hull_area <= 0) hull_area = max_area; 
    
    double solidity = max_area / hull_area;

    if (debug) {
        std::cout << ">>> Holes: " << holes << ", Solidity: " << solidity << std::endl;
        // 画出识别到的字符轮廓
        cv::Mat debug_img = roi.clone();
        cv::drawContours(debug_img, char_contours, max_idx, cv::Scalar(0, 0, 255), 2);
        cv::imshow("Debug: Result", debug_img);
    }

    // ================= 4. 综合逻辑判决 =================
    
    // 判决 B:
    // 强特征：有2个洞 (直接确诊)
    // 弱特征：只有1个洞(可能光照太强堵住了一个) 或者 0个洞(太远看不清)，但是实心度极高(像块砖头)
    if (holes >= 2) {
        return 2; // B (Two holes)
    }
    if (solidity > 0.88) {
        // 实心度极高，说明几乎填满了凸包，大概率是B (即使没看到洞)
        return 2; // B (Solid block)
    }

    // 判决 A:
    // 强特征：有1个洞
    // 弱特征：0个洞，但实心度适中 (因为A是三角形，凸包是三角形，轮廓也是三角形，实心度比L型的箭头高，比B低)
    // 注意：A的实心度通常在 0.7~0.85 之间 (取决于字体粗细)
    if (holes == 1) {
        return 1; // A (One hole)
    }
    if (holes == 0 && solidity > 0.65 && solidity <= 0.88) {
        // 没有洞，且不是特别实心，也不是特别空心，认为是A
        return 1; // A
    }

    // 补充：如果需要兼容箭头识别
    // 箭头通常 solidity < 0.6 (因为凹陷很大) 且 holes == 0
    
    return 0; // 无法确定
}
// 蓝色卡片检测函数
// 功能：在预处理后的掩码图像中寻找蓝色卡片，并在检测到目标时进行标记
void blue_card_find(void)
{
    vector<vector<Point>> contours;  // 用于存储找到的所有轮廓
    vector<Vec4i> hierarcy;         // 轮廓的层次结构信息
    // 从掩码图像中寻找轮廓，只检索最外层轮廓
    findContours(mask, contours, hierarcy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    
    // 如果找到轮廓
    if (contours.size() > 0)
    {
        // 按轮廓面积降序排序
        sort(contours.begin(), contours.end(), Contour_Area);
        vector<vector<Point>> newContours;  // 存储筛选后的轮廓
        
        // 遍历所有轮廓，筛选出y坐标在90-160之间的轮廓
        for (const vector<Point> &contour : contours)
        {
            Point2f center;  // 轮廓最小包围圆的圆心
            float radius;    // 轮廓最小包围圆的半径
            minEnclosingCircle(contour, center, radius);
            
            // 筛选出位于特定垂直范围内的轮廓（排除图像顶部和底部的干扰）
            if (center.y > 90 && center.y < 160)
            {
                newContours.push_back(contour);
            }
        }

        // 更新轮廓列表为筛选后的结果
        contours = newContours;

        // 如果筛选后仍有轮廓
        if (contours.size() > 0)
        {
            // 检查最大轮廓的面积是否大于阈值（500）
            if (contourArea(contours[0]) > 500)
            {
                cout << "find biggest blue" << endl;  // 输出调试信息
                Point2f center;  // 最大轮廓的圆心
                float radius;    // 最大轮廓的半径
                minEnclosingCircle(contours[0], center, radius);
                // 在原始帧上绘制绿色的包围圆（线宽2像素）
                circle(frame, center, static_cast<int>(radius), Scalar(0, 255, 0), 2);
                find_first = 1;  // 设置找到蓝色卡片的标志位
            }
            else
            {
                cout << "not found blue" << endl;  // 输出未找到蓝色卡片的调试信息
            }
        }
    }
    else
    {
        cout << "not found blue" << endl;  // 输出未找到任何轮廓的调试信息
    }
}
// 蓝色卡片移除检测函数
// 功能：检查蓝色卡片是否已经被移除，如果移除则更新状态标志
void blue_card_remove(void)
{
    cout << "entry move blue process" << endl;  // 输出调试信息，表示进入蓝色卡片移动检测流程
    vector<vector<Point>> contours;  // 用于存储找到的所有轮廓
    vector<Vec4i> hierarcy;         // 轮廓的层次结构信息
    
    // 从掩码图像中寻找轮廓，只检索最外层轮廓
    findContours(mask, contours, hierarcy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    
    // 如果找到轮廓
    if (contours.size() > 0)
    {
        // 按轮廓面积降序排序
        sort(contours.begin(), contours.end(), Contour_Area);
        vector<vector<Point>> newContours;  // 存储筛选后的轮廓
        
        // 遍历所有轮廓，筛选出y坐标在90-160之间的轮廓
        for (const vector<Point> &contour : contours)
        {
            Point2f center;  // 轮廓最小包围圆的圆心
            float radius;    // 轮廓最小包围圆的半径
            minEnclosingCircle(contour, center, radius);
            
            // 筛选出位于特定垂直范围内的轮廓（与blue_card_find函数使用相同的筛选条件）
            if (center.y > 90 && center.y < 160)
            {
                newContours.push_back(contour);
            }
        }

        // 更新轮廓列表为筛选后的结果
        contours = newContours;

        // 如果筛选后没有轮廓，说明蓝色卡片已被移除
        if (contours.size() == 0)
        {
            Running_State = START;  // 重置开始标志位
            cout << "move" << endl;  // 输出调试信息
            sleep(2);  // 等待2秒，可能用于确保卡片完全移开
        }
    }
    else
    {
        // 如果一开始就没有找到轮廓，说明蓝色卡片已被移除
        Running_State = START;  // 重置开始标志位
        cout << "蓝色挡板移开" << endl;  // 输出调试信息
        sleep(2);  // 等待2秒，可能用于确保卡片完全移开
    }
}

/** 
 * @brief 专用停车线检测函数 
 * @param frame 输入图像 
 * @param debug 是否显示调试窗口 
 * @return int 返回最近横向黄线的 Y 坐标（值越大代表离车越近）。如果没找到，返回 -1。 
 */ 
int find_parking_stop_line(cv::Mat frame, bool debug = false) 
{ 
    // 1. 预处理与颜色分割 
    cv::Mat hsv, mask_yellow;  cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV); 
    
    // 黄色阈值 (根据你的赛道环境微调) 
    // H: 15-35 是典型的黄色范围 
    // S: 43-255 去除灰白 
    // V: 46-255 去除暗黑 
    cv::Scalar lower_yellow(15, 43, 46); 
    cv::Scalar upper_yellow(35, 255, 255); 
    cv::inRange(hsv, lower_yellow, upper_yellow, mask_yellow); 
    
    // 2. ROI 裁剪 (只看车头前方的一块区域) 
    // 假设图像 320x240，我们只关心下半部分，避开远处干扰 
    int roi_y = 120;  
    int roi_h = 120; 
    cv::Rect roi_rect(0, roi_y, frame.cols, roi_h); 
    cv::Mat roi = mask_yellow(roi_rect); 
    
    // 3. 形态学操作 (连接断裂的黄线) 
    // 使用横向的长方形核，因为我们要找的是横线 
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 3)); 
    cv::morphologyEx(roi, roi, cv::MORPH_CLOSE, kernel); 
    
    // 4. 寻找轮廓 
    std::vector<std::vector<cv::Point>> contours; 
    cv::findContours(roi, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); 
    
    int max_y = -1; // 记录最靠下的线的底部 Y 坐标 
    
    cv::Mat debug_img; 
    if (debug) debug_img = frame.clone(); 
    
    for (const auto& cnt : contours) 
    { 
    double area = cv::contourArea(cnt); 
    if (area < 200) continue; // 面积太小认为是噪点 
    
    cv::Rect box = cv::boundingRect(cnt); 
    float aspect_ratio = (float)box.width / (float)box.height; 
    
    // 5. 筛选条件：必须是扁长的横线 
    // 宽高比 > 2.0 (越宽越好)，且宽度要占据一定比例 
    if (aspect_ratio > 2.5 && box.width > 60)  
    {  // 还原回原图坐标系 
    int current_bottom_y = box.y + box.height + roi_y; 
    
    // 我们只关心最靠近车头的那条线（Y坐标最大） 
    if (current_bottom_y > max_y) { 
    max_y = current_bottom_y; 
    } 
    
    if (debug) { 
    cv::rectangle(debug_img, cv::Rect(box.x, box.y + roi_y, box.width, box.height), 
    cv::Scalar(0, 0, 255), 2); 
    } 
    } 
    } 
    
    if (debug) { 
    cv::imshow("Yellow Line Debug", debug_img); 
    cv::waitKey(1); 
    } 
    
    return max_y; 
}

// 包含必要的头文件
#include <iostream>
#include <chrono>      // 用于高精度计时
#include <opencv2/opencv.hpp> // OpenCV库，用于图像处理
#include <thread>      // 用于多线程操作

using namespace cv;
using namespace std;



// 主函数
int main() {
	// 清空图片文件夹
    std::string clear_command = "rm -rf " + save_dir + "/*";
    int clear_result = system(clear_command.c_str());
    if (clear_result == 0) {
        std::cout << "已清空图片文件夹: " << save_dir << std::endl;
    } else {
        std::cout << "清空文件夹失败: " << save_dir << std::endl;
    }
    // 启动输入线程，使用detach使其在后台运行
    thread(input_thread).detach(); 

    // 初始化变量
    bool visual_flag = false;  // 可视化标志位
    int yellow_num_edge = 0;   // 黄色边缘数量计数
    int yellow_num_hsv = 0;    // 基于HSV的黄色区域数量计数
    VideoCapture capture;      // 摄像头捕获对象
    
    // 初始化GPIO和舵机控制
    gpioTerminate();           // 终止之前可能存在的GPIO连接
    servo_motor_pwmInit();     // 初始化舵机PWM控制
    
    // 打开摄像头（设备ID为0）
    capture.open(0);
    if (!capture.isOpened())
    {
        cout << "Can not open video file!" << endl;  // 输出错误信息
        system("pause");  // 暂停程序执行
        return -1;        // 返回错误代码
    }
    
    // 输出摄像头参数信息
    std::cout << "FPS: " << capture.get(cv::CAP_PROP_FPS) << std::endl;
    std::cout << "Frame Width: " << capture.get(cv::CAP_PROP_FRAME_WIDTH) << std::endl;
    std::cout << "Frame Height: " << capture.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;

    // 设置摄像头参数
    capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G')); // 设置视频编码格式
    capture.set(cv::CAP_PROP_FPS, 30);       // 设置帧率为30fps
    capture.set(cv::CAP_PROP_FRAME_WIDTH, 320);  // 设置图像宽度为320
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, 240); // 设置图像高度为240
    
    // 记录开始时间用于计时
   auto start_time = std::chrono::high_resolution_clock::now();
    
    // 主循环，逐帧读取摄像头图像
    while (capture.read(frame))
    {
        // 图像去畸变处理
        frame = undistort(frame);
        
        // 模式1：检测蓝色卡片模式（READY状态）
        if (Running_State == READY)
        {
            Mat frame_a = frame.clone();  // 克隆当前帧
            cvtColor(frame_a, change_frame, 40);  // 将BGR图像转换为HSV色彩空间（40表示COLOR_BGR2HSV）
            
            // 设置蓝色的HSV阈值范围
            //Scalar scalarl = Scalar(100, 43, 46);   // 蓝色的低阈值
            //Scalar scalarH = Scalar(124, 255, 255); // 蓝色的高阈值   室外

            Scalar scalarl = Scalar(67, 98, 134);   // 蓝色的低阈值
            Scalar scalarH = Scalar(179, 255, 255); // 蓝色的高阈值     室内
            inRange(change_frame, scalarl, scalarH, mask);  // 根据阈值创建二值掩码
            
            // 创建形态学操作的结构元素并进行形态学处理
            Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));  // 5x5矩形结构元素
            morphologyEx(mask, mask, MORPH_OPEN, kernel);  // 开操作（先腐蚀后膨胀），用于去除噪声
            morphologyEx(mask, mask, MORPH_CLOSE, kernel); // 闭操作（先膨胀后腐蚀），用于填充小洞
            
            // 手动过滤掉不在指定垂直范围内的像素
            for (int y = 0; y < mask.rows; y++)
            {
                for (int x = 0; x < mask.cols; x++)
                {
                    if (y < 90 || y > 160)  // 只保留y坐标在90-160之间的区域
                    {
                        mask.at<uchar>(y, x) = 0;
                    }
                }
            }
            
            // 根据标志位决定调用哪个函数
            if (find_first == 0)  // 如果尚未找到蓝色卡片
            {
                blue_card_find();  // 查找蓝色卡片
            }
            else  // 如果已经找到过蓝色卡片
            {
                blue_card_remove();  // 检查蓝色卡片是否被移除
            }
        }
        else
        {
            Mat frame_a = frame.clone();  // 克隆当前帧
                
            // 蓝色检测预处理（与模式1相同）
            cvtColor(frame, change_frame, COLOR_BGR2HSV);  // 注意：用 frame，不是 frame_a

            // 1. CLAHE 增强亮度通道（解决阳光高光/阴影）
            Mat hsv_channels[3];
            split(change_frame, hsv_channels);
            Ptr<CLAHE> clahe = createCLAHE(3.0, Size(8,8));
            clahe->apply(hsv_channels[2], hsv_channels[2]);  // V通道增强
            merge(hsv_channels, 3, change_frame);

            // 2. 蓝色阈值（实测最稳）
            inRange(change_frame, Scalar(100, 43, 46), Scalar(124, 255, 255), mask);//////////////////////////////////////////////////////////////////

            // 3. 形态学：先轻开去噪，再强闭填洞
            Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5,5));
            morphologyEx(mask, mask, MORPH_OPEN,  kernel, Point(-1,-1), 1);   // 去小噪声
            morphologyEx(mask, mask, MORPH_CLOSE, kernel, Point(-1,-1), 3);   // 填阳光碎块
                
            // 图像预处理
            cv::cvtColor(frame, grayImage, cv::COLOR_BGR2GRAY);  // 转换为灰度图
            bin_image = ImagePreprocessing(frame);  // 图像预处理函数，可能包括二值化等操作
            auto end_time = std::chrono::high_resolution_clock::now(); // 声明并初始化end_time
            //cout << ".....!" << endl;  // 输出错误信息
            // 路径跟踪处理
           Tracking(bin_image);
            if (Running_State == START)  // 模式2：正常行驶模式（START状态）
            {
                blue_contours = blue_vertebral_barrel_find_all(mask, frame, false, 90);
                // 检查是否检测到蓝色锥桶轮廓
                if (blue_contours.size() != 0)
                {
                    // 输出调试信息
                    cout << "过滤后的点：center.x" << center_x << endl;
                    cout << "center.y " << center_y << endl;
                    cout << "left_line " << left_line[center_y - 108].x << endl;
                    cout << "right_line " << right_line[center_y - 108].x << endl; 
                    if(blue_check <= 3)//检测到蓝色锥桶小于3次
                    {
                        blue_check++;
                    
                        servo_pwm_now = servo_pd(172);  // 目标值设为166（正常行驶）
                        cout << "pwm" << servo_pwm_now << endl;
                        gpioPWM(13, speed_all);  // 正常速度
                        gpioPWM(12, servo_pwm_now);
                        motorSpeed = speed_all;                                    
                    }
                    else
                    {
//                        servo_pwm_now = servo_pd(190);  // 目标值设为197（靠左）
//                        std::cout << "靠左躲避" << endl;
//                        cout << "pwm" << servo_pwm_now << endl;
//                        gpioPWM(13, 9400);  // 设置电机速度
//                        gpioPWM(12, servo_pwm_now);  // 应用舵机控制
//                        motorSpeed = 9400;
					    std::cout << "开始避障" << endl;
	                    TimeRecord_Start_1 = std::chrono::high_resolution_clock::now();
	                    servo_pwm_now = 780;  // 向左避障
	                    cout << "pwm" << servo_pwm_now << endl;
	                    gpioPWM(13, 9300);  // 正常速度              
	                    gpioPWM(12, servo_pwm_now);
	                    motorSpeed = 9300;
	                    while(1)
	                    {
	                        TimeRecord_End_1 = std::chrono::high_resolution_clock::now();
	                        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(TimeRecord_End_1 - TimeRecord_Start_1);
	                        if(duration.count() >= 460 )
	                        {
	                            break;
	                        }
	                    }
	                    TimeRecord_Start_1 = std::chrono::high_resolution_clock::now();
	                    servo_pwm_now = 920;  // 向右回正
	                    cout << "pwm" << servo_pwm_now << endl;
	                    gpioPWM(13, 9300);  // 正常速度              
	                    gpioPWM(12, servo_pwm_now);
	                    motorSpeed = 9300;
	                    while(1)
	                    {
	                        TimeRecord_End_1 = std::chrono::high_resolution_clock::now();
	                        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(TimeRecord_End_1 - TimeRecord_Start_1);
	                        if(duration.count() >= 400)
	                        {
	                            break;
	                        }
	                    }
	                        Running_State = FIND_CROSS;
	                 }
                }
                else
                {
                    servo_pwm_now = servo_pd(166 );  // 目标值设为160（正常行驶）166
                    cout << "pwm" << servo_pwm_now << endl;
                    gpioPWM(13, speed_all);  // 正常速度
                    gpioPWM(12, servo_pwm_now);
                    motorSpeed = speed_all;                   
                }

                std::cout << "正常行驶模式" << endl;
            }
            else if (Running_State == AVOID_CONE)  // 模式4：避开锥桶模式（AVOID_CONE状态）
            {
            	if(Keep_Avoid != 1)
	               {
	                blue_contours = blue_vertebral_barrel_find_all(mask, frame, false, 70);
	                // 检查是否检测到蓝色锥桶轮廓
	                if (blue_contours.size() != 0)
	                {
	                    // 输出调试信息
	                    cout << "过滤后的点：center.x" << center_x << endl;
	                    cout << "center.y " << center_y << endl;
	                    cout << "left_line " << left_line[center_y - 108].x << endl;
	                    cout << "right_line " << right_line[center_y - 108].x << endl; 
	
	                    servo_pwm_now = servo_pd(190);  // 目标值设为197（靠左）
	                    std::cout << "靠左躲避" << endl;
	                    cout << "pwm" << servo_pwm_now << endl;
	                    gpioPWM(13, 9400);  // 设置电机速度
	                    gpioPWM(12, servo_pwm_now);  // 应用舵机控制
	                    motorSpeed = 9400;

                	 }
	              }
                else 
                {
                    TimeRecord_End_1 = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(TimeRecord_End_1 - TimeRecord_Start_1);

                    servo_pwm_now = servo_pd(197);  // 目标值设为197（靠左）
                    cout << "pwm" << servo_pwm_now << endl;
                    gpioPWM(13, 9400);  // 设置电机速度
                    gpioPWM(12, servo_pwm_now);  // 应用舵机控制
                    motorSpeed = 9400;

                    std::cout << " 继续躲避" << duration.count() / 1000 << "s" << std::endl;

                    if(duration.count() > 3000)//持续5s退出
                    {                                           
                        std::cout << "躲避结束" << std::endl;
                        Running_State = FIND_CROSS;
                    }
                }
                std::cout << "避开锥桶模式" << endl;
            }
            else if (Running_State == FIND_CROSS)  // 模式3：十字路口检测模式（FIND_CROSS状态）
            {
                // 十字路口检测逻辑
                cross = crossroad(frame);  // 调用十字路口检测函数
                if(cross == 1)
                {
                    cout << "斑马线位置" << zebra_y << endl;//输出斑马线位置
                    if(cross_check <= 5)//检测到十字路口小于3次
                    {
                        //正常循迹
                        servo_pwm_now = servo_pd(172);  // 目标值设为160（正常行驶）166
                        cout << "pwm" << servo_pwm_now << endl;
                        gpioPWM(13, speed_all);  // 正常速度
                        gpioPWM(12, servo_pwm_now);
                        motorSpeed = speed_all;   
                        cross_check++;
                    }
                    else
                    {
                        std::cout << "检测到十字路口" << std::endl;
                        Running_State = CROSS_STOP;//切换到十字路口停止模式
                        // 刹车 电机停止
                        servo_pwm_now = Servo_middle;//864
                        cout << "pwm" << servo_pwm_now << endl;
                        gpioPWM(13, 10800);  
                        gpioPWM(12, servo_pwm_now);
                        motorSpeed = 10800;        
                    }
                }
                else
                {   
                    //正常循迹
                    servo_pwm_now = servo_pd(170);  // 目标值设为160（正常行驶）166
                    cout << "pwm" << servo_pwm_now << endl;
                    gpioPWM(13, speed_all);  // 正常速度
                    gpioPWM(12, servo_pwm_now);
                    motorSpeed = speed_all;          
                }
                std::cout << "十字路口检测模式" << endl;
            }
            else if (Running_State == CROSS_STOP)  // 模式5：十字路口停止模式（CROSS_STOP状态）
            {
            	
            	static int play_flag=1;
                // 十字路口停止逻辑
                servo_pwm_now = Servo_middle;//864
                cout << "pwm" << servo_pwm_now << endl;
                gpioPWM(13, 11000); // 刹车电机停止
                gpioPWM(12, servo_pwm_now);
                motorSpeed = 11000;    
                servo_motor_pwmInit();    

			if(play_flag==1)
			{
                // 播放声音提示
                std::cout << "播报语音" << endl;
                system("aplay /home/pi/5Gcode3/SDK/RaspberryPiSDK-master/Linux_voice_1.109/bin/nibuhuidawo.WAV");//确保文件存在，不存在需手动创建
                play_flag=0;
                gpioPWM(13, 10500);
			}


                //检测方向标识
                // 已经停车，检测变道箭头
                if(Left_check<3&&Right_check<3)//检测到变道箭头小于3次
                {
                	//gpioPWM(13, 10000);
                    int arrow = detectLaneChangeArrow2(frame, false);  // true 打开调试窗口//(0:无箭头 -1:左转 1:右转)
                    if (arrow != 0)  // 检测到有效箭头
                    {
                        if(arrow == -1)
                        {
                            Left_check++;
                        }
                        else if(arrow == 1)
                        {
                            Right_check++;
                        }
                    }
                    else std::cout << "未识别到标识牌" << endl;
                    
                }
                // 根据检测结果决定变道方向
                if(Left_check==3 || biandao == -1)
                {
                	//gpioPWM(13, 10000);
                    std::cout << "变道方向：左转" << std::endl;
                    change_direction = -1;//设置变道方向为左转
                    Running_State = LANE_CHANGE_1;//切换到变道1模式
                }
                else if(Right_check==3 || biandao == 1)
                {
                    std::cout << "变道方向：右转" << std::endl;
                    change_direction = 1;//设置变道方向为右转
                    Running_State = LANE_CHANGE_1;//切换到变道1模式
                }
                std::cout << "十字路口刹车模式" << endl;
            }
            else if (Running_State == LANE_CHANGE_1)  // 模式6：变道1模式（LANE_CHANGE_1状态）
            {
                // 变道1逻辑
                if(change_direction == 1)
                {
                    std::cout << "开始变道" << endl;
                    TimeRecord_Start_1 = std::chrono::high_resolution_clock::now();
                    servo_pwm_now = 960;  // 
                    cout << "pwm" << servo_pwm_now << endl;
                    gpioPWM(13, 9400);  // 正常速度                   
                    gpioPWM(12, servo_pwm_now);
                    motorSpeed = 9400;
                    while(1)
                    {
                        TimeRecord_End_1 = std::chrono::high_resolution_clock::now();
                        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(TimeRecord_End_1 - TimeRecord_Start_1);
                        if(duration.count() >= 1300)//***********************************************//
                        {
                            break;
                        }
                    }
                    TimeRecord_Start_1 = std::chrono::high_resolution_clock::now();
                    servo_pwm_now = 740;
                    cout << "pwm" << servo_pwm_now << endl;
                    gpioPWM(13, 9400);  // 正常速度
                    gpioPWM(12, servo_pwm_now);
                    motorSpeed = 9400;
                    while(1)
                    {
                        TimeRecord_End_1 = std::chrono::high_resolution_clock::now();
                        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(TimeRecord_End_1 - TimeRecord_Start_1);
                        if(duration.count() >= 800)
                        {
                            break;
                        }
                    }
                    servo_pwm_now = 858;//860  // 回正
                    cout << "pwm" << servo_pwm_now << endl;
                    gpioPWM(13, 9400);  // 正常速度
                    gpioPWM(12, servo_pwm_now);
                    motorSpeed = 9400;       
                    Running_State = FIND_CHANGE_LINE;
                    std::cout << "变道完成" << endl;
                }
                else if(change_direction == -1)
                {
                    std::cout << "开始变道" << endl;
                    TimeRecord_Start_1 = std::chrono::high_resolution_clock::now();
                    servo_pwm_now = 740;  // 
                    cout << "pwm" << servo_pwm_now << endl;
                    gpioPWM(13, 9400);  // 正常速度              
                    gpioPWM(12, servo_pwm_now);
                    motorSpeed = 9400;
                    while(1)
                    {
                        TimeRecord_End_1 = std::chrono::high_resolution_clock::now();
                        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(TimeRecord_End_1 - TimeRecord_Start_1);
                        if(duration.count() >= 1200)
                        {
                            break;
                        }
                    }
                    TimeRecord_Start_1 = std::chrono::high_resolution_clock::now();
                    servo_pwm_now = 960;
                    cout << "pwm" << servo_pwm_now << endl;
                    gpioPWM(13, 9400);  // 正常速度                  
                    gpioPWM(12, servo_pwm_now);
                    motorSpeed = 9400;
                    while(1)
                    {
                        TimeRecord_End_1 = std::chrono::high_resolution_clock::now();
                        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(TimeRecord_End_1 - TimeRecord_Start_1);
                        if(duration.count() >= 730)
                        {
                            break;
                        }
                    }
                    servo_pwm_now = 858;//860  // 回正
                    cout << "pwm" << servo_pwm_now << endl;
                    gpioPWM(13, 9400);  // 正常速度
                    gpioPWM(12, servo_pwm_now);
                    motorSpeed = 9400;
                    Running_State = FIND_CHANGE_LINE;//切换到变道2模式
                    std::cout << "变道完成" << endl;
                    blue_check = 0;
                }

                std::cout << "变道1模式" << endl;
            }
            else if(Running_State == FIND_CHANGE_LINE)
            {
                std::cout << "寻找变道线模式" << endl;
                servo_pwm_now = servo_pd(166);  // 正常寻迹
                cout << "pwm" << servo_pwm_now << endl;
                gpioPWM(13, 9400);  // 正常速度
                gpioPWM(12, servo_pwm_now);
                motorSpeed = 9400;  
                if(change_direction == 1)//右道
                {
                	 
//                    servo_pwm_now = servo_pd(166);  // 贴左线
//                    cout << "pwm" << servo_pwm_now << endl;
//                    gpioPWM(13, 9600);  // 正常速度
//                    gpioPWM(12, servo_pwm_now);
//                    motorSpeed = 9600;    
               	if(biandao2 == 1)
               	{
               		std::cout << "未识别" << endl;//向左变道
                       std::cout << "开始变道" << endl;
                       TimeRecord_Start_1 = std::chrono::high_resolution_clock::now();
                       servo_pwm_now = 740;  // 
                       cout << "pwm" << servo_pwm_now << endl;
                       gpioPWM(13, 9400);  // 正常速度
                       gpioPWM(12, servo_pwm_now);
                       motorSpeed = 9400;
                       while(1)
                       {
                           TimeRecord_End_1 = std::chrono::high_resolution_clock::now();
                           auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(TimeRecord_End_1 - TimeRecord_Start_1);
                           if(duration.count() >= 700)
                           {
                               break;
                           }
                       }
                       TimeRecord_Start_1 = std::chrono::high_resolution_clock::now();
                       servo_pwm_now = 960;
                       cout << "pwm" << servo_pwm_now << endl;
                       gpioPWM(13, 9400);  // 正常速度
                       gpioPWM(12, servo_pwm_now);
                       motorSpeed = 9400;
                       while(1)
                       {
                           TimeRecord_End_1 = std::chrono::high_resolution_clock::now();
                           auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(TimeRecord_End_1 - TimeRecord_Start_1);
                           if(duration.count() >= 800)
                           {
                               break;
                           }
                       }
                       servo_pwm_now = 858;//860  // 回正
                       cout << "pwm" << servo_pwm_now << endl;
                       gpioPWM(13, 9400);  // 正常速度
                       gpioPWM(12, servo_pwm_now);
                       motorSpeed = 9400;
                       //Running_State = FIND_CHANGE_LINE;//切换到变道2模式
                       std::cout << "变道完成" << endl;
                       Running_State =  FIND_PARK_SIGN;
               	}                                
                }
                else if(change_direction == -1)//左道
                {
//                    servo_pwm_now = servo_pd(166);  // 贴右线
//                    cout << "pwm" << servo_pwm_now << endl;
//                    gpioPWM(13, 9600);  // 正常速度
//                    gpioPWM(12, servo_pwm_now);
//                    motorSpeed = 9600;
               	if(biandao2 == -1)
               	{
               		 std::cout << "未识别" << endl;//向右变道
                       std::cout << "开始变道" << endl;
                       TimeRecord_Start_1 = std::chrono::high_resolution_clock::now();
                       servo_pwm_now = 960;  // 
                       cout << "pwm" << servo_pwm_now << endl;
                       gpioPWM(13, 9400);  // 正常速度
                       gpioPWM(12, servo_pwm_now);
                       motorSpeed = 9400;
                       while(1)
                       {
                           TimeRecord_End_1 = std::chrono::high_resolution_clock::now();
                           auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(TimeRecord_End_1 - TimeRecord_Start_1);
                           if(duration.count() >= 780)
                           {
                               break;
                           }
                       }
                       TimeRecord_Start_1 = std::chrono::high_resolution_clock::now();
                       servo_pwm_now = 740;
                       cout << "pwm" << servo_pwm_now << endl;
                       gpioPWM(13, 9400);  // 正常速度
                       gpioPWM(12, servo_pwm_now);
                       motorSpeed = 9400;
                       while(1)
                       {
                           TimeRecord_End_1 = std::chrono::high_resolution_clock::now();
                           auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(TimeRecord_End_1 - TimeRecord_Start_1);
                           if(duration.count() >= 740)
                           {
                               break;
                           }
                       }
                       servo_pwm_now = 858;//860  // 回正
                       cout << "pwm" << servo_pwm_now << endl;
                       gpioPWM(13, 9400);  // 正常速度
                       gpioPWM(12, servo_pwm_now);
                       motorSpeed = 9400;
                       //Running_State = FIND_CHANGE_LINE;//切换到变道2模式
                       std::cout << "变道完成" << endl;
                       Running_State =  FIND_PARK_SIGN;
                   
                     }
                  }  
            }
            else if(Running_State == FIND_PARK_SIGN)
            {
                std::cout << "寻找停车标志模式" << endl;
            
                //正常寻迹
                servo_pwm_now = servo_pd(166);  // 中线
                cout << "pwm" << servo_pwm_now << endl;
                gpioPWM(13, 9400);  // 正常速度
                gpioPWM(12, servo_pwm_now);
                motorSpeed = 9400;
                static unsigned  int A_Count = 0;
                static unsigned  int B_Count = 0;
                static int Detect_Time = 0;
                // if(Detect_Time<100)
                // {
                //int stop_sign = detectStopSign(frame, false);
                if(park_place == -1)
                {
                    std::cout << ">>> 锁定任务 A (左侧) <<<" << std::endl;
                    park_place = -1; // 锁定状态，防止跳变 
                    Running_State = PARKING; // 切换到停车执行状态
                    // A_Count++;
                    // Detect_Time ++;
                    // std::cout<<"发现停车标志 A"<< std::endl;//执行A任务:比如左变道停A点
                    // //正常寻迹
                    // servo_pwm_now = servo_pd(186);  // 靠左
                    // cout << "pwm" << servo_pwm_now << endl;
                    // gpioPWM(13, 9400);  // 正常速度
                    // gpioPWM(12, servo_pwm_now);
                    // motorSpeed = 9400;
                    // while(1)
                    // {
                    //     TimeRecord_End_1 = std::chrono::high_resolution_clock::now();
                    //     auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(TimeRecord_End_1 - TimeRecord_Start_1);
                    //     if(duration.count() >= 2000)
                    //     {
                    //         break;
                    //     }
                    // }
                    // cout << "pwm" << servo_pwm_now << endl;
                    // gpioPWM(13, 10300);  // 正常速度
                    // gpioPWM(12, servo_pwm_now);
                    // motorSpeed = 10300;
                    // std::cout << "停车 A" << std::endl;
                    /******************************************** */
                }

                else if(park_place == 1)
                {  
                    std::cout << ">>> 锁定任务 B (右侧) <<<" << std::endl; 
                    park_place = 1; // 锁定状态 
                    Running_State = PARKING; // 切换到停车执行状态
                    // B_Count++;
                    // Detect_Time ++;
                    // std::cout<<"发现停车标志 B"<< std::endl;//执行B任务:比如左变道停B点
                    // //正常寻迹
                    // servo_pwm_now = servo_pd(150);  // 靠右
                    // cout << "pwm" << servo_pwm_now << endl;
                    // gpioPWM(13, 9400);  // 正常速度
                    // gpioPWM(12, servo_pwm_now);
                    // motorSpeed = 9400;
                    //  while(1)
                    // {
                    //     TimeRecord_End_1 = std::chrono::high_resolution_clock::now();
                    //     auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(TimeRecord_End_1 - TimeRecord_Start_1);
                    //     if(duration.count() >= 2000)
                    //     {
                    //         break;
                    //     }
                    // }
                    // cout << "pwm" << servo_pwm_now << endl;
                    // gpioPWM(13, 10300);  // 正常速度
                    // gpioPWM(12, servo_pwm_now);
                    // motorSpeed = 10300;
                    // std::cout << "停车 B" << std::endl;
                    /******************************************** */
                }
                else
                {
                     //正常寻迹
                    servo_pwm_now = servo_pd(166);  // 中线
                    cout << "pwm" << servo_pwm_now << endl;
                    gpioPWM(13, 9400);  // 正常速度
                    gpioPWM(12, servo_pwm_now);
                    motorSpeed = 9400;
                    std::cout<<"啥也没看见"<< std::endl;//未识别
                }
                //std::cout<<"....."<< std::endl;
               // break;

            }
            else if(Running_State == PARKING)
            {
//                // 1. 检测横向黄线位置 
//                // y_pos 越大，说明线离车越近 (0在顶部，240在底部) 
//                int y_pos = find_parking_stop_line(frame, false); // true 可开启调试窗 
//                
//                std::cout << "停车模式 - 黄线Y坐标: " << y_pos << std::endl; 
//                // 2. 停车/转向 触发阈值 (需现场调试) 
//                // 假设当线到达 y=180 时，车头刚好压线或适合转弯 
//                int stop_threshold = 180;  
//                
//                if (y_pos > stop_threshold) 
//                { 
//                    std::cout << ">>> 到达停车线！执行停车动作 <<<" << std::endl; 
                    
                    // 执行入库动作 (根据你的赛道，是直接刹车还是拐进去) 
                    
                    // 动作 A: 如果你是直着开进去直接停 
                    // gpioPWM(13, 10000); // 刹车 
                    // usleep(200000); 
                    // Running_State = END; 
                    
                    // 动作 B: 如果需要拐弯进库 (比如检测到线后向左/右拐) 
                    if (park_place == -1) { // A任务 (例如向左拐) 
                         TimeRecord_Start_1 = std::chrono::high_resolution_clock::now();
                        servo_pwm_now = 790;  // 
                        cout << "pwm" << servo_pwm_now << endl;
                        gpioPWM(13, 9800);  // 正常速度
                        gpioPWM(12, servo_pwm_now);
                        motorSpeed = 9800;
                        while(1)
                        {
                            TimeRecord_End_1 = std::chrono::high_resolution_clock::now();
                            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(TimeRecord_End_1 - TimeRecord_Start_1);
                            if(duration.count() >= 630)
                            {
                                break;
                            }
                        }
                        TimeRecord_Start_1 = std::chrono::high_resolution_clock::now();
                        servo_pwm_now = 920;
                        cout << "pwm" << servo_pwm_now << endl;
                        gpioPWM(13, 9800);  // 正常速度
                        gpioPWM(12, servo_pwm_now);
                        motorSpeed = 9800;
                        while(1)
                        {
                            TimeRecord_End_1 = std::chrono::high_resolution_clock::now();
                            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(TimeRecord_End_1 - TimeRecord_Start_1);
                            if(duration.count() >= 750)
                            {
                                break;
                            }
                        }
                        servo_pwm_now = 858;//860  // 回正
                        cout << "pwm" << servo_pwm_now << endl;
                        gpioPWM(13, 10900);  // 正常速度
                        gpioPWM(12, servo_pwm_now);
                        motorSpeed = 10900;
                        //Running_State = FIND_CHANGE_LINE;//切换到变道2模式
                        std::cout << "变道完成" << endl;
                        break;	
                    }
                    else if (park_place == 1){ // B任务 (例如向右拐) 
                        TimeRecord_Start_1 = std::chrono::high_resolution_clock::now();
                        servo_pwm_now = 920;  // 
                        cout << "pwm" << servo_pwm_now << endl;
                        gpioPWM(13, 9800);  // 正常速度
                        gpioPWM(12, servo_pwm_now);
                        motorSpeed = 9800;
                        while(1)
                        {
                            TimeRecord_End_1 = std::chrono::high_resolution_clock::now();
                            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(TimeRecord_End_1 - TimeRecord_Start_1);
                            if(duration.count() >= 620)
                            {
                                break;
                            }
                        }
                        TimeRecord_Start_1 = std::chrono::high_resolution_clock::now();
                        servo_pwm_now = 740;
                        cout << "pwm" << servo_pwm_now << endl;
                        gpioPWM(13, 9800);  // 正常速度
                        gpioPWM(12, servo_pwm_now);
                        motorSpeed = 9800;
                        while(1)
                        {
                            TimeRecord_End_1 = std::chrono::high_resolution_clock::now();
                            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(TimeRecord_End_1 - TimeRecord_Start_1);
                            if(duration.count() >= 620)
                            {
                                break;
                            }
                        }
                        servo_pwm_now = 858;//860  // 回正
                        cout << "pwm" << servo_pwm_now << endl;
                        gpioPWM(13, 10900);  // 正常速度
                        gpioPWM(12, servo_pwm_now);
                        motorSpeed = 10900;
                        //Running_State = FIND_CHANGE_LINE;//切换到变道2模式
                        std::cout << "变道完成" << endl;
                        break;	
                    }
               
	                else 
	                { 
	                    // 还没到线，继续巡线或直行 
	                    // 建议稍微压低速度，防止冲过头 
	                    servo_pwm_now = servo_pd(166);  
	                    gpioPWM(12, servo_pwm_now); 
	                    gpioPWM(13, 9800); // 慢速接近 
	                    motorSpeed = 9800;
	                    //break;
	                } 
            }
            else if(Running_State == DEBUG)
            {
                std::cout << "调试模式" << endl;
                servo_pwm_now = 720;  // 目标值设为160（正常行驶）166
            	cout << "pwm" << servo_pwm_now << endl;
                gpioPWM(13, 11000);  // 正常速度
                gpioPWM(12, servo_pwm_now);
                motorSpeed = 11000;

            }
//			  bool has_cone = check_blue_exist_simple(frame, -1, false); // false 关闭调试窗 
//                if (has_cone)
//                {
//                    std::cout << "识别到锥桶" << endl;
//                    servo_pwm_now = 858;//860  // 回正
//                    cout << "pwm" << servo_pwm_now << endl;
//                    gpioPWM(13, 9400);  // 正常速度
//                    gpioPWM(12, servo_pwm_now);
//                    motorSpeed = 9400;
//                }
//                else
//               {
//                    if(blue_disapear < 8)
//                    {
//                        blue_disapear ++ ;
//                        std::cout << "识别到锥桶" << endl;
//                        servo_pwm_now = 858;//860  // 回正
//                        cout << "pwm" << servo_pwm_now << endl;
//                        gpioPWM(13, 9400);  // 正常速度
//                        gpioPWM(12, servo_pwm_now);
//                        motorSpeed = 9400;
//
//                    }
//                    else
//                    {
//                        std::cout << "未识别" << endl;
//                        std::cout << "开始变道" << endl;
//                        TimeRecord_Start_1 = std::chrono::high_resolution_clock::now();
//                        servo_pwm_now = 760;  // 
//                        cout << "pwm" << servo_pwm_now << endl;
//                        gpioPWM(13, 9300);  // 正常速度
//                        gpioPWM(12, servo_pwm_now);
//                        motorSpeed = 9300;
//                        while(1)
//                        {
//                            TimeRecord_End_1 = std::chrono::high_resolution_clock::now();
//                            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(TimeRecord_End_1 - TimeRecord_Start_1);
//                            if(duration.count() >= 820)
//                            {
//                                break;
//                            }
//                        }
//                        TimeRecord_Start_1 = std::chrono::high_resolution_clock::now();
//                        servo_pwm_now = 960;
//                        cout << "pwm" << servo_pwm_now << endl;
//                        gpioPWM(13, 9300);  // 正常速度
//                        gpioPWM(12, servo_pwm_now);
//                        motorSpeed = 9300;
//                        while(1)
//                        {
//                            TimeRecord_End_1 = std::chrono::high_resolution_clock::now();
//                            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(TimeRecord_End_1 - TimeRecord_Start_1);
//                            if(duration.count() >= 730)
//                            {
//                                break;
//                            }
//                        }
//                        servo_pwm_now = 858;//860  // 回正
//                        cout << "pwm" << servo_pwm_now << endl;
//                        gpioPWM(13, 10000);  // 正常速度
//                        gpioPWM(12, servo_pwm_now);
//                        motorSpeed = 10000;
//                        //Running_State = FIND_CHANGE_LINE;//切换到变道2模式
//                        std::cout << "变道完成" << endl;
//                        break;	
//                    }
//               }
            std::cout << " speed:" << motorSpeed << std::endl;//输出当前速度
            // 计算并输出处理时间
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            std::cout << " times：" << duration.count() / 1000 << "s" << std::endl;
            auto current_duration = duration.count() / 1000;
            std::cout << " Error:" << error_first << std::endl;//输出当前偏差

            // 累计总时间
            total_duration += current_duration;
            std::cout << "Total duration: " << total_duration/1000 << "s" << std::endl;
            auto current_time = std::chrono::high_resolution_clock::now();

            if (first_capture) {
                last_capture_time = current_time;
                first_capture = false;
            }

            auto time_since_last_capture = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_capture_time).count();

            if (time_since_last_capture >= CAPTURE_INTERVAL) {
                // 获取当前时间并格式化
                auto now = std::chrono::system_clock::now();
                auto time_t_now = std::chrono::system_clock::to_time_t(now);
                std::tm* local_time = std::localtime(&time_t_now);
                
                std::stringstream time_ss;
                time_ss << std::put_time(local_time, "%Y-%m-%d %H:%M:%S");
                std::string time_str = time_ss.str();
                
                // 在图像上添加时间戳
                cv::Mat frame_with_timestamp = frame.clone();
                cv::putText(frame_with_timestamp, time_str, 
                            cv::Point(10, 30), // 文字位置（左上角开始）
                            cv::FONT_HERSHEY_SIMPLEX, 
                            0.6, // 字体大小
                            cv::Scalar(0, 255, 0), // 绿色文字
                            2); // 线宽
                
                // 生成文件名（包含时间戳）
                std::stringstream filename_ss;
                filename_ss << "/home/pi/5Gcode4/captured_frames/" // 确保目录存在（如果不存在需要手动创建）
                            << std::put_time(local_time, "%Y%m%d_%H%M%S") 
                            << ".jpg";
                std::string filename = filename_ss.str();
                
                // 确保目录存在（如果不存在需要手动创建）
                // system("mkdir -p /home/pi/captured_images");
                
                // 保存图片
                bool save_success = cv::imwrite(filename, frame_with_timestamp);
                if (save_success) {
                    std::cout << "照片已保存: " << filename << std::endl;
                } else {
                    std::cout << "照片保存失败: " << filename << std::endl;
                }
                
                // 更新最后拍照时间
                last_capture_time = current_time;
            }
            // 更新开始时间
            start_time = end_time;
                 
        
           }
    }
    // 释放资源
    capture.release();  // 关闭摄像头
    destroyAllWindows();  // 关闭所有OpenCV窗口
    system("pause");  // 暂停程序
    return 0;  // 程序正常退出
}
