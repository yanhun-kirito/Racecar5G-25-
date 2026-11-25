# Racecar5G-25- · 室外 5G 智能车

2025 年全国大学生智能车竞赛室外 5G 赛道华南赛区二等奖参赛代码。项目运行在树莓派上，用 OpenCV 处理车载摄像头画面，用 pigpio 向转向舵机和电机电调输出 PWM。小车通过蓝色挡板发车，随后执行循迹、蓝色锥桶避障、十字路口停车、变道和入库。

车端程序从摄像头和交互终端获取输入。复现远程操作时，可通过已有网络进入树莓派终端并传入按键，例如使用带交互终端的 SSH 会话。5G 接入、独立遥控客户端和视频推流需另行部署。

## 项目结构

| 文件 | 用途 |
| --- | --- |
| [demo.cpp](Smartcar_5g-main%28chai%29/demo.cpp) | 车端入口。包含摄像头采集、视觉算法、任务状态机、终端输入和 GPIO 输出。 |
| [CMakeLists.txt](Smartcar_5g-main%28chai%29/CMakeLists.txt) | 原车的 CMake 配置，定义 `main`、`pid` 和 `init` 构建目标。 |
| [pid.h](Smartcar_5g-main%28chai%29/pid.h) / [pid.cpp](Smartcar_5g-main%28chai%29/pid.cpp) | 通用位置式 PID 和输出限幅。当前主流程的转向由 `demo.cpp` 中的 PD 函数计算。 |
| [init.cpp](Smartcar_5g-main%28chai%29/init.cpp) | 旧版硬件初始化，包含电机、舵机和蜂鸣器配置。`main()` 使用自身的 `servo_motor_pwmInit()`。 |
| [take_photo.py](Smartcar_5g-main%28chai%29/take_photo.py) / [calibration.py](Smartcar_5g-main%28chai%29/calibration.py) | 分别用于校正后抓拍和去畸变预览。 |
| [blue_area.py](Smartcar_5g-main%28chai%29/blue_area.py) / [bluecard_jiance.py](Smartcar_5g-main%28chai%29/bluecard_jiance.py) | 分别用于离线蓝色轮廓检查和实时蓝色挡板检测实验。 |
| [使用前请阅读.txt](使用前请阅读.txt) | 原始编译和终端恢复说明。下文给出了 OpenCV 4 环境下的完整步骤。 |

## 实现逻辑

### 视觉与控制

1. **采集与校正。** `main()` 打开摄像头索引 `0`，请求 MJPG、30 FPS、320 × 240。每帧由 `undistort()` 使用预设相机内参和畸变系数校正。赛道处理使用了固定像素坐标，换摄像头或分辨率时需要同时核对内参、取景和感兴趣区域。摄像头实际接受的参数取决于设备及采集后端。 [OpenCV VideoCapture 文档](https://docs.opencv.org/4.x/d8/dfe/classcv_1_1VideoCapture.html)
2. **提取赛道。** `ImagePreprocessing()` 使用灰度图、Sobel 梯度、Otsu 阈值和膨胀提取边缘，在指定区域做概率霍夫直线检测，过滤近水平线段。`Tracking()` 从下向上搜索左右边界，计算中线 `mid`。单侧丢线时用估计赛道宽度补齐，两侧均丢失时沿用前一行中点，并限制中点跳变。
3. **计算转向。** `servo_pd(target)` 采样多行中线横坐标，用目标横坐标减去采样均值得到像素偏差，再结合比例项和相邻帧偏差变化计算舵机 PWM。正常循迹分支常用 `servo_pd(166)`，其他状态可以改变目标点或直接输出定时 PWM。
4. **识别赛道标志。** 发车挡板使用 HSV 掩码、形态学处理和轮廓面积。锥桶检测对 HSV 亮度通道做 CLAHE，再按颜色、面积、圆度和赛道边界筛选。十字路口检测在 HSV 掩码的感兴趣区域内统计相邻像素跳变。方向牌检测先透视校正蓝底标牌，再用上半部分白色箭头的重心判别左右。
5. **调度与输出。** 主线程按当前状态处理画面并输出 PWM。后台 `input_thread()` 从终端读取单字符。正常运行阶段每隔约 30 秒尝试保存一张带时间戳的 JPEG，部分定时动作会暂时阻塞逐帧处理。

### 主流程状态

| 状态 | 行为与转移条件 |
| --- | --- |
| `READY` | 在图像约第 90–160 行寻找蓝色挡板。识别后移开挡板，程序确认目标消失并等待约 2 秒后发车。 |
| `START` | 正常循迹并累计有效蓝色锥桶检测结果。达到条件后执行一组定时避障动作，进入 `FIND_CROSS`。 |
| `FIND_CROSS` | 保持循迹并寻找斑马线。累计检测达到条件后刹车，进入 `CROSS_STOP`。 |
| `CROSS_STOP` | 小车在路口等待，尝试播放一次语音并识别左右箭头。同一方向累计识别到 3 次即可触发首次变道，也可用终端按键选方向。 |
| `LANE_CHANGE_1` | 依据已选方向执行首次变道，随后进入 `FIND_CHANGE_LINE`。 |
| `FIND_CHANGE_LINE` | 继续循迹，等待与首次方向对应的按键触发第二次变道。动作完成后进入 `FIND_PARK_SIGN`。 |
| `FIND_PARK_SIGN` | 循迹等待终端选择 A 或 B 车位。源码中的 A/B 自动识别调用在当前主流程中处于注释状态。 |
| `PARKING` | 按 A/B 车位执行对应的定时转向和停车动作，随后退出摄像头循环。 |

`AVOID_CONE`、`LANE_CHANGE_2` 和 `END` 保留在状态枚举中。上表按当前 `main()` 的实际状态转移整理。`DEBUG` 分支可用于源码调试，程序启动时进入 `READY`。

## 安装与编译

### 1. 准备树莓派环境

- 使用可运行 pigpio C 库的树莓派和 Raspberry Pi OS。树莓派 5 上使用上游 pigpio 时需检查兼容性。 [pigpio 项目](https://github.com/joan2937/pigpio) · [树莓派 5 兼容性跟踪](https://github.com/joan2937/pigpio/issues/589)
- 接入摄像头、转向舵机和电调。GPIO 使用 BCM 编号。联调时先架空驱动轮，检查舵机方向和电调中位。

| 设备 | BCM GPIO | PWM 频率 | PWM 范围 | 初始化写入值 |
| --- | --- | --- | --- | --- |
| 转向舵机 | 12 | 50 Hz | 10000 | 864 |
| 电机电调 | 13 | 200 Hz | 40000 | 10500 |

在 Raspberry Pi OS 上安装编译和图像处理依赖。树莓派上若已有本项目文件，直接进入相同的源码目录即可。

```bash
sudo apt update
sudo apt install git build-essential pkg-config libopencv-dev
git clone https://github.com/yanhun-kirito/Racecar5G-25-.git
cd 'Racecar5G-25-/Smartcar_5g-main(chai)'
```

还需要本机的 `pigpio.h` 和 `libpigpio.so`。若系统仓库提供 `libpigpio-dev`，可以运行 `sudo apt install libpigpio-dev`。也可以按上游说明构建和安装 pigpio。车端程序直接调用 `gpioInitialise()`。 [pigpio 安装说明](https://github.com/joan2937/pigpio/blob/master/README)

语音播报使用 `aplay`（`alsa-utils` 软件包）。如需播报，在 `demo.cpp` 中搜索 `nibuhuidawo.WAV`，配置为树莓派上实际存在的音频文件，再重新编译。该 WAV 文件需自行提供。

### 2. 编译车端程序

在源码目录执行以下命令。OpenCV 4 的 pkg-config 模块名通常为 `opencv4`。旧环境若只提供 `opencv`，把两条命令中的模块名改为 `opencv`。

```bash
pkg-config --modversion opencv4
mkdir -p build
g++ -std=c++17 -pthread demo.cpp -o build/demo \
    $(pkg-config --cflags --libs opencv4) -lpigpio -lrt
```

安装 `cmake` 后，也可以使用原车 CMake 配置构建。使用新的构建目录，避免读到旧设备生成的缓存。

```bash
cmake -S . -B build-cmake
cmake --build build-cmake --parallel
```

CMake 方式生成 `build-cmake/main`。原配置写死了 `/usr/lib/libpigpio.so` 等路径，库位置不同时可使用上方的 `g++` 命令。原配置的最低版本声明为 2.8，用 CMake 4.x 遇到策略版本报错时，可在配置命令中加上 `-DCMAKE_POLICY_VERSION_MINIMUM=3.5`。 [CMake 最低版本说明](https://cmake.org/cmake/help/latest/command/cmake_minimum_required.html)

## 执行与操作

### 启动前准备抓拍目录

> **程序启动会清理抓拍目录中的既有文件。** 当前清理目录和抓拍文件名前缀分别写在 `demo.cpp` 的 `save_dir` 与 `filename_ss` 处，默认均指向 `/home/pi/5Gcode4/captured_frames`。请使用专用目录，先备份需要保留的照片。若要改路径，请同时修改两处并重新编译。

```bash
sudo mkdir -p /home/pi/5Gcode4/captured_frames
sudo ./build/demo
```

使用 CMake 时运行 `sudo ./build-cmake/main`。程序在初始化时即会向执行器写入 PWM，请先确认车辆有可用的物理断电方式。通过 SSH 操作时，先进入交互会话再启动程序。

### 按赛道顺序操作

1. 启动后把蓝色挡板放入摄像头可识别范围。看到 `find biggest blue` 后移开挡板，等待程序进入正常行驶。
2. 小车循迹、避障并在斑马线处停车。进入 `CROSS_STOP` 后，可以等箭头识别，也可以用终端按键选择首次变道方向。
3. 按终端日志确认 `变道完成` 后，进入 `FIND_CHANGE_LINE`。根据首次变道方向按下相应按键，触发返回车道的动作。
4. 进入 `FIND_PARK_SIGN` 后，选择左侧 A 车位或右侧 B 车位。程序执行对应动作，随后退出。

终端按键无需回车，`=` 与 `+` 等效。

| 当前状态 | `-` | `+` / `=` |
| --- | --- | --- |
| `CROSS_STOP` | 首次向左变道 | 首次向右变道 |
| `FIND_CHANGE_LINE` | 首次向左时触发返回 | 首次向右时触发返回 |
| `FIND_PARK_SIGN` | 选择左侧 A 车位 | 选择右侧 B 车位 |

数字键 `0`–`9` 写入旧逻辑使用的 `key`（初始为 `8`），最多接收 10 次。当前入库和变道选项使用上表中的符号键。

用 `Ctrl+C` 中断进程。终端输入不回显时，输入 `reset` 后按回车。停车和中断时请现场确认驱动轮已经停转，必要时切断驱动电源。

## Python 视觉工具

在有图形桌面的环境中运行这些工具。安装 Python 3、OpenCV Python 和 NumPy。使用 Raspberry Pi OS 系统 Python 时，可运行 `sudo apt install python3-opencv python3-numpy`。实时工具默认打开摄像头 `0`，使用前先关闭正在占用它的主程序。

| 工具 | 执行及操作 |
| --- | --- |
| `take_photo.py` | 在源码目录运行 `mkdir -p test_photo`，再运行 `python3 take_photo.py`。在图像窗口按 `q`，保存去畸变画面到 `test_photo/40.jpg` 并退出。再次抓拍使用同一文件名。 |
| `calibration.py` | 运行 `python3 calibration.py`，用脚本内给定的另一组参数查看校正效果。等待程序结束或在终端按 `Ctrl+C` 退出。标定参数需要事先测得，脚本只做去畸变预览。 |
| `blue_area.py` | 先把 `image_path` 改为一张存在的本地图片，再运行 `python3 blue_area.py`。窗口会显示筛选出的蓝色轮廓和面积，按任意键退出。 |
| `bluecard_jiance.py` | 运行 `python3 bluecard_jiance.py`，查看原图和蓝色掩码，在图像窗口按 `Esc` 退出。该实验脚本的检测区域和面积门限与车端 `READY` 分支不同。 |

## 调试与排错

| 现象 | 检查方法 |
| --- | --- |
| 编译找不到 OpenCV 头文件 | 运行 `pkg-config --cflags opencv4`，确认安装了开发包并能查询到头文件路径。 |
| 找不到 `pigpio.h` 或 `-lpigpio` | 检查 pigpio C 库及头文件是否安装，确认链接器能搜索所在目录。 |
| 输出 `GPIO failed` | 检查树莓派型号、pigpio 版本和运行权限，并确认其他进程是否占用了 pigpio。 |
| 输出 `Can not open video file!` | 当前入口打开的是摄像头索引 `0`。检查摄像头连接、索引、占用情况和实际输出分辨率。 |
| 挡板出现后仍不发车 | 查看 `find biggest blue` 日志并确认挡板完全离开识别区域。按实拍画面检查 `READY` 分支的 HSV 阈值和区域。 |
| 循迹偏移或转向异常 | 先核对相机安装角度和 `undistort()`，再检查 `Tracking()` 的扫描区间、赛道宽度、`servo_pd()` 的目标点及 `kp` / `kd`。正常循迹速度使用 `speed_all`，部分动作有独立 PWM 和计时值。 |
| 第二次变道或停车一直等待 | 核对终端日志和当前状态，只在相应状态按键。第二次变道按键需与首次变道方向匹配，入库需选择 A 或 B。 |
| 抓拍失败或路口没有语音 | 检查抓拍路径是否存在且可写，核对 `aplay`、WAV 文件路径及音频输出设备。 |

源码调试时，`servo_pd()` 中的局部变量 `kd_filter` 在初始化表达式中读取自身，转向输出异常时应优先检查该处。修改视觉参数、PWM 或路径后需要重新编译车端程序，再在架空驱动轮的条件下核对输出。

原项目学习资料见源码目录的 [原始 README](Smartcar_5g-main%28chai%29/README.md)。
