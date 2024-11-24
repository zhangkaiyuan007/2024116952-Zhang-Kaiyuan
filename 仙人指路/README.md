# 仙人指路

> 2025赛季Helios视觉部笔试

##### 写在前面

本题中希望大家使用卡尔曼滤波算法解决问题，但考虑到大多数同学在接触本题前没有接触过卡尔曼滤波，本题末尾附有教程和参考资料供大家学习。

本题题干很长，有较多的信息描述，乍一看令人头大，但如果仔细思考并加以尝试，问题其实没有那么复杂。

虽然本题希望大家通过卡尔曼滤波作答，但本题还有许多其他解决方法，你可以用任何想到的方式解决本题，如果无法解决，只提交解题思路也能拿到少量分数。

---------------------------

## 题目背景

南瓜是一个怀揣着机甲梦的少年，8月的一天，他在春茧体育馆的地下迷宫中求仙问道，偶遇了勺子仙人。勺子仙人素以机甲界的控之尊者闻名，南瓜有此奇遇，便意欲向之求教，望得指点一二。

勺子仙人不堪其扰，便赐予南瓜宝具``超级司南``一副，让南瓜持``超级司南``与自己共同进行控之修行。

勺子仙人会进行三种不同的控之修行，分别是

1. 原地打坐(速度为0)
2. 定速飘逸(速度为定值)
3. 控之舞步(移动速度遵循三角函数曲线)

而，南瓜若想与勺子仙人共同修行，则需要模仿勺子仙人的动作(模仿指和勺子仙人处于相同位置，相同速度)。

但勺子仙人在修行中会隐藏自己的气息，尚未成仙的南瓜固然无法察觉勺子仙人身在何处，不过勺子仙人的宝具``超级司南``会指明勺子仙人在自己的哪个方位，距离自己还有多远。南瓜需要根据``超级司南``提供的勺子仙人的位置信息，与勺子仙人一同修行。

不过``超级司南``与勺子仙人久居，已沾染了不少仙气，他知道仙法不可轻传，因此他报给南瓜的位置会在勺子仙人的真实位置上叠加一层随机误差，如何在有误差的位子的基础上追随勺子仙人的位置，就需要南瓜自己克服了。

## 题目描述

本题中，你需要控制一个机器人跟随一个不可见的目标进行移动，目标的位置会由一个传感器获得。下面分别描述``机器人``，``不可见目标``，``传感器``的特性。

#### 机器人

在实际应用中，机器人的控制会伴随复杂的控制算法与模型，在本题中，机器人的控制并非核心考察点，因此我们简化机器人的控制模型。

本题中涉及的机器人只有一维的移动(即只有x坐标)，你可以控制机器人前往一个目标点。假设机器人的当前位置为``x0``，目标位置为``x1``，则每经过一个控制周期，机器人会到达``x0 + (x1 - x0) * 0.8``这一新位置。程序中模拟的机器人控制频率为``1000Hz``。

机器人的控制存在延迟，每当你向机器人发送控制指令后，机器人会在``10ms``后收到指令并开始执行。

在程序实现中，我们将和机器人的交互封装成了一个动态链接库，你可以直接调用接口函数来控制机器人，但是你不能看到机器人的内部实现代码。下面介绍接口函数。

```cpp
/*
 * get your current position
*/
void GET_CURRENT_POSITION(float &result);

/*
 * set where is the target
 * after you set the target position, you will move to the target position
*/
void SET_TARGET_POSITION(const float &target);

/*
 * is log enable is set true, log "Chassis Control" will be enabled
 * otherwise, you will not see the log
*/
void ENABLE_LOG(const bool &log_enable);

/*
 * if you compile this project with OpenCV and this function is called,
 * the debug window(the image show where you are and where the target is) will be shown.
*/
void SHOW_DEBUG_IMAGE();
```

* ``GET_CURRENT_POSITION``:得到机器人的当前位置，本函数不存在延迟。
* ``SET_TARGET_POSITION``:设置机器人的目标位置，本函数存在``10ms``的延迟，在调用本函数的``10ms``后，机器人按照上文描述的控制方式，即以``1000Hz``的控制频率，每次控制以``0.8``的学习率前往目标位置。
* ``ENABLE_LOG``:可以控制是否打开机器人的调试信息输出。如果开启``Log``，则机器人的每次控制时，将会输出真正的不可见目标的真实位置(勺子仙人的位置)，当前用户设定的目标的位置(``SET_TARGET_POSITION``函数设定的位置)，和机器人的当前位置(由于机器人有控制模型，机器人的目标位置和当前位置并不完全相同)。
* ``SHOW_DEBUG_IMAGE``:显示调试图像，图像中会有``刻度``，``绿色的球``，``红色的圈``，``黄色的球``。分别辅助表示位置信息，不可见目标的实际位置，当前的``target_position``，当前机器人的真是位置。需要注意的是，只有在编译过程中包含``OpenCV``编译，``SHOW_DEBUG_IMAGE``才会正常显示图像。

#### 不可见目标

你需要控制机器人跟随不可见目标，不可见目标只有一维的移动(即只有x坐标)，不可见目标会有三种不同的模式，我们分别记为``模式零``，``模式一``，``模式二``。

* 模式零：不可见目标将在一个随机位置原地不动。

* 模式一：不可见目标将会从``x=0``开始，以一个``1m/s~2m/s``之间的随机固定速度进行移动。

* 模式二：不可见目标将会从``x=0``开始，以下面的三角函数作为速度进行移动，其中``t``为时间，`t0`为`0s~3s`之间的随机初始时刻。


$$
speed = 1.57 \times sin(3.688(t_0+t)) + 2.61
$$

程序实现中，你可以通过函数接口控制不可见目标的移动模式。下面介绍函数接口。

```cpp
/*
 * set the mode of the target
 * if mode = 0, the target will stay still
 * if mode = 1, the target will move at a definate speed
 * if mode = 2, the target will move at a chaning speed(speed = 1.57 * sin(3.688 * t) + 2.61) 
*/
void SET_MODE(const int &mode);
```

你可以通过``SET_MODE``函数，将不可见目标的移动方式设置为``1``,``2``,``3``。

#### 传感器

传感器可以得到机器人位置到不可见目标的距离，距离的返回值存在随机误差，传感器数据返回存在``10ms``延迟。

设机器人的实际位置为``current_position``(这个数值不是你设置的机器人位置，而是机器人的实际到达的位置)，不可见目标的实际位置为``object_position``，则传感器回传的数据为``(object_position - current_position) + random_error``。

在程序实现中，你可以通过接口函数获得传感器的数据回传。下面介绍接口函数。

```cpp
/*
 * get how far you are from the target
*/
void GET_SENSOR_DISTANCE(float &result);
```

你可以通过``GET_SENSOR_DISTANCE``函数得到机器人到不可见目标的位置，但是返回值存在随机噪声，数据返回具有``10ms``延迟。

#### 实现目标

在本题中，你需要通过读取传感器数据，控制机器人跟随不可见目标，我们将考察你具体的程序实现以及跟随目标的效果来评定你的分数。

你的控制结果会有三种误差来进行衡量，分别为``current_error``，``average_error``,``expotional_error``。

* ``current error``：当前的控制误差，``current_error = |object_position - current_position|``。
* ``average error``：即``current_error``取平均值。
* ``expotional error``：即``expotional_error = (1 - 0.001) * expotional_error_last + 0.001 * current_error``，这一误差会反映你在最近1秒左右的跟随质量到底如何。

#### 惩罚重启机制

在你的跟随过程中，如果程序检测到你连续``5s``中的``expotional_error``数值大于``0.1``，即你在连续``5s``内的跟随质量较差，那么不可见目标的位置和状态将会发生变化。

* 模式零：不可见目标将会更换位置。
* 模式一：不可见目标将会更换随机速度，并重新回到原点。
* 模式二：不可见目标的速度将会重置(时间归零，更换初始时刻)，并重新回到原点。

## 题目任务

本题中，你需要设计程序，在三种模式下，分别控制机器人以尽可能小的误差跟随不可见目标。

## 项目结构

在笔试试题中，你会拿到名为`cv_enter_examination_kalman_filter`的文件夹，下图为该文件夹中的项目结构树，你最终应该提交修改过 `main.cpp` 的项目文件夹。

```
.
├── CMakeLists.txt					项目的CMake工程文件，包含是否开启OpenCV编译的选项
├── chassis.h					    函数接口头文件
├── graphic							SHOW_DEBUG_IMAGE函数相关文件，你可以自行修改
│   ├── CMakeLists.txt
│   ├── display_board.cpp
│   └── display_board.h
├── kalman							卡尔曼滤波的第三方库文件，你可以调用该文件实现项目
│   ├── AdaptiveEKF.h
│   ├── CMakeLists.txt
│   └── kalman.h
├── kalman_filter_tutorial.pdf		一份卡尔曼滤波的相关教程
├── lib								动态链接库，包含接口头文件对应的实现库
│   └── libchassis_lib.a
└── main.cpp						项目主文件，你需要修改该文件中的代码
```

## 梯度说明

#### 梯度一

实现模式零。

#### 梯度二

实现模式零、模式一。

#### 梯度三

完整实现本题全部功能。

## 提示

在本题中，你可以使用各种不同手段答题。本题考查核心点为滤波算法，如果你掌握了卡尔曼滤波，那么你一定可以使用卡尔曼滤波解决三种模式下的跟随问题。

但也鼓励使用其他的方法解决，例如曲线拟合，快速傅里叶变换等方法，如果设计合理，也可以解决本题中的跟随问题。

下面给出卡尔曼滤波的一些参考学习资料：

| 资料名称                                              | 链接                                                         |
| ----------------------------------------------------- | ------------------------------------------------------------ |
| 基于python的滤波教程，但理论知识是共通的              | https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python |
| 交龙博客中的卡尔曼滤波教程，和项目文件中的pdf为同一个 | https://sjtu-robomaster-team.github.io/vision-learning-6/    |
发


#### 项目依赖

| 第三方库 | 安装指令                       |
| -------- | ------------------------------ |
| opencv   | sudo apt install libopencv-dev |
| eigen    | sudo apt install libeigen-dev  |

你可以使用以下指令编译项目：

```
mkdir build
cd build
cmake ..
make
```

## 提示

在本题中，你可以使用各种不同手段答题。本题考查核心点为滤波算法，如果你掌握了卡尔曼滤波，那么你一定可以使用卡尔曼滤波解决三种模式下的跟随问题。

但也鼓励使用其他的方法解决，例如曲线拟合，快速傅里叶变换等方法，如果设计合理，也可以解决本题中的跟随问题。

下面给出卡尔曼滤波的一些参考学习资料：

| 资料名称                                              | 链接                                                         |
| ----------------------------------------------------- | ------------------------------------------------------------ |
| 基于python的滤波教程，但理论知识是共通的              | https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python |
| 交龙博客中的卡尔曼滤波教程，和项目文件中的pdf为同一个 | https://sjtu-robomaster-team.github.io/vision-learning-6/    |



