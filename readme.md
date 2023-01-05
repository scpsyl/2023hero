# 吉甲大师—英雄机器人电控代码(2023)

---

## 模块及文件说明

---

注：**“包含文件”**所示目录为**工程相对目录** 通过打开IDE查看

### 工程基础配置

#### 底层库HAL

* 说明：通过CubeMX生成
* 包含文件：`./Application/User`下`.c/h`
* 相关说明：通过查看CubeMX具体配置

#### FreeRTOS

* 说明：工程基于FreeRTOS，方便进行系统整体任务调度
* 包含文件“`./Middlewares/FreeRTOS`下`.c/h`

---

### 使用的队内开源代码

#### 通用队列算法

* 说明：提供了一套完整的类型通用的队列算法
* 包含文件：`./math/myQueue.c/h`
* 相关说明：[通用队列算法](https://gitee.com/tarsgo-embedded/open-source/tree/master/universal_queue)

#### 鼠标键盘事件模块

* 说明：对遥控器数据进行了二次封装，能够更方便地实现更复杂的功能
* 包含文件：`./bsp/RC.c/h`
* 相关说明：[鼠标键盘事件模块](https://gitee.com/tarsgo-embedded/open-source/tree/master/key_event)

#### 离线检测模块

* 说明：进行各模块掉线和上线的检测，并提供回调触发功能
* 包含文件：`./UI/Detect.c/h`
* 相关说明：[离线检测模块](https://gitee.com/tarsgo-embedded/open-source/tree/master/offline_detect)

---

### 数学工具模块

* **说明**：包含在`./math`目录下

#### PID模块

* 说明：PID控制函数，包含单级和串级计算
* 包含文件：`PID.c/h`

#### 滤波模块

* 说明：包含低通滤波、均值滤波、一阶卡尔曼滤波计算
* 包含文件：`Filter.c/h`

#### 斜坡函数模块

* 说明：进行斜坡函数的相关计算，用于突变信号的缓冲

* 包含文件：`Slope.c/h`

---

### 裁判系统封装模块

* **说明**：包含在`./UI`目录下

#### CRC校验模块

* 说明：通过循环数组模拟软件CRC校验，用于对裁判系统数据校验。校验公式为
  $$
  G(x)=x^8+x^5+x^4+1
  $$

* 包含文件：`crc.c/h`

#### UI绘图模块

* 说明：对裁判系统UI绘制进行封装，更方便创建和绘制UI界面
* 包含文件：`Grapghics.c/h`

#### 裁判系统模块

* 说明：包含接收裁判系统数据、解析裁判系统数据、发送数据给裁判系统的相关功能

* 包含文件：`Judge.c/h`

---

### 主逻辑模块

* **说明**：包含于`./module`目录下

#### 底盘模块

* 说明 包含所有与**底盘动作**相关的任务逻辑及接口函数
* 包含文件：`Chassis.c/h`

#### 云台模块

* 说明：包含所有与**云台动作**相关的任务逻辑及接口函数
* 包含文件：`Gimbal.c/h`

#### 发射机构模块

* 说明：包含所有与**发射机构**相关的任务逻辑及接口函数
* 包含文件：`Shooter.c/h`

#### 视觉数模块

* 说明：包含与视觉上位机收发数据的相关函数

* 包含文件：`Vision.c/h`

---

### BSP模块

* **说明**：包含在`./bsp`目录下

#### 电机驱动模块

* 说明：包含电机数据处理(数据更新、角度累计、编码器角度结算等)和电机控制任务
* 包含文件：`Motor.c/h`

#### 蜂鸣器模块

* 说明：封装板载蜂鸣器驱动 方便利用蜂鸣器提示当前工作状态
* 包含文件：`Beep.c/h`

#### CAN用户驱动模块

* 说明：包含CAN接收中断回调(及数据解析)和电机及超级电容数据发送功能,
* 包含文件：`USER_CAN.c/h`

#### 超级电容模块

* 说明：与功率控制板进行通信，控制输入输出功率
* 包含文件：`super_cap.c/h`

---

### 陀螺仪模块

* **说明**：
  * 包含在`./IMU`目录下
  * C板子带的高精度陀螺仪和磁力计，移植自官方

---

### TOF测距模块

* **说明**：

  * 包含在`./TOF`目录下

  * 对TOF测距模块进行了封装，可以方便的调用TOF距离等数据

* 包含文件：`User_TOF.c/h`，`nlink_tofsense_frame0.c/h`，`nlink_utils.c/h`

### SK80测距模块
