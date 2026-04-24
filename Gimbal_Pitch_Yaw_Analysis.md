# 双轴云台 Pitch / Yaw 链路分析

## 1. 结论先行

这个项目里和云台姿态有关的代码，实际上有两条链路：

1. 当前真正运行的链路  
   `BMI088 -> Mahony 四元数 -> 世界坐标系下的云台 yaw/pitch -> PID -> DJI6020 电机`

2. 已经实现但当前没有启用的链路  
   `电机编码器 -> 机构模型 -> 相对“启动时基座(BaseStart)”的 yaw/pitch`

当前 `main_Task_1ms()` 里启用的是第一条链路，第二条链路对应的
`Gimbal.Update_Imu_Pose_Relative_BaseStart(...)` 已经被注释掉了，实际调用的是
`Gimbal.Update_Imu_Pose_Relative_World(...)`
（见 `Usercode/3_application/Gimbal_Task/Gimbal_Task.cpp:301-313`）。

最关键的结论有 6 条：

- 当前角度环反馈不是电机编码器角，而是 IMU 解算出的世界系连续角。
- 当前速度环反馈是 DJI 电机反馈的转速，单位是 `rad/s`。
- 当前 `yaw` / `pitch` 目标单位是角度 `deg`，角度环输出的是给速度环用的“速度目标值”，其量纲靠 PID 参数隐含匹配到 `rad/s`。
- 当前世界系 `yaw` 正方向是“向左转为正”，`pitch` 正方向是“下俯为正”。
- 代码内部的 `RotationMatrix_To_YawPitchRoll()` 本身定义的 `pitch` 却是“上仰为正”，因此世界系链路里又额外做了一次取反。
- 当前所谓“世界坐标系”不是地理北东地或绝对航向系，而是一个“重力对齐、航向零点任意”的局部参考系。因为 Mahony 只用了 IMU，没有磁力计，`yaw` 只靠陀螺积分。

## 2. 当前实际运行的控制链路

### 2.1 任务和数据流

当前实际运行链路可以写成：

```text
BMI088 中断(200Hz)
-> bmi088_calculate_task()
-> MahonyAHRSupdateIMU()
-> q0 q1 q2 q3
-> main_Task_1ms() (1kHz)
-> Gimbal.Update_Imu_Pose_Relative_World()
-> 世界系连续 Yaw/Pitch
-> PID 角度环
-> 速度目标低通(可选)
-> PID 速度环
-> 前馈/重力补偿(可选)
-> DJI6020 电机电流/电压指令
-> CAN2 发给 Yaw/Pitch 电机
```

对应代码位置：

- BMI088 配置为 `ACC 200Hz`、`GYRO 200Hz`  
  `Usercode/2_module/bmi088/bmi088.cpp:91-136`
- Mahony 固定采样频率 `sampleFreq = 200Hz`  
  `Usercode/2_module/Alg/MahonyAHRS/MahonyAHRS.c:23-32`
- BMI088 解算任务在陀螺数据到齐后更新四元数  
  `Usercode/3_application/bmi088/app_bmi088.cpp:428-466`
- 主控 1ms 任务更新云台世界系姿态并执行 PID  
  `Usercode/3_application/Gimbal_Task/Gimbal_Task.cpp:295-423`
- FreeRTOS 中 `main_Task_1khz` 的调度入口  
  `Core/Src/freertos.c:57-76, 135-146`

### 2.2 当前每个环节的真实含义

#### IMU 姿态解算

BMI088 的 ACC / GYRO 原始数据经过零偏校准、单位转换后，进入 `MahonyAHRSupdateIMU()`。
这里没有磁力计，因此：

- `roll` / `pitch` 可由重力方向约束
- `yaw` 没有绝对观测量，只能靠陀螺积分维持

也就是说，当前项目里的“世界系 yaw”是一个局部参考系里的航向角，不是朝北角。

相关代码：

- `bmi088_mahony()` 最终调用的是 `MahonyAHRSupdateIMU()`，而不是带磁力计版本  
  `Usercode/3_application/bmi088/bmi088_math.cpp:294-327`
- 四元数全局量 `q0 q1 q2 q3` 定义在 Mahony 模块  
  `Usercode/2_module/Alg/MahonyAHRS/MahonyAHRS.c:30-33`

#### 主控制任务

`main_Task_1ms()` 每 1ms 执行一次，但 IMU 姿态本身只在 200Hz 更新，所以：

- 控制环标称运行频率是 1kHz
- 姿态角反馈的有效更新频率更接近 200Hz
- 中间 4 个 1ms 周期里，控制环会重复使用上一帧四元数

这点要记住，因为它解释了为什么角度环和速度环的带宽不能盲目拉高。

## 3. 坐标系定义

## 3.1 IMU 原始坐标系 `ImuRaw`

四元数转旋转矩阵后，直接对应的是 IMU 原始安装坐标系 `ImuRaw`：

$v_world = R_World_From_ImuRaw_Measurement * v_imuRaw$

代码：

- `Usercode/3_application/Gimbal/Gimbal.h:143-153`
- `Usercode/3_application/Gimbal/Gimbal.cpp:498-500`

## 3.2 IMU 虚拟坐标系 `ImuVirtual`

项目没有直接把 IMU 原始轴当成云台前向轴，而是额外定义了一个“虚拟坐标系”：

```text
Virtual_X = -Raw_Y
Virtual_Y = +Raw_X
Virtual_Z = +Raw_Z
```

对应矩阵：

```text
R_ImuVirtual_From_ImuRaw =
[ 0 -1  0 ]
[ 1  0  0 ]
[ 0  0  1 ]

R_ImuRaw_From_ImuVirtual =
[ 0  1  0 ]
[-1  0  0 ]
[ 0  0  1 ]
```

代码：

- `Usercode/3_application/Gimbal/Gimbal.cpp:110-139`

这说明当前工程希望使用的云台/机体坐标系是：

- `+X`: 前方
- `+Y`: 左方
- `+Z`: 上方

这是一个右手系，并且 IMU 物理安装相对该坐标系绕 `Z` 轴转了 90 度。

这一点和 `app_bmi088.cpp` 里另外一套“前向轴定义”是一致的：

- 前向轴取 `(0, -1, 0)`，也就是 `Raw_-Y`
- 上方向取 `(0, 0, 1)`，也就是 `Raw_+Z`

见：

- `Usercode/3_application/bmi088/app_bmi088.cpp:379-388`

## 3.3 世界坐标系 `World`

当前工程里的世界系是 Mahony 辅助参考系：

- `+Z_world` 与重力方向对齐
- `yaw=0` 的方向由滤波器初始状态决定
- 没有磁北约束

所以这个世界系更准确地说是：

`重力对齐 + 航向任意的局部惯性参考系`

## 3.4 启动基座坐标系 `BaseStart`

`BaseStart` 是系统启动后一小段延时之后，用当时电机位置和 IMU 实测姿态共同建立的“启动时基座坐标系”。

建立完成后：

- 之后所有 `Relative_BaseStart` 的量，都是“相对于启动时基座”的姿态
- 这是你后面做“云台相对底盘角度”最接近工程语义的一套定义

代码：

- `Usercode/3_application/Gimbal/Gimbal.cpp:636-668`

## 3.5 `BaseCurrent` 在当前代码里的真实含义

`Imu_Relative_BaseCurrent_Model_*` 这组变量名字看起来像“相对当前基座”，
但当前实现里它只是直接复制了 `BaseStart` 模型链路的结果：

- `Imu_Relative_BaseCurrent_Model_Yaw = Model_Relative_BaseStart_Yaw`
- `Imu_Relative_BaseCurrent_Model_Pitch = Model_Relative_BaseStart_Pitch`

见：

- `Usercode/3_application/Gimbal/Gimbal.cpp:592-610`

所以当前代码中的 `BaseCurrent_Model` 并不是一个独立建模后的“当前基座坐标系”，只是名字上更像如此。

## 4. 姿态数学关系

## 4.1 四元数到旋转矩阵

当前工程先把 `q0 q1 q2 q3` 归一化，再转成 3x3 旋转矩阵：

```text
W = q0 / ||q||
X = q1 / ||q||
Y = q2 / ||q||
Z = q3 / ||q||
```

```text
R00 = 1 - 2(Y² + Z²)
R01 = 2(XY - WZ)
R02 = 2(XZ + WY)

R10 = 2(XY + WZ)
R11 = 1 - 2(X² + Z²)
R12 = 2(YZ - WX)

R20 = 2(XZ - WY)
R21 = 2(YZ + WX)
R22 = 1 - 2(X² + Y²)
```

代码：

- `Usercode/3_application/Gimbal/Gimbal.cpp:371-399`

## 4.2 原始 IMU 到虚拟 IMU

当前实测姿态矩阵链路是：

```text
R_World_From_ImuVirtual_Measurement
= R_World_From_ImuRaw_Measurement * R_ImuRaw_From_ImuVirtual
```

代码：

- `Usercode/3_application/Gimbal/Gimbal.cpp:507-512`

这一步的作用，就是把传感器安装坐标修正成“云台前向坐标”。

## 4.3 从旋转矩阵提取 yaw / pitch / roll

`Class_Gimbal::RotationMatrix_To_YawPitchRoll()` 采用的提取方式是：

1. 取旋转矩阵第一列作为前向轴 `f`
2. 取第三列作为上方向轴 `u`
3. 由前向轴先解 `yaw` / `pitch`
4. 再由上方向轴相对参考上方向解 `roll`

代码里写得很清楚：

- 第一列 `f = (fx, fy, fz)`  
  `Usercode/3_application/Gimbal/Gimbal.cpp:411-414`
- 第三列 `u = (ux, uy, uz)`  
  `Usercode/3_application/Gimbal/Gimbal.cpp:417-419`

提取公式是：

```text
yaw_up_positive_left = atan2(fy, fx)
pitch_up_positive    = atan2(fz, sqrt(fx^2 + fy^2))
```

代码：

- `Usercode/3_application/Gimbal/Gimbal.cpp:421-423`

这套公式对应的约定，在头文件注释里已经写明：

- 坐标系：`X 前, Y 左, Z 上`
- `Yaw` 正方向：向左转为正
- `Pitch` 正方向：抬头为正

见：

- `Usercode/3_application/Gimbal/Gimbal.h:324-338`

## 4.4 世界系 pitch 为什么又被取反

虽然 `RotationMatrix_To_YawPitchRoll()` 给出的 `pitch` 是“上仰为正”，
但世界系链路最终又做了一次：

```cpp
Update_Angle_State(&Imu_Relative_World_Pitch_State, -Pitch);
```

代码：

- `Usercode/3_application/Gimbal/Gimbal.cpp:713-723`

因此当前实际用于控制的世界系 pitch 定义是：

```text
pitch_world_positive_down = -pitch_up_positive
                          = atan2(-fz, sqrt(fx^2 + fy^2))
```

也就是说：

- `pitch > 0`：下俯
- `pitch < 0`：上仰

这和 `bmi088_math.cpp` 中 `ForwardUp_To_YawPitchRoll_Deg()` 的定义完全一致：

```text
yaw   = atan2(f_world.y, f_world.x)
pitch = atan2(-f_world.z, sqrt(f_world.x^2 + f_world.y^2))
```

见：

- `Usercode/3_application/bmi088/bmi088_math.cpp:675-735`

所以就“当前实际运行链路”而言，世界系 pitch 的工程约定是统一的：`下俯为正`。

## 4.5 yaw / pitch 的几何关系

对当前工程而言，`yaw` 和 `pitch` 不是两个彼此独立的平面角，而是同一个前向单位向量 `f` 的两种投影描述：

如果定义世界系下的前向向量为：

```text
f = [fx, fy, fz]^T
```

则有：

```text
fx^2 + fy^2 + fz^2 = 1
```

当前世界系约定下：

```text
yaw   = atan2(fy, fx)
pitch = atan2(-fz, sqrt(fx^2 + fy^2))
```

反过来，若已知 `yaw` 和当前工程定义的 `pitch_down_positive`，则前向向量可写成：

```text
fx = cos(yaw) * cos(pitch)
fy = sin(yaw) * cos(pitch)
fz = -sin(pitch)
```

所以它们的本质关系是：

- `yaw` 决定前向向量在水平面的方位角
- `pitch` 决定前向向量离开水平面的俯仰量
- 两者共享同一个前向向量，因此几何上耦合

但在控制实现上，项目把它们拆成了两个独立 SISO 环：

- `Yaw PID`
- `Pitch PID`

没有做显式解耦控制。

## 4.6 连续角的展开方式

多个地方都用了同一种跨圈展开逻辑：

```text
delta = raw - last_raw
if delta < -180: angle_count++
if delta >  180: angle_count--
continuous = raw + angle_count * 360
```

代码：

- 通用角状态更新：`Usercode/3_application/Gimbal/Gimbal.cpp:328-360`
- IMU BaseStart 状态更新：`Usercode/3_application/Gimbal/Gimbal.cpp:229-306`

所以：

- `Raw_Angle` 是包角，范围近似 `(-180, 180]` 或单圈角
- `Continuous_Angle` 是跨圈展开后的连续角

当前世界系反馈使用的就是连续角：

- `Get_Imu_Relative_World_Continuous_Yaw()`
- `Get_Imu_Relative_World_Continuous_Pitch()`

定义见：

- `Usercode/3_application/Gimbal/Gimbal.h:678-718`

## 5. 电机模型链路的数学关系

这一部分代码已经写好，但当前没接入主控制环。

## 5.1 电机反馈量

DJI 6020 的反馈定义：

- `Get_Angle()`  
  `RawAngle * 360 / 8192`，单位 `deg`
- `Get_AngleSpeed()`  
  `Speed_Rpm * 2π / 60`，单位 `rad/s`

代码：

- `Usercode/2_module/DJI_Motor/DJI_Motor.h:117-123`
- `Usercode/2_module/DJI_Motor/DJI_Motor.cpp:221-228`

本项目中：

- `Pitch` 电机是 6020，ID=`4`
- `Yaw` 电机是 6020，ID=`2`
- 都挂在 `CAN2`

代码：

- `Usercode/3_application/Gimbal_Task/Gimbal_Task.cpp:230-235`

## 5.2 模型链路里的 yaw / pitch

模型链路里定义：

```text
Yaw_model_continuous =
    (YawMotor_continuous - YawMotor_startup_zero) * Yaw_Motor_Direction

Pitch_model_continuous =
    wrap180(PitchMotor_raw - Pitch_Mechanical_Zero_Angle) * Pitch_Motor_Direction
```

代码：

- `Usercode/3_application/Gimbal/Gimbal.cpp:518-531`

含义分别是：

- `Yaw`：用编码器连续角减去启动零位，所以是“相对启动时”的多圈角
- `Pitch`：用单圈编码器角减去机械零位，因此是“相对机械水平位”的单圈角

其中：

- `Pitch_Mechanical_Zero_Angle` 默认在复位时被设成 `64.0 deg`
- 其物理意义是“云台 pitch 物理水平时，电机编码器应对应的角度”

代码：

- 默认值：`Usercode/3_application/Gimbal/Gimbal.cpp:183-188`
- 成员定义：`Usercode/3_application/Gimbal/Gimbal.h:135-141`

## 5.3 机构模型旋转矩阵

模型链路把 yaw / pitch 转成旋转矩阵时，用的是：

```text
R_BaseStart_From_ImuVirtual_Model
= Rz(yaw) * Ry(-pitch) * Rx(roll)
```

当前 `roll = 0`

代码：

- `Usercode/3_application/Gimbal/Gimbal.cpp:537-575`

注意这里是 `Ry(-pitch)`，不是 `Ry(pitch)`。  
这等价于让“模型 pitch 正号”对应“前向轴向上抬头”。

验证很简单：若 `yaw=0`，`roll=0`，只看 `pitch>0`，则前向向量变成：

```text
f = [cos(pitch), 0, sin(pitch)]
```

其 `z` 分量为正，所以这套模型链路里的 `pitch` 是“上仰为正”。

这和第 4 节说的世界系 pitch（下俯为正）刚好相反。

## 5.4 启动基座坐标系的初始化

当系统启动一段时间以后，代码会记录当前电机角度为启动零位，并根据当前 IMU 实测姿态和模型姿态共同建立：

```text
R_World_From_BaseStart
```

公式是：

```text
R_World_From_BaseStart
= R_World_From_ImuVirtual_Measurement
 * transpose(R_BaseStart_From_ImuVirtual_Model)
```

代码：

- `Usercode/3_application/Gimbal/Gimbal.cpp:636-668`

工程含义：

- “此时此刻电机模型认为自己是什么姿态”
- “此时此刻 IMU 实际测到自己是什么姿态”

把两者对齐之后，`BaseStart` 就被定义出来了。

## 5.5 真值链路：IMU 相对 BaseStart 的姿态

建立好 `BaseStart` 后，代码再实时计算：

```text
R_BaseStart_From_ImuVirtual_True
= transpose(R_World_From_BaseStart)
 * R_World_From_ImuVirtual_Measurement
```

代码：

- `Usercode/3_application/Gimbal/Gimbal.cpp:674-682`

然后再从这个矩阵解出：

- `Imu_Relative_BaseStart_Yaw`
- `Imu_Relative_BaseStart_Pitch`
- `Imu_Relative_BaseStart_Roll`

代码：

- `Usercode/3_application/Gimbal/Gimbal.cpp:699-707`

这里没有对 `Pitch` 再取反，所以这套 `BaseStart` 链路里的 pitch 保持的是：

`上仰为正`

## 6. 当前控制环里 pitch / yaw 分别怎么走

## 6.1 当前角度反馈

当前 PID 角度反馈使用的是：

- `Yaw`：`Gimbal.Get_Imu_Relative_World_Continuous_Yaw()`
- `Pitch`：`Gimbal.Get_Imu_Relative_World_Continuous_Pitch()`

代码：

- `Usercode/3_application/Gimbal_Task/Gimbal_Task.cpp:379-383`

所以当前控制闭环实际上是：

- 外环角度：IMU 世界系角度
- 内环速度：电机转速

这是一种“混合传感器级联控制”。

## 6.2 当前速度反馈

当前速度反馈使用的是电机反馈：

- `DJI_Motor_Yaw.Get_AngleSpeed()`
- `DJI_Motor_Pitch.Get_AngleSpeed()`

代码：

- `Usercode/3_application/Gimbal_Task/Gimbal_Task.cpp:747-750`

因此：

- 角度环感知的是“云台实际朝向”
- 速度环感知的是“电机轴速度”

## 6.3 目标是怎么来的

### 自瞄模式

当前代码是：

```cpp
float Pitch_Target = Vision.Get_Pitch();
float Yaw_Target = -Vision.Get_Yaw();
```

然后：

```cpp
PID_Gimbal_Motor_Pitch.Set_Angle_Target(limit(Pitch_Target, -40, 40));
PID_Gimbal_Motor_Yaw.Set_Angle_Target(limit(Yaw_Target, -180, 180));
```

代码：

- `Usercode/3_application/Gimbal_Task/Gimbal_Task.cpp:336-349`

这里有两个非常重要的点：

1. `Yaw` 目标被取反，`Pitch` 没取反  
   这说明视觉定义的 yaw 正方向和本工程 yaw 正方向相反，而 pitch 正方向基本一致。

2. 当前代码直接把 `Vision.Get_Yaw/Pitch()` 当成绝对目标角  
   之前“当前角度 + 视觉增量”的写法还保留在注释里：
   `Usercode/3_application/Gimbal_Task/Gimbal_Task.cpp:338-345`

因此可以做一个很强的工程推断：

- 上位机视觉端现在返回的不是“误差增量角”，而是“目标绝对角”
- 并且这个绝对角大概率和下位机发回去的世界系 yaw/pitch 是同一种语义

这里是“推断”，不是代码里显式写死的协议说明。

### 哨兵模式

哨兵模式目标是正弦扫描：

```text
Yaw_target   = A_yaw   * sin(2π f_yaw t)
Pitch_target = A_pitch * sin(2π f_pitch t + phase)
```

代码：

- `Usercode/3_application/Gimbal_Task/Gimbal_Task.cpp:577-602`

当前参数默认值：

- `Yaw_f = 0.8`
- `Yaw_a = 50`
- `Pitch_f = 0.4`
- `Pitch_a = 38`

定义位置：

- `Usercode/3_application/Gimbal_Task/Gimbal_Task.cpp:45-58`

### Pitch 重力补偿采集模式

该模式下：

- `Yaw` 锁定当前世界系角度
- `Pitch` 按 `-40 -> +40` 每 5 度阶梯扫描

代码：

- `Usercode/3_application/Gimbal_Task/Gimbal_Task.cpp:500-559`

## 6.4 PID 结构

当前 PID 是串级 PID：

```text
角度环:  Angle_Target - Current_Angle -> Speed_Target
速度环:  Speed_Target - Current_Speed -> Out
```

代码：

- `Usercode/2_module/Alg/PID/PID.cpp:40-66`

主任务里还可以选择是否在角度环输出和速度环输入之间插一个一阶低通：

```text
y[k] = alpha * x[k] + (1 - alpha) * y[k-1]
```

代码：

- 调用位置：`Usercode/3_application/Gimbal_Task/Gimbal_Task.cpp:387-399`
- 低通实现：`Usercode/2_module/Alg/LowPassFilter/LowPassFilter.cpp:20-47`

## 6.5 最终输出

当前最终输出为：

- `yaw_out = PID_yaw_out + 摩擦前馈(可选)`
- `pitch_out = PID_pitch_out + 重力补偿(可选)`

代码：

- `Usercode/3_application/Gimbal_Task/Gimbal_Task.cpp:769-803`

其中 pitch 的重力补偿是一个拟合函数：

```text
pitch_comp = 1419.6 * sin(theta + 0.661) - 1032.5
theta = pitch_deg * π / 180
```

代码：

- `Usercode/3_application/Gimbal_Task/Gimbal_Task.cpp:488-492`

注意这里用的是：

- `Gimbal.Get_Imu_Relative_World_Continuous_Pitch()`

也就是“世界系 pitch，且下俯为正”的定义。

## 7. 当前项目里每个量到底是什么

### Yaw

- 电机单圈角：`DJI_Motor_Yaw.Get_Angle()`  
  来源于 13bit 编码器单圈角，单位 `deg`
- 电机速度：`DJI_Motor_Yaw.Get_AngleSpeed()`  
  来源于电机反馈转速，单位 `rad/s`
- 当前角度反馈：`Gimbal.Get_Imu_Relative_World_Continuous_Yaw()`  
  来源于 IMU 世界系连续角，单位 `deg`
- 当前目标角：`PID_Gimbal_Motor_Yaw.Angle_Target`  
  当前由视觉绝对角或哨兵函数给出，单位 `deg`
- 电机输出：`DJI_Motor_Yaw.Set_Out(yaw_out)`  
  经 CAN2 发给 ID2 的 6020

### Pitch

- 电机单圈角：`DJI_Motor_Pitch.Get_Angle()`  
  来源于 13bit 编码器单圈角，单位 `deg`
- 电机速度：`DJI_Motor_Pitch.Get_AngleSpeed()`  
  来源于电机反馈转速，单位 `rad/s`
- 当前角度反馈：`Gimbal.Get_Imu_Relative_World_Continuous_Pitch()`  
  来源于 IMU 世界系连续角，单位 `deg`，`下俯为正`
- 当前目标角：`PID_Gimbal_Motor_Pitch.Angle_Target`  
  当前由视觉绝对角、哨兵函数或重力采集函数给出，单位 `deg`
- 电机输出：`DJI_Motor_Pitch.Set_Out(pitch_out)`  
  经 CAN2 发给 ID4 的 6020

## 8. 需要特别注意的工程问题

## 8.1 当前真正启用的是世界系链路，不是 BaseStart 链路

这意味着：

- 当前控制环完全没用到你写好的“相对启动基座”模型链路
- `Yaw_Motor_Angle_State` / `Pitch_Motor_Angle_State` 对当前闭环没有直接贡献
- 角度外环完全依赖 IMU 世界系姿态

如果你后面想做“底盘转动时云台相对底盘角控制”，那应该重新接回
`Update_Imu_Pose_Relative_BaseStart(...)` 这条链路。

## 8.2 世界系 pitch 和 BaseStart / Model pitch 的正方向不一致

这是当前代码里最容易搞错的地方：

- `World pitch`: 下俯为正
- `BaseStart pitch`: 上仰为正
- `Model pitch`: 上仰为正

所以如果你以后把这两套量混着用，必须先统一符号。

## 8.3 世界系 yaw 不是绝对航向

因为当前 Mahony 没有磁力计输入，所以：

- `roll` / `pitch` 可靠
- `yaw` 只是局部航向，长时间会漂移

因此：

- 若只是做短时自瞄和哨兵扫描，通常可接受
- 若你要做严格“相对地面/绝对方位”的 yaw 控制，这条链路不够

## 8.4 当前 yaw 反馈是连续角，但目标被限制在 `[-180, 180]`

代码里：

- 反馈用 `Get_Imu_Relative_World_Continuous_Yaw()`
- 目标却被 `Limit(..., -180, 180)`

见：

- 反馈：`Usercode/3_application/Gimbal_Task/Gimbal_Task.cpp:379-380`
- 目标限幅：`Usercode/3_application/Gimbal_Task/Gimbal_Task.cpp:348-349`

这说明当前设计默认：

- yaw 不会跨很多圈
- 或视觉返回的目标也总能保持在与反馈一致的单圈范围

如果以后要支持大范围连续旋转，这里必须改成同一语义的连续角目标。

## 8.5 角度环和速度环量纲不同

当前：

- 角度反馈 / 目标：`deg`
- 速度反馈：`rad/s`

这本身没有错，但意味着：

- `Kp_a / Ki_a / Kd_a` 的单位隐含是“每度误差对应多少速度目标”
- 调参时不能把角度环参数和速度环参数混着理解

## 8.6 当前“世界系”与视觉协议的匹配是工程约定，不是代码里严格证明

下位机会把当前 `World Continuous Yaw/Pitch` 发给视觉：

- `Usercode/3_application/Gimbal_Task/Gimbal_Task.cpp:262-271`

同时又把视觉回传角度直接当成目标：

- `Usercode/3_application/Gimbal_Task/Gimbal_Task.cpp:336-349`

所以比较合理的推断是：

- 视觉和下位机正在共用同一种世界系角度语义

但这件事在当前仓库里没有独立协议文档明写，仍建议你确认上位机端实现。

## 9. 如果只看“当前在线运行的数学模型”，可以压缩成下面这组式子

### 9.1 姿态测量链

```text
q -> R_World_From_ImuRaw
R_World_From_ImuVirtual = R_World_From_ImuRaw * R_ImuRaw_From_ImuVirtual

f = 第一列(R_World_From_ImuVirtual)

yaw_world   = atan2(fy, fx)
pitch_world = atan2(-fz, sqrt(fx^2 + fy^2))
```

### 9.2 控制链

```text
e_angle_yaw   = yaw_target   - yaw_world_continuous
e_angle_pitch = pitch_target - pitch_world_continuous

speed_target_yaw   = PID_angle_yaw(e_angle_yaw)
speed_target_pitch = PID_angle_pitch(e_angle_pitch)

out_yaw   = PID_speed_yaw(speed_target_yaw   - motor_speed_yaw)
out_pitch = PID_speed_pitch(speed_target_pitch - motor_speed_pitch)

out_yaw   += friction_feedforward(speed_target_yaw)      // 可选
out_pitch += gravity_compensation(pitch_world_continuous) // 可选
```

### 9.3 当前工程语义

```text
Yaw:   左转为正
Pitch: 下俯为正   (仅限当前实际运行的 World 链路)
Roll:  绕前向轴右手定则为正
```

## 10. 代码定位索引

如果你要继续深挖，最重要的入口文件就是下面这些：

- `Usercode/3_application/Gimbal_Task/Gimbal_Task.cpp`
- `Usercode/3_application/Gimbal/Gimbal.cpp`
- `Usercode/3_application/Gimbal/Gimbal.h`
- `Usercode/3_application/bmi088/app_bmi088.cpp`
- `Usercode/3_application/bmi088/bmi088_math.cpp`
- `Usercode/2_module/Alg/MahonyAHRS/MahonyAHRS.c`
- `Usercode/2_module/DJI_Motor/DJI_Motor.cpp`

---

如果你后面还要继续分析“底盘坐标系、枪口坐标系、视觉坐标系、相机外参和云台两轴机构链如何统一”，建议下一步直接把以下 4 件事一起做：

1. 统一 `pitch` 正方向，只保留一种工程约定。
2. 明确视觉返回的是“绝对角”还是“增量角”。
3. 决定控制到底用 `World` 还是 `BaseStart` 作为主反馈坐标系。
4. 给 yaw 连续角目标建立和反馈一致的语义，避免跨圈后出错。
