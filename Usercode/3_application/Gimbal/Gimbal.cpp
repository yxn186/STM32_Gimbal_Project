/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Gimbal.cpp
  * @brief   云台库
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "Gimbal.h"
#include "MyMath.h"
#include <math.h>


static float Gimbal_Angle_Wrap_To_180(float Angle)
{
    while(Angle > 180.0f)  Angle -= 360.0f;
    while(Angle <= -180.0f) Angle += 360.0f;
    return Angle;
}

static void Gimbal_Matrix3x3_Copy(const float Src[3][3], float Dst[3][3])
{
    for(uint8_t i = 0; i < 3; i++)
    {
        for(uint8_t j = 0; j < 3; j++)
        {
            Dst[i][j] = Src[i][j];
        }
    }
}

static void Gimbal_Matrix3x3_Transpose(const float Src[3][3], float Dst[3][3])
{
    for(uint8_t i = 0; i < 3; i++)
    {
        for(uint8_t j = 0; j < 3; j++)
        {
            Dst[i][j] = Src[j][i];
        }
    }
}

static void Gimbal_Matrix3x3_Multiply(const float A[3][3], const float B[3][3], float C[3][3])
{
    for(uint8_t i = 0; i < 3; i++)
    {
        for(uint8_t j = 0; j < 3; j++)
        {
            C[i][j] = A[i][0] * B[0][j]
                    + A[i][1] * B[1][j]
                    + A[i][2] * B[2][j];
        }
    }
}

static float Gimbal_Vector3_Dot(float ax, float ay, float az, float bx, float by, float bz)
{
    return ax * bx + ay * by + az * bz;
}

static float Gimbal_Vector3_Norm(float x, float y, float z)
{
    return sqrtf(x * x + y * y + z * z);
}

/**
 * @brief IMU原始坐标系 -> IMU虚拟坐标系（前x 左y 上z）
 * 
 * 当前固定关系：
 * Virtual_X = Raw_-Y
 * Virtual_Y = Raw_+X
 * Virtual_Z = Raw_+Z
 * 
 * 含义：
 * v_imuVirtual = R_ImuVirtual_From_ImuRaw * v_imuRaw
 */
static const float R_ImuVirtual_From_ImuRaw[3][3] =
{
    { 0.0f, -1.0f,  0.0f },
    { 1.0f,  0.0f,  0.0f },
    { 0.0f,  0.0f,  1.0f }
};

/**
 * @brief IMU虚拟坐标系 -> IMU原始坐标系
 * 
 * 含义：
 * v_imuRaw = R_ImuRaw_From_ImuVirtual * v_imuVirtual
 */
static const float R_ImuRaw_From_ImuVirtual[3][3] =
{
    { 0.0f,  1.0f,  0.0f },
    {-1.0f,  0.0f,  0.0f },
    { 0.0f,  0.0f,  1.0f }
};



/**
 * @brief 复位“IMU相对于启动时基座”相关状态
 *
 */
void Class_Gimbal::Reset_Imu_Relative_BaseStart_State(void)
{
    Yaw_Motor_Angle_State = Struct_Gimbal_Angle_State_t();
    Pitch_Motor_Angle_State = Struct_Gimbal_Angle_State_t();

    Imu_Relative_BaseStart_Yaw_State = Struct_Gimbal_Angle_State_t();
    Imu_Relative_BaseStart_Pitch_State = Struct_Gimbal_Angle_State_t();
    Imu_Relative_BaseStart_Roll_State = Struct_Gimbal_Angle_State_t();

    Imu_Yaw = 0.0f;
    Imu_Pitch = 0.0f;
    Imu_Roll = 0.0f;

    Model_Relative_BaseStart_Yaw = 0.0f;
    Model_Relative_BaseStart_Pitch = 0.0f;
    Model_Relative_BaseStart_Roll = 0.0f;

    Model_Relative_BaseStart_Continuous_Yaw = 0.0f;
    Model_Relative_BaseStart_Continuous_Pitch = 0.0f;
    Model_Relative_BaseStart_Continuous_Roll = 0.0f;

    Target_Imu_Relative_BaseStart_Continuous_Yaw = 0.0f;
    Target_Imu_Relative_BaseStart_Continuous_Pitch = 0.0f;
    Target_Imu_Relative_BaseStart_Continuous_Roll = 0.0f;

    Imu_Relative_BaseCurrent_Model_Yaw = 0.0f;
    Imu_Relative_BaseCurrent_Model_Pitch = 0.0f;
    Imu_Relative_BaseCurrent_Model_Roll = 0.0f;

    Imu_Relative_BaseCurrent_Model_Continuous_Yaw = 0.0f;
    Imu_Relative_BaseCurrent_Model_Continuous_Pitch = 0.0f;
    Imu_Relative_BaseCurrent_Model_Continuous_Roll = 0.0f;

    BaseStart_World_Initialized = 0U;

    Pitch_Mechanical_Zero_Angle = 64.0f;

    Yaw_Motor_Direction = 1.0f;
    Pitch_Motor_Direction = 1.0f;

    for(uint8_t i = 0; i < 3; i++)
    {
        for(uint8_t j = 0; j < 3; j++)
        {
            R_World_From_ImuRaw_Measurement[i][j] = (i == j) ? 1.0f : 0.0f;
            R_World_From_ImuVirtual_Measurement[i][j] = (i == j) ? 1.0f : 0.0f;
            R_World_From_BaseStart[i][j] = (i == j) ? 1.0f : 0.0f;
            R_BaseStart_From_ImuVirtual_True[i][j] = (i == j) ? 1.0f : 0.0f;
            R_BaseStart_From_ImuRaw_True[i][j] = (i == j) ? 1.0f : 0.0f;
            R_BaseStart_From_ImuVirtual_Model[i][j] = (i == j) ? 1.0f : 0.0f;
            R_BaseStart_From_ImuRaw_Model[i][j] = (i == j) ? 1.0f : 0.0f;
            R_BaseCurrent_From_ImuVirtual_Model[i][j] = (i == j) ? 1.0f : 0.0f;
            R_BaseCurrent_From_ImuRaw_Model[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

/**
 * @brief 设置IMU角度（Yaw + Pitch + Roll）
 *
 * @param Yaw
 * @param Pitch
 * @param Roll
 */
void Class_Gimbal::Set_Imu_Angle(float Yaw, float Pitch, float Roll)
{
    Imu_Yaw = Yaw;
    Imu_Pitch = Pitch;
    Imu_Roll = Roll;

    Imu_Relative_BaseStart_Yaw_State.Raw_Angle = Yaw;
    Imu_Relative_BaseStart_Pitch_State.Raw_Angle = Pitch;
    Imu_Relative_BaseStart_Roll_State.Raw_Angle = Roll;
}

/**
 * @brief 计算IMU角度差值，并自动处理跨圈计数
 *
 */
void Class_Gimbal::Set_Delta_Imu_Angle(void)
{
    if(Imu_Relative_BaseStart_Yaw_State.Initialized == 0U)
    {
        Imu_Relative_BaseStart_Yaw_State.Last_Raw_Angle = Imu_Relative_BaseStart_Yaw_State.Raw_Angle;
        Imu_Relative_BaseStart_Pitch_State.Last_Raw_Angle = Imu_Relative_BaseStart_Pitch_State.Raw_Angle;
        Imu_Relative_BaseStart_Roll_State.Last_Raw_Angle = Imu_Relative_BaseStart_Roll_State.Raw_Angle;

        Imu_Relative_BaseStart_Yaw_State.Delta_Angle = 0.0f;
        Imu_Relative_BaseStart_Pitch_State.Delta_Angle = 0.0f;
        Imu_Relative_BaseStart_Roll_State.Delta_Angle = 0.0f;

        Imu_Relative_BaseStart_Yaw_State.Angle_Count = 0;
        Imu_Relative_BaseStart_Pitch_State.Angle_Count = 0;
        Imu_Relative_BaseStart_Roll_State.Angle_Count = 0;

        Imu_Relative_BaseStart_Yaw_State.Initialized = 1U;
        Imu_Relative_BaseStart_Pitch_State.Initialized = 1U;
        Imu_Relative_BaseStart_Roll_State.Initialized = 1U;
        return;
    }

    Imu_Relative_BaseStart_Yaw_State.Delta_Angle =
        Imu_Relative_BaseStart_Yaw_State.Raw_Angle - Imu_Relative_BaseStart_Yaw_State.Last_Raw_Angle;

    Imu_Relative_BaseStart_Pitch_State.Delta_Angle =
        Imu_Relative_BaseStart_Pitch_State.Raw_Angle - Imu_Relative_BaseStart_Pitch_State.Last_Raw_Angle;

    Imu_Relative_BaseStart_Roll_State.Delta_Angle =
        Imu_Relative_BaseStart_Roll_State.Raw_Angle - Imu_Relative_BaseStart_Roll_State.Last_Raw_Angle;

    if(Imu_Relative_BaseStart_Yaw_State.Delta_Angle < -180.0f)
    {
        Imu_Relative_BaseStart_Yaw_State.Angle_Count++;
    }
    else if(Imu_Relative_BaseStart_Yaw_State.Delta_Angle > 180.0f)
    {
        Imu_Relative_BaseStart_Yaw_State.Angle_Count--;
    }

    if(Imu_Relative_BaseStart_Pitch_State.Delta_Angle < -180.0f)
    {
        Imu_Relative_BaseStart_Pitch_State.Angle_Count++;
    }
    else if(Imu_Relative_BaseStart_Pitch_State.Delta_Angle > 180.0f)
    {
        Imu_Relative_BaseStart_Pitch_State.Angle_Count--;
    }

    if(Imu_Relative_BaseStart_Roll_State.Delta_Angle < -180.0f)
    {
        Imu_Relative_BaseStart_Roll_State.Angle_Count++;
    }
    else if(Imu_Relative_BaseStart_Roll_State.Delta_Angle > 180.0f)
    {
        Imu_Relative_BaseStart_Roll_State.Angle_Count--;
    }

    Imu_Relative_BaseStart_Yaw_State.Last_Raw_Angle = Imu_Relative_BaseStart_Yaw_State.Raw_Angle;
    Imu_Relative_BaseStart_Pitch_State.Last_Raw_Angle = Imu_Relative_BaseStart_Pitch_State.Raw_Angle;
    Imu_Relative_BaseStart_Roll_State.Last_Raw_Angle = Imu_Relative_BaseStart_Roll_State.Raw_Angle;
}

/**
 * @brief 根据当前包角和圈数计数，计算IMU连续角度
 *
 */
void Class_Gimbal::Set_Imu_Continuous_Angle(void)
{
    Imu_Relative_BaseStart_Yaw_State.Continuous_Angle =
        Imu_Relative_BaseStart_Yaw_State.Raw_Angle + (float)Imu_Relative_BaseStart_Yaw_State.Angle_Count * 360.0f;

    Imu_Relative_BaseStart_Pitch_State.Continuous_Angle =
        Imu_Relative_BaseStart_Pitch_State.Raw_Angle + (float)Imu_Relative_BaseStart_Pitch_State.Angle_Count * 360.0f;

    Imu_Relative_BaseStart_Roll_State.Continuous_Angle =
        Imu_Relative_BaseStart_Roll_State.Raw_Angle + (float)Imu_Relative_BaseStart_Roll_State.Angle_Count * 360.0f;
}

/**
 * @brief 更新IMU角度（Yaw + Pitch + Roll），并自动完成差值与连续角计算
 *
 * @param Yaw
 * @param Pitch
 * @param Roll
 */
void Class_Gimbal::Update_Imu_Angle(float Yaw, float Pitch, float Roll)
{
    Set_Imu_Angle(Yaw, Pitch, Roll);
    Set_Delta_Imu_Angle();
    Set_Imu_Continuous_Angle();
}

/**
 * @brief 更新单个角度状态（连续角）
 *
 * @param Angle_State
 * @param Raw_Angle
 */
void Class_Gimbal::Update_Angle_State(Struct_Gimbal_Angle_State_t *Angle_State, float Raw_Angle)
{
    if(Angle_State == nullptr)
    {
        return;
    }

    Angle_State->Raw_Angle = Raw_Angle;

    if(Angle_State->Initialized == 0U)
    {
        Angle_State->Last_Raw_Angle = Raw_Angle;
        Angle_State->Delta_Angle = 0.0f;
        Angle_State->Continuous_Angle = Raw_Angle;
        Angle_State->Angle_Count = 0;
        Angle_State->Initialized = 1U;
        return;
    }

    Angle_State->Delta_Angle = Raw_Angle - Angle_State->Last_Raw_Angle;

    if(Angle_State->Delta_Angle < -180.0f)
    {
        Angle_State->Angle_Count++;
    }
    else if(Angle_State->Delta_Angle > 180.0f)
    {
        Angle_State->Angle_Count--;
    }

    Angle_State->Continuous_Angle = Raw_Angle + (float)Angle_State->Angle_Count * 360.0f;
    Angle_State->Last_Raw_Angle = Raw_Angle;
}

/**
 * @brief 四元数转旋转矩阵
 *
 * @param Q0
 * @param Q1
 * @param Q2
 * @param Q3
 * @param R
 */
void Class_Gimbal::Quaternion_To_RotationMatrix(float Q0, float Q1, float Q2, float Q3, float R[3][3])
{
    float Norm = sqrtf(Q0 * Q0 + Q1 * Q1 + Q2 * Q2 + Q3 * Q3);

    if(Norm <= 1e-6f)
    {
        R[0][0] = 1.0f; R[0][1] = 0.0f; R[0][2] = 0.0f;
        R[1][0] = 0.0f; R[1][1] = 1.0f; R[1][2] = 0.0f;
        R[2][0] = 0.0f; R[2][1] = 0.0f; R[2][2] = 1.0f;
        return;
    }

    float W = Q0 / Norm;
    float X = Q1 / Norm;
    float Y = Q2 / Norm;
    float Z = Q3 / Norm;

    R[0][0] = 1.0f - 2.0f * (Y * Y + Z * Z);
    R[0][1] = 2.0f * (X * Y - W * Z);
    R[0][2] = 2.0f * (X * Z + W * Y);

    R[1][0] = 2.0f * (X * Y + W * Z);
    R[1][1] = 1.0f - 2.0f * (X * X + Z * Z);
    R[1][2] = 2.0f * (Y * Z - W * X);

    R[2][0] = 2.0f * (X * Z - W * Y);
    R[2][1] = 2.0f * (Y * Z + W * X);
    R[2][2] = 1.0f - 2.0f * (X * X + Y * Y);
}

/**
 * @brief 旋转矩阵解算Yaw / Pitch / Roll
 *
 * @param R
 * @param Yaw
 * @param Pitch
 * @param Roll
 */
void Class_Gimbal::RotationMatrix_To_YawPitchRoll(const float R[3][3], float *Yaw, float *Pitch, float *Roll)
{
    /* 第一列 = 机体系 X轴（前向轴）在参考系中的方向 */
    float fx = R[0][0];
    float fy = R[1][0];
    float fz = R[2][0];

    /* 第三列 = 机体系 Z轴（上方向）在参考系中的方向 */
    float ux = R[0][2];
    float uy = R[1][2];
    float uz = R[2][2];

    float Yaw_Rad = atan2f(fy, fx);
    float Pitch_Rad = atan2f(fz, sqrtf(fx * fx + fy * fy));

    float Roll_Rad = 0.0f;

    /* 参考上方向 = +Z */
    float Ref_Up_X = 0.0f;
    float Ref_Up_Y = 0.0f;
    float Ref_Up_Z = 1.0f;

    float Dot_Ref_F = Gimbal_Vector3_Dot(Ref_Up_X, Ref_Up_Y, Ref_Up_Z, fx, fy, fz);

    float Proj_Ref_X = Ref_Up_X - Dot_Ref_F * fx;
    float Proj_Ref_Y = Ref_Up_Y - Dot_Ref_F * fy;
    float Proj_Ref_Z = Ref_Up_Z - Dot_Ref_F * fz;

    float Proj_Ref_Norm = Gimbal_Vector3_Norm(Proj_Ref_X, Proj_Ref_Y, Proj_Ref_Z);

    if(Proj_Ref_Norm < 1e-6f)
    {
        Ref_Up_X = 0.0f;
        Ref_Up_Y = 1.0f;
        Ref_Up_Z = 0.0f;

        Dot_Ref_F = Gimbal_Vector3_Dot(Ref_Up_X, Ref_Up_Y, Ref_Up_Z, fx, fy, fz);

        Proj_Ref_X = Ref_Up_X - Dot_Ref_F * fx;
        Proj_Ref_Y = Ref_Up_Y - Dot_Ref_F * fy;
        Proj_Ref_Z = Ref_Up_Z - Dot_Ref_F * fz;

        Proj_Ref_Norm = Gimbal_Vector3_Norm(Proj_Ref_X, Proj_Ref_Y, Proj_Ref_Z);
    }

    if(Proj_Ref_Norm > 1e-6f)
    {
        Proj_Ref_X /= Proj_Ref_Norm;
        Proj_Ref_Y /= Proj_Ref_Norm;
        Proj_Ref_Z /= Proj_Ref_Norm;
    }

    float Dot_U_F = Gimbal_Vector3_Dot(ux, uy, uz, fx, fy, fz);

    float Proj_U_X = ux - Dot_U_F * fx;
    float Proj_U_Y = uy - Dot_U_F * fy;
    float Proj_U_Z = uz - Dot_U_F * fz;

    float Proj_U_Norm = Gimbal_Vector3_Norm(Proj_U_X, Proj_U_Y, Proj_U_Z);

    if(Proj_U_Norm > 1e-6f)
    {
        Proj_U_X /= Proj_U_Norm;
        Proj_U_Y /= Proj_U_Norm;
        Proj_U_Z /= Proj_U_Norm;

        float Cross_X = Proj_Ref_Y * Proj_U_Z - Proj_Ref_Z * Proj_U_Y;
        float Cross_Y = Proj_Ref_Z * Proj_U_X - Proj_Ref_X * Proj_U_Z;
        float Cross_Z = Proj_Ref_X * Proj_U_Y - Proj_Ref_Y * Proj_U_X;

        float Sin_Roll = Gimbal_Vector3_Dot(fx, fy, fz, Cross_X, Cross_Y, Cross_Z);
        float Cos_Roll = Gimbal_Vector3_Dot(Proj_Ref_X, Proj_Ref_Y, Proj_Ref_Z, Proj_U_X, Proj_U_Y, Proj_U_Z);

        Roll_Rad = atan2f(Sin_Roll, Cos_Roll);
    }

    if(Yaw != nullptr)   *Yaw = Yaw_Rad * RAD2DEG;
    if(Pitch != nullptr) *Pitch = Pitch_Rad * RAD2DEG;
    if(Roll != nullptr)  *Roll = Roll_Rad * RAD2DEG;
}

/**
 * @brief 计算IMU原始坐标系在世界系中的实测旋转矩阵
 *
 * @param Q0
 * @param Q1
 * @param Q2
 * @param Q3
 */
void Class_Gimbal::Set_R_World_From_ImuRaw_Measurement(float Q0, float Q1, float Q2, float Q3)
{
    Quaternion_To_RotationMatrix(Q0, Q1, Q2, Q3, R_World_From_ImuRaw_Measurement);
}

/**
 * @brief 计算IMU虚拟坐标系在世界系中的实测旋转矩阵
 *
 */
void Class_Gimbal::Set_R_World_From_ImuVirtual_Measurement(void)
{
    Gimbal_Matrix3x3_Multiply(R_World_From_ImuRaw_Measurement,
                              R_ImuRaw_From_ImuVirtual,
                              R_World_From_ImuVirtual_Measurement);
}

/**
 * @brief 计算电机模型参考角度
 *
 */
void Class_Gimbal::Set_Model_Relative_BaseStart_Angle(void)
{
    Model_Relative_BaseStart_Continuous_Yaw =
        (Yaw_Motor_Angle_State.Continuous_Angle - Yaw_Motor_Angle_State.Startup_Zero_Angle) * Yaw_Motor_Direction;

    Model_Relative_BaseStart_Continuous_Pitch =
        Gimbal_Angle_Wrap_To_180(Pitch_Motor_Angle_State.Raw_Angle - Pitch_Mechanical_Zero_Angle) * Pitch_Motor_Direction;

    Model_Relative_BaseStart_Continuous_Roll = 0.0f;

    Model_Relative_BaseStart_Yaw = Gimbal_Angle_Wrap_To_180(Model_Relative_BaseStart_Continuous_Yaw);
    Model_Relative_BaseStart_Pitch = Gimbal_Angle_Wrap_To_180(Model_Relative_BaseStart_Continuous_Pitch);
    Model_Relative_BaseStart_Roll = 0.0f;
}

/**
 * @brief 计算IMU虚拟坐标系相对于启动时基座的模型参考矩阵
 *
 */
void Class_Gimbal::Set_R_BaseStart_From_ImuVirtual_Model(void)
{
    float Yaw_Rad = Model_Relative_BaseStart_Continuous_Yaw * DEG2RAD;
    float Pitch_Rad = Model_Relative_BaseStart_Continuous_Pitch * DEG2RAD;
    float Roll_Rad = Model_Relative_BaseStart_Continuous_Roll * DEG2RAD;

    float cy = cosf(Yaw_Rad);
    float sy = sinf(Yaw_Rad);
    float cp = cosf(Pitch_Rad);
    float sp = sinf(Pitch_Rad);
    float cr = cosf(Roll_Rad);
    float sr = sinf(Roll_Rad);

    /* R = Rz(yaw) * Ry(-pitch) * Rx(roll) */
    float Rz[3][3] =
    {
        { cy, -sy, 0.0f },
        { sy,  cy, 0.0f },
        { 0.0f, 0.0f, 1.0f }
    };

    float Ry_NegPitch[3][3] =
    {
        { cp, 0.0f, -sp },
        { 0.0f, 1.0f, 0.0f },
        { sp, 0.0f, cp }
    };

    float Rx[3][3] =
    {
        { 1.0f, 0.0f, 0.0f },
        { 0.0f, cr, -sr },
        { 0.0f, sr, cr }
    };

    float Temp[3][3];
    Gimbal_Matrix3x3_Multiply(Rz, Ry_NegPitch, Temp);
    Gimbal_Matrix3x3_Multiply(Temp, Rx, R_BaseStart_From_ImuVirtual_Model);
}

/**
 * @brief 计算IMU原始坐标系相对于启动时基座的模型参考矩阵
 *
 */
void Class_Gimbal::Set_R_BaseStart_From_ImuRaw_Model(void)
{
    Gimbal_Matrix3x3_Multiply(R_BaseStart_From_ImuVirtual_Model,
                              R_ImuVirtual_From_ImuRaw,
                              R_BaseStart_From_ImuRaw_Model);
}

/**
 * @brief 计算IMU相对于当前时刻基座的模型角度
 * 
 */
void Class_Gimbal::Set_Imu_Relative_BaseCurrent_Model_Angle(void)
{
    /**
     * 这里为什么可以直接复制 BaseStart Model？
     * 
     * 因为你当前这条 Model 链本来就没有使用世界系矩阵，
     * 它只用了电机角 + 机构链。
     * 
     * 所以从控制语义上，它本来就是“相对当前基座”的模型角，
     * 只是你之前名字里写了 BaseStart，容易让人误解。
     */
    Imu_Relative_BaseCurrent_Model_Yaw = Model_Relative_BaseStart_Yaw;
    Imu_Relative_BaseCurrent_Model_Pitch = Model_Relative_BaseStart_Pitch;
    Imu_Relative_BaseCurrent_Model_Roll = Model_Relative_BaseStart_Roll;

    Imu_Relative_BaseCurrent_Model_Continuous_Yaw = Model_Relative_BaseStart_Continuous_Yaw;
    Imu_Relative_BaseCurrent_Model_Continuous_Pitch = Model_Relative_BaseStart_Continuous_Pitch;
    Imu_Relative_BaseCurrent_Model_Continuous_Roll = Model_Relative_BaseStart_Continuous_Roll;
}

/**
 * @brief 计算IMU虚拟坐标系相对于当前时刻基座的模型矩阵
 * 
 */
void Class_Gimbal::Set_R_BaseCurrent_From_ImuVirtual_Model(void)
{
    Gimbal_Matrix3x3_Copy(R_BaseStart_From_ImuVirtual_Model, R_BaseCurrent_From_ImuVirtual_Model);
}

/**
 * @brief 计算IMU原始坐标系相对于当前时刻基座的模型矩阵
 * 
 */
void Class_Gimbal::Set_R_BaseCurrent_From_ImuRaw_Model(void)
{
    Gimbal_Matrix3x3_Copy(R_BaseStart_From_ImuRaw_Model, R_BaseCurrent_From_ImuRaw_Model);
}

/**
 * @brief 尝试建立“启动时基座在世界系中的矩阵”
 *
 * @param Now_Time
 * @param Delay_Time
 */
void Class_Gimbal::Try_Init_BaseStart_World(uint32_t Now_Time, uint32_t Delay_Time)
{
    if(BaseStart_World_Initialized != 0U)
    {
        return;
    }

    if(Now_Time < Delay_Time)
    {
        return;
    }

    if((Yaw_Motor_Angle_State.Initialized == 0U) || (Pitch_Motor_Angle_State.Initialized == 0U))
    {
        return;
    }

    Yaw_Motor_Angle_State.Startup_Zero_Angle = Yaw_Motor_Angle_State.Continuous_Angle;
    Pitch_Motor_Angle_State.Startup_Zero_Angle = Pitch_Motor_Angle_State.Continuous_Angle;

    Set_Model_Relative_BaseStart_Angle();
    Set_R_BaseStart_From_ImuVirtual_Model();

    float R_ImuVirtualModel_From_BaseStart[3][3];
    Gimbal_Matrix3x3_Transpose(R_BaseStart_From_ImuVirtual_Model, R_ImuVirtualModel_From_BaseStart);

    Gimbal_Matrix3x3_Multiply(R_World_From_ImuVirtual_Measurement,
                              R_ImuVirtualModel_From_BaseStart,
                              R_World_From_BaseStart);

    BaseStart_World_Initialized = 1U;
}

/**
 * @brief 计算IMU虚拟坐标系相对于启动时基座的真实矩阵
 *
 */
void Class_Gimbal::Set_R_BaseStart_From_ImuVirtual_True(void)
{
    float R_BaseStart_From_World[3][3];
    Gimbal_Matrix3x3_Transpose(R_World_From_BaseStart, R_BaseStart_From_World);

    Gimbal_Matrix3x3_Multiply(R_BaseStart_From_World,
                              R_World_From_ImuVirtual_Measurement,
                              R_BaseStart_From_ImuVirtual_True);
}

/**
 * @brief 计算IMU原始坐标系相对于启动时基座的真实矩阵
 *
 */
void Class_Gimbal::Set_R_BaseStart_From_ImuRaw_True(void)
{
    Gimbal_Matrix3x3_Multiply(R_BaseStart_From_ImuVirtual_True,
                              R_ImuVirtual_From_ImuRaw,
                              R_BaseStart_From_ImuRaw_True);
}

/**
 * @brief 从“真实相对矩阵”中解算IMU相对于启动时基座的角度
 *
 */
void Class_Gimbal::Set_Imu_Relative_BaseStart_Angle(void)
{
    float Yaw = 0.0f;
    float Pitch = 0.0f;
    float Roll = 0.0f;

    RotationMatrix_To_YawPitchRoll(R_BaseStart_From_ImuVirtual_True, &Yaw, &Pitch, &Roll);
    Update_Imu_Angle(Yaw, Pitch, Roll);
}

/**
 * @brief 总更新函数：同时更新IMU实测链和电机模型链
 *
 * @param Q0
 * @param Q1
 * @param Q2
 * @param Q3
 * @param Yaw_Motor_Raw_Angle
 * @param Pitch_Motor_Raw_Angle
 * @param Now_Time
 * @param Delay_Time
 */
void Class_Gimbal::Update_Imu_Pose_Relative_BaseStart(float Q0,
                                                      float Q1,
                                                      float Q2,
                                                      float Q3,
                                                      float Yaw_Motor_Raw_Angle,
                                                      float Pitch_Motor_Raw_Angle,
                                                      uint32_t Now_Time,
                                                      uint32_t Delay_Time)
{
    Update_Angle_State(&Yaw_Motor_Angle_State, Yaw_Motor_Raw_Angle);
    Update_Angle_State(&Pitch_Motor_Angle_State, Pitch_Motor_Raw_Angle);

    Set_R_World_From_ImuRaw_Measurement(Q0, Q1, Q2, Q3);
    Set_R_World_From_ImuVirtual_Measurement();

    Try_Init_BaseStart_World(Now_Time, Delay_Time);

    if(BaseStart_World_Initialized == 0U)
    {
        return;
    }

    Set_Model_Relative_BaseStart_Angle();
    Set_R_BaseStart_From_ImuVirtual_Model();
    Set_R_BaseStart_From_ImuRaw_Model();

    Set_Imu_Relative_BaseCurrent_Model_Angle();
    Set_R_BaseCurrent_From_ImuVirtual_Model();
    Set_R_BaseCurrent_From_ImuRaw_Model();

    Set_R_BaseStart_From_ImuVirtual_True();
    Set_R_BaseStart_From_ImuRaw_True();

    Set_Imu_Relative_BaseStart_Angle();
}

