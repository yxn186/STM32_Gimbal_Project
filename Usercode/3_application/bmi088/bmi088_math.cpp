/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    bmi088_math.c
  * @brief   bmi088解算
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "bmi088_math.h"
#include <math.h>
#include "MyMath.h"
#include <stdint.h>
#include "MahonyAHRS.h"

/* ===================== 量程/灵敏度 ===================== */
#define BMI088_GYRO_LSB_PER_DPS_2000   (16.384f)   /* ±2000 dps */
#define BMI088_ACC_LSB_PER_G_6G        (5460.0f)   /* ±6g */



/* 加速度幅值门限（运动/震动时不使用 acc 修正） */
#define BMI088_ACC_NORM_MIN_G          (0.70f)
#define BMI088_ACC_NORM_MAX_G          (1.30f)
//↑把ACC数据归一化 来框限acc参考


/* ===================== 姿态输出（单位：度） ===================== */
static float bmi088_roll_angle_deg        = 0.0f;
static float bmi088_pitch_angle_deg       = 0.0f;
static float bmi088_yaw_angle_deg         = 0.0f;
static float bmi088_real_pitch_angle_deg  = 0.0f;


/* 用于“启动阶段 Alpha 强一点”的计时 已弃用 */
static float bmi088_filter_elapsed_ms = 0.0f;

/**
 * @brief BMI088 零偏结构体
 * 
 */
typedef struct
{
    //gyro零偏 单位dps (°/s)(度每秒)
    float gyro_bias_x;
    float gyro_bias_y;
    float gyro_bias_z;

    //acc零偏 单位是g （——只去小偏置，不去重力）
    float acc_bias_x;
    float acc_bias_y;
    float acc_bias_z;

    //零偏校准标志
    uint8_t  biascalibration_finish_flag;

    //目标采样数
    uint32_t biascalibration_target_samples;

    //当前采样数
    uint32_t biascalibration_current_samples;

    //实际有效采样数
    uint32_t biascalibration_current_samples_effective;

    //gyro求和
    float gyro_sum_dps_x;
    float gyro_sum_dps_y;
    float gyro_sum_dps_z;

    //acc求和
    float acc_sum_g_x;
    float acc_sum_g_y;
    float acc_sum_g_z;

} bmi088_biascalibration_t;

static bmi088_biascalibration_t bmi088_biascalibration = {0};


/* ===================== 工具函数：raw -> 单位 ===================== */

/**
 * @brief BMI088数据转换函数（gyro：转换为dps acc：转换为g-重力加速度）
 * 
 * @param gyro_raw_x 
 * @param gyro_raw_y 
 * @param gyro_raw_z 
 * @param acc_raw_x 
 * @param acc_raw_y 
 * @param acc_raw_z 
 * @param gyro_dps_x 
 * @param gyro_dps_y 
 * @param gyro_dps_z 
 * @param acc_g_x 
 * @param acc_g_y 
 * @param acc_g_z 
 */
static void bmi088_datatransformation(const int16_t gyro_raw_x,
                                   const int16_t gyro_raw_y,
                                   const int16_t gyro_raw_z,
                                   const int16_t acc_raw_x,
                                   const int16_t acc_raw_y,
                                   const int16_t acc_raw_z,
                                   float *gyro_dps_x,
                                   float *gyro_dps_y,
                                   float *gyro_dps_z,
                                   float *acc_g_x,
                                   float *acc_g_y,
                                   float *acc_g_z)
{
    *gyro_dps_x = ((float)gyro_raw_x) / BMI088_GYRO_LSB_PER_DPS_2000;
    *gyro_dps_y = ((float)gyro_raw_y) / BMI088_GYRO_LSB_PER_DPS_2000;
    *gyro_dps_z = ((float)gyro_raw_z) / BMI088_GYRO_LSB_PER_DPS_2000;

    *acc_g_x = ((float)acc_raw_x) / BMI088_ACC_LSB_PER_G_6G;
    *acc_g_y = ((float)acc_raw_y) / BMI088_ACC_LSB_PER_G_6G;
    *acc_g_z = ((float)acc_raw_z) / BMI088_ACC_LSB_PER_G_6G;
}

/* =============================================================================
 *  零偏校准 BiasCalibration ：开始
 * ============================================================================= */

/**
 * @brief bmi088开始零偏校准函数
 * 
 * @param biascalibration_target_samples 期望参与校准的目标采样数
 */
void bmi088_biascalibration_start(uint32_t biascalibration_target_samples)
{
    //清零标志位
    bmi088_biascalibration.biascalibration_finish_flag = 0;

    //设置采样数
    bmi088_biascalibration.biascalibration_target_samples  = biascalibration_target_samples;
    bmi088_biascalibration.biascalibration_current_samples = 0;

    //恢复状态
    bmi088_biascalibration.gyro_sum_dps_x = 0.0f;
    bmi088_biascalibration.gyro_sum_dps_y = 0.0f;
    bmi088_biascalibration.gyro_sum_dps_z = 0.0f;

    bmi088_biascalibration.acc_sum_g_x = 0.0f;
    bmi088_biascalibration.acc_sum_g_y = 0.0f;
    bmi088_biascalibration.acc_sum_g_z = 0.0f;

    bmi088_biascalibration.gyro_bias_x = 0.0f;
    bmi088_biascalibration.gyro_bias_y = 0.0f;
    bmi088_biascalibration.gyro_bias_z = 0.0f;

    bmi088_biascalibration.acc_bias_x = 0.0f;
    bmi088_biascalibration.acc_bias_y = 0.0f;
    bmi088_biascalibration.acc_bias_z = 0.0f;

    //重新计时：让 Alpha 前 1.5s 强一些
    bmi088_filter_elapsed_ms = 0.0f;
}

float gyro_dps_norm;
float acc_g_norm;
/**
 * @brief BMI088 零偏校准 传入样本数据进行校准数据计算
 * 
 * @param gyro_raw_x 
 * @param gyro_raw_y 
 * @param gyro_raw_z 
 * @param acc_raw_x 
 * @param acc_raw_y 
 * @param acc_raw_z 
 * @return uint8_t 零偏校准是否完成 完成返回1 未完成返回0
 */
uint8_t bmi088_biascalibration_pushsampletocalculate(int16_t gyro_raw_x,int16_t gyro_raw_y,int16_t gyro_raw_z,int16_t acc_raw_x,int16_t acc_raw_y,int16_t acc_raw_z)                                       
{
    //完成检查
    if (bmi088_biascalibration.biascalibration_finish_flag) return 1;

    //目标是否为0检查
    if (bmi088_biascalibration.biascalibration_target_samples == 0) return 0;

    //局部数据
    float gyro_dps_x, gyro_dps_y, gyro_dps_z;
    float acc_g_x, acc_g_y, acc_g_z;

    //单位转换
    bmi088_datatransformation(gyro_raw_x, gyro_raw_y, gyro_raw_z,acc_raw_x,  acc_raw_y,  acc_raw_z,&gyro_dps_x, &gyro_dps_y, &gyro_dps_z,&acc_g_x, &acc_g_y, &acc_g_z);


    gyro_dps_norm = sqrtf(gyro_dps_x*gyro_dps_x + gyro_dps_y*gyro_dps_y + gyro_dps_z*gyro_dps_z);
    if(gyro_dps_norm <= 3.0f)
    {
        acc_g_norm = sqrtf(acc_g_x*acc_g_x + acc_g_y*acc_g_y + acc_g_z*acc_g_z);
        if (acc_g_norm > 0.90f && acc_g_norm < 1.10f)  // 单位：g
        {
            //获取足够目标采样数的数据 进行累加求和
            bmi088_biascalibration.gyro_sum_dps_x += gyro_dps_x;
            bmi088_biascalibration.gyro_sum_dps_y += gyro_dps_y;
            bmi088_biascalibration.gyro_sum_dps_z += gyro_dps_z;

            bmi088_biascalibration.acc_sum_g_x += acc_g_x;
            bmi088_biascalibration.acc_sum_g_y += acc_g_y;
            bmi088_biascalibration.acc_sum_g_z += acc_g_z;
            bmi088_biascalibration.biascalibration_current_samples_effective ++;
        }
    }

    bmi088_biascalibration.biascalibration_current_samples++;

    //如果不够目标采样数据 返回 也有防止超时的作用
    if (bmi088_biascalibration.biascalibration_current_samples < bmi088_biascalibration.biascalibration_target_samples)
    {
        return 0;
    }
    //到达这里说明理论足够采样数据 进行数据平均值计算

    ////先计算得到采样数的倒数 减少后续使用除法

    float current_samples_inv = 0;

    if(bmi088_biascalibration.biascalibration_current_samples_effective != 0)
    {
        current_samples_inv = 1.0f / (float)bmi088_biascalibration.biascalibration_current_samples_effective;
    }
    else
    {
        bmi088_biascalibration.biascalibration_finish_flag = 2;
        return 2;
    }
    

    float gyro_avg_x = bmi088_biascalibration.gyro_sum_dps_x * current_samples_inv;
    float gyro_avg_y = bmi088_biascalibration.gyro_sum_dps_y * current_samples_inv;
    float gyro_avg_z = bmi088_biascalibration.gyro_sum_dps_z * current_samples_inv;

    float acc_avg_x  = bmi088_biascalibration.acc_sum_g_x * current_samples_inv;
    float acc_avg_y  = bmi088_biascalibration.acc_sum_g_y * current_samples_inv;
    float acc_avg_z  = bmi088_biascalibration.acc_sum_g_z * current_samples_inv;

    //gyro零偏赋值（以后每次使用陀螺仪数据都减这个）
    bmi088_biascalibration.gyro_bias_x = gyro_avg_x;
    bmi088_biascalibration.gyro_bias_y = gyro_avg_y;
    bmi088_biascalibration.gyro_bias_z = gyro_avg_z;

    /* acc 零偏：只去掉“静止时那一点点偏置”，不把重力去掉 */

    //计算acc向量模长
    float acc_norm = sqrtf(acc_avg_x * acc_avg_x + acc_avg_y * acc_avg_y + acc_avg_z * acc_avg_z);

    if (acc_norm > 1e-6f)//1e-6f是10的-6次方
    {
        //把平均向量归一化成 1g，认为它就是重力方向
        //也就是把xyz轴的平均向量化为单位向量
        float gravity_x = acc_avg_x / acc_norm;
        float gravity_y = acc_avg_y / acc_norm;
        float gravity_z = acc_avg_z / acc_norm;

        bmi088_biascalibration.acc_bias_x = acc_avg_x - gravity_x;
        bmi088_biascalibration.acc_bias_y = acc_avg_y - gravity_y;
        bmi088_biascalibration.acc_bias_z = acc_avg_z - gravity_z;
    }

    bmi088_biascalibration.biascalibration_finish_flag = 1;
    return 1;
}

uint32_t bmi088_getbiascalibration_target_samples(void)          
{ 
    return bmi088_biascalibration.biascalibration_target_samples; 
}

uint32_t bmi088_getbiascalibration_current_samples(void)          
{ 
    return bmi088_biascalibration.biascalibration_current_samples; 
}

uint32_t bmi088_getbiascalibration_current_samples_effective(void)          
{ 
    return bmi088_biascalibration.biascalibration_current_samples_effective; 
}

float acc_norm_mahony;

/**
 * @brief BMI088 Mahony算法
 * 
 * @param gyro_raw_x 
 * @param gyro_raw_y 
 * @param gyro_raw_z 
 * @param acc_raw_x 
 * @param acc_raw_y 
 * @param acc_raw_z 
 */
void bmi088_mahony(int16_t gyro_raw_x,int16_t gyro_raw_y,int16_t gyro_raw_z,int16_t acc_raw_x,int16_t acc_raw_y,int16_t acc_raw_z)
{
    float gyro_dps_x, gyro_dps_y, gyro_dps_z;
    float gyro_rad_x, gyro_rad_y, gyro_rad_z;
    float acc_g_x, acc_g_y, acc_g_z;
    bmi088_datatransformation(gyro_raw_x, gyro_raw_y, gyro_raw_z,acc_raw_x,  acc_raw_y,  acc_raw_z,&gyro_dps_x, &gyro_dps_y, &gyro_dps_z,&acc_g_x, &acc_g_y, &acc_g_z);

    //零偏校准
    gyro_dps_x -= bmi088_biascalibration.gyro_bias_x;
    gyro_dps_y -= bmi088_biascalibration.gyro_bias_y;
    gyro_dps_z -= bmi088_biascalibration.gyro_bias_z;

    acc_g_x -= bmi088_biascalibration.acc_bias_x;
    acc_g_y -= bmi088_biascalibration.acc_bias_y;
    acc_g_z -= bmi088_biascalibration.acc_bias_z;

    //gyro dps转rad
    gyro_rad_x = gyro_dps_x * DEG2RAD;
    gyro_rad_y = gyro_dps_y * DEG2RAD;
    gyro_rad_z = gyro_dps_z * DEG2RAD;


      //判断 acc 是否可信（幅值门限）
    acc_norm_mahony = sqrtf(acc_g_x * acc_g_x +acc_g_y * acc_g_y +acc_g_z * acc_g_z);

    if (acc_norm_mahony > BMI088_ACC_NORM_MIN_G && acc_norm_mahony < BMI088_ACC_NORM_MAX_G)
    {
        MahonyAHRSupdateIMU(gyro_rad_x,gyro_rad_y,gyro_rad_z,acc_g_x,acc_g_y,acc_g_z);
    }
    else 
    {
        MahonyAHRSupdateIMU(gyro_rad_x,gyro_rad_y,gyro_rad_z,0,0,0);
    }
}

/**
 * @brief 限幅
 * 
 * @param v 
 * @param lo 
 * @param hi 
 * @return float 
 */
static inline float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

/**
 * @brief BMI088 Mahony算法 输出外旋ZXY欧拉角
 * 
 * @param gyro_raw_x 
 * @param gyro_raw_y 
 * @param gyro_raw_z 
 * @param acc_raw_x 
 * @param acc_raw_y 
 * @param acc_raw_z 
 */
void bmi088_mahony_zxy(int16_t gyro_raw_x,int16_t gyro_raw_y,int16_t gyro_raw_z,int16_t acc_raw_x,int16_t acc_raw_y,int16_t acc_raw_z)
{
    bmi088_mahony(gyro_raw_x,gyro_raw_y,gyro_raw_z,acc_raw_x,acc_raw_y,acc_raw_z);
    float w = q0;
    float x = q1;
    float y = q2;
    float z = q3;

    float sx = 2.0f * (w*x - y*z);
    if (sx >  1.0f) sx =  1.0f;
    if (sx < -1.0f) sx = -1.0f;
    bmi088_roll_angle_deg = asinf(sx) * RAD2DEG;

    /* -------- Z (第一个旋转，绕世界 Z) --------
    *  z = atan2( 2*(x*y + w*z), 1 - 2*(x*x + z*z) )
    */
    bmi088_yaw_angle_deg = atan2f(2.0f*(x*y + w*z),1.0f - 2.0f*(x*x + z*z)) * RAD2DEG;

    /* -------- Y (第三个旋转，绕世界 Y) --------
    *  y = atan2( 2*(x*z + w*y), 1 - 2*(x*x + y*y) )
    */
    bmi088_pitch_angle_deg = atan2f(2.0f*(x*z + w*y),1.0f - 2.0f*(x*x + y*y)) * RAD2DEG;
}

/**
 * @brief BMI088 Mahony算法 输出外旋ZYX欧拉角
 * 
 * @param gyro_raw_x 
 * @param gyro_raw_y 
 * @param gyro_raw_z 
 * @param acc_raw_x 
 * @param acc_raw_y 
 * @param acc_raw_z 
 */
void bmi088_mahony_zyx(int16_t gyro_raw_x,int16_t gyro_raw_y,int16_t gyro_raw_z,int16_t acc_raw_x,int16_t acc_raw_y,int16_t acc_raw_z)
{
    bmi088_mahony(gyro_raw_x,gyro_raw_y,gyro_raw_z,acc_raw_x,acc_raw_y,acc_raw_z);
    float w = q0;
    float x = q1;
    float y = q2;
    float z = q3;
   // 0) normalize（建议保留）
    float n = sqrtf(w*w + x*x + y*y + z*z);
    if (n > 0.0f) { w/=n; x/=n; y/=n; z/=n; }

    // pitch (about world-Y)  = asin( 2*(w*y + x*z) )
    float s = 2.0f*(w*y + x*z);
    s = clampf(s, -1.0f, 1.0f);
    float pitch = asinf(s);

    // roll (about world-X) = atan2( -2*(y*z - w*x), 1 - 2*(x^2 + y^2) )
    // yaw  (about world-Z) = atan2( -2*(x*y - w*z), 1 - 2*(y^2 + z^2) )
    float yaw  = atan2f(-2.0f*(x*y - w*z), 1.0f - 2.0f*(y*y + z*z));
    float roll = atan2f(-2.0f*(y*z - w*x), 1.0f - 2.0f*(x*x + y*y));

    // 万向节锁附近（|pitch|≈90°）时 yaw/roll 会耦合、变得不唯一
    // 工程常用处理：保留 pitch，yaw 用 atan2(r10,r11) 或保持上一帧
    const float eps = 1e-6f;
    float cp = cosf(pitch);
    if (fabsf(cp) < eps)
    {
        // 固定 roll = 0，yaw 用 r10/r11 给组合角
        roll = 0.0f;
        yaw  = atan2f(2.0f*(x*y + w*z), 1.0f - 2.0f*(x*x + z*z));
    }

    // 3) 转回 deg
    bmi088_yaw_angle_deg = yaw * RAD2DEG;
    bmi088_pitch_angle_deg = pitch * RAD2DEG;
    bmi088_roll_angle_deg = roll * RAD2DEG;
}

/**
 * 外旋 ZYX（世界轴：Z->Y->X） -> 内旋 ZYX（机体轴：yaw->pitch->roll）
 *
 * 输入：ex_z_deg = α (绕世界Z)
 *      ex_y_deg = β (绕世界Y)
 *      ex_x_deg = γ (绕世界X)
 *
 * 输出：*in_yaw_deg   = ψ
 *      *in_pitch_deg = θ
 *      *in_roll_deg  = φ
 */
void euler_extrinsic_ZYX_to_intrinsic_ZYX_deg(float ex_z_deg, float ex_y_deg, float ex_x_deg,
                                              float *in_yaw_deg, float *in_pitch_deg, float *in_roll_deg)
{
    // 1) 外旋角 -> rad
    float a = ex_z_deg * DEG2RAD; // alpha
    float b = ex_y_deg * DEG2RAD; // beta
    float g = ex_x_deg * DEG2RAD; // gamma

    float ca = cosf(a), sa = sinf(a);
    float cb = cosf(b), sb = sinf(b);
    float cg = cosf(g), sg = sinf(g);

    // 2) 构造 R = Rx(g)*Ry(b)*Rz(a)
    float r00 =  cb*ca;
    //float r01 = -cb*sa;
    //float r02 =  sb;

    float r10 =  cg*sa + sg*sb*ca;
    //float r11 =  cg*ca - sg*sb*sa;
    //float r12 = -sg*cb;

    float r20 =  sg*sa - cg*sb*ca;
    float r21 =  sg*ca + cg*sb*sa;
    float r22 =  cg*cb;

    // 3) 从 R 提取内旋 ZYX：yaw/pitch/roll
    float yaw   = atan2f(r10, r00);
    float pitch = atan2f(-r20, sqrtf(r21*r21 + r22*r22));
    float roll  = atan2f(r21, r22);

    // 4) 转回 deg
    if (in_yaw_deg)   *in_yaw_deg   = yaw   * RAD2DEG;
    if (in_pitch_deg) *in_pitch_deg = pitch * RAD2DEG;
    if (in_roll_deg)  *in_roll_deg  = roll  * RAD2DEG;
}

// 外旋 ZXY  ->  内旋 ZXY   (deg)
// 外旋：先绕 world-Z 旋 ex_z，再绕 world-X 旋 ex_x，再绕 world-Y 旋 ex_y
// 矩阵：R = Ry(ex_y) * Rx(ex_x) * Rz(ex_z)
// 内旋：R = Rz(in_z) * Rx(in_x) * Ry(in_y)
void euler_extrinsic_ZXY_to_intrinsic_ZXY_deg(float ex_z_deg, float ex_x_deg, float ex_y_deg,
                                              float *in_z_deg, float *in_x_deg, float *in_y_deg)
{
    // 1) 外旋角 -> rad
    float a = ex_z_deg * DEG2RAD; // about Z
    float b = ex_x_deg * DEG2RAD; // about X
    float c = ex_y_deg * DEG2RAD; // about Y

    float ca = cosf(a), sa = sinf(a);
    float cb = cosf(b), sb = sinf(b);
    float cc = cosf(c), sc = sinf(c);

    // 2) 构造 R = Ry(c)*Rx(b)*Rz(a)
    // 只算后面提取内旋ZXY所需的元素（以及万向节锁时备用的 r00/r10）
    float r00 =  cc*ca + sc*sb*sa;
    float r01 = -cc*sa + sc*sb*ca;

    float r10 =  cb*sa;
    float r11 =  cb*ca;

    float r20 = -sc*ca + cc*sb*sa;
    float r21 =  sc*sa + cc*sb*ca;
    float r22 =  cc*cb;

    // 3) 从 R 提取内旋 ZXY：in_z / in_x / in_y
    // 对于内旋 ZXY: R = Rz(z)*Rx(x)*Ry(y)
    // 可得：sin(x)=r21, z=atan2(-r01,r11), y=atan2(-r20,r22)
    float sx = r21;
    if (sx >  1.0f) sx =  1.0f;
    if (sx < -1.0f) sx = -1.0f;

    float in_x = asinf(sx);
    float cx   = cosf(in_x);

    float in_z, in_y;
    const float eps = 1e-6f;

    if (fabsf(cx) > eps)
    {
        in_z = atan2f(-r01, r11);
        in_y = atan2f(-r20, r22);
    }
    else
    {
        // 万向节锁：|in_x| ≈ 90°，z 和 y 不再唯一，只能确定组合量
        // 这里选一个常用策略：令 in_z = 0，然后用 r10/r00 解出 in_y
        in_z = 0.0f;
        float phi = atan2f(r10, r00);   // 当 in_x=+90°: phi = in_z + in_y；当 in_x=-90°: phi = in_z - in_y
        if (sx > 0.0f)  in_y = phi;     // +90°
        else            in_y = -phi;    // -90°
    }

    // 4) 转回 deg
    if (in_z_deg) *in_z_deg = in_z * RAD2DEG;
    if (in_x_deg) *in_x_deg = in_x * RAD2DEG;
    if (in_y_deg) *in_y_deg = in_y * RAD2DEG;
}

/**
 * @brief ZYX欧拉角输出前向轴Yaw and Pitch
 * 
 * @param ex_z_deg 
 * @param ex_y_deg 
 * @param ex_x_deg 
 * @param yaw_deg 
 * @param pitch_deg 
 */
void euler_extrinsic_ZYX_to_front_yaw_pitch_deg(float ex_z_deg, float ex_y_deg, float ex_x_deg,float *yaw_deg, float *pitch_deg)
{
    // deg -> rad
    float z = ex_z_deg * DEG2RAD;
    float y = ex_y_deg * DEG2RAD;
    float x = ex_x_deg * DEG2RAD;

    float sx = sinf(x), cx = cosf(x);
    float sy = sinf(y), cy = cosf(y);
    float sz = sinf(z), cz = cosf(z);

    // R = Rx(x)*Ry(y)*Rz(z) 的第一列 f = (fx,fy,fz)
    float fx = cy * cz;
    float fy = cx * sz + sx * sy * cz;
    float fz = sx * sz - cx * sy * cz;

    float yaw   = atan2f(fy, fx);
    float pitch = atan2f(-fz, sqrtf(fx*fx + fy*fy));

    if (yaw_deg)   *yaw_deg   = yaw   * RAD2DEG;
    if (pitch_deg) *pitch_deg = pitch * RAD2DEG;
}

/**
 * @brief ZXY欧拉角输出前向轴Yaw and Pitch
 * 
 * @param ex_z_deg 
 * @param ex_x_deg 
 * @param ex_y_deg 
 * @param yaw_deg 
 * @param pitch_deg 
 */
void euler_extrinsic_ZXY_to_front_yaw_pitch_deg(float ex_z_deg, float ex_x_deg, float ex_y_deg,float *yaw_deg, float *pitch_deg)
{
    float z = ex_z_deg * DEG2RAD;
    float x = ex_x_deg * DEG2RAD;
    float y = ex_y_deg * DEG2RAD;

    float sx = sinf(x), cx = cosf(x);
    float sy = sinf(y), cy = cosf(y);
    float sz = sinf(z), cz = cosf(z);

    // R = Ry(y)*Rx(x)*Rz(z) 的第一列
    float fx = cy*cz + sy*sx*sz;
    float fy = cx*sz;
    float fz = -sy*cz + cy*sx*sz;

    float yaw   = atan2f(fy, fx);
    float pitch = atan2f(-fz, sqrtf(fx*fx + fy*fy));

    if (yaw_deg)   *yaw_deg   = yaw   * RAD2DEG;
    if (pitch_deg) *pitch_deg = pitch * RAD2DEG;
}

/* ===================== 数据接口 ===================== */
float BMI088_GetRollDeg(void)          
{ 
    return bmi088_roll_angle_deg; 
}

float BMI088_GetPitchDeg(void)         
{ 
    return bmi088_pitch_angle_deg; 
}

float BMI088_GetYawDeg(void)           
{ 
    return bmi088_yaw_angle_deg; 
}

float BMI088_GetRealPitchDeg(void)     
{ 
    return bmi088_real_pitch_angle_deg; 
}

uint8_t bmi088_get_biascalibration_finish_flag(void)  
{ 
    return bmi088_biascalibration.biascalibration_finish_flag; 
}



//---------------------------------已弃用------------------------------
//---------------------------------已弃用------------------------------
//---------------------------------已弃用------------------------------
//---------------------------------已弃用------------------------------
//---------------------------------已弃用------------------------------
//---------------------------------已弃用------------------------------
//---------------------------------已弃用------------------------------
//---------------------------------已弃用------------------------------
//---------------------------------已弃用------------------------------
//---------------------------------已弃用------------------------------
//---------------------------------已弃用------------------------------
//---------------------------------已弃用------------------------------
//---------------------------------已弃用------------------------------
//---------------------------------已弃用------------------------------
//---------------------------------已弃用------------------------------
//---------------------------------已弃用------------------------------
//---------------------------------已弃用------------------------------
//---------------------------------已弃用------------------------------
//---------------------------------已弃用------------------------------
//---------------------------------已弃用------------------------------
//---------------------------------已弃用------------------------------
//---------------------------------已弃用------------------------------
//---------------------------------已弃用------------------------------
//---------------------------------已弃用------------------------------
//---------------------------------已弃用------------------------------
//---------------------------------已弃用------------------------------
//---------------------------------已弃用------------------------------






/* ===================== 互补滤波参数 ===================== */
/* 6050 里：Timer_ms < 1500 -> Alpha=0.70，否则 Alpha=0.25
   这里用 dt_seconds 累积时间实现同样效果 */
#define BMI088_ALPHA_STRONG            (0.70f)
#define BMI088_ALPHA_WEAK              (0.15f)
#define BMI088_ALPHA_SWITCH_TIME_MS    (1500.0f)

/* RealPitch 的 Alpha（你 6050 给的是 0.4） */
#define BMI088_REAL_PITCH_ALPHA        (0.40f)

/* 欧拉角运动学防 gimbal lock：cos(theta) 下限 */
#define BMI088_COS_THETA_EPS           (0.01f)     /* 约等于 pitch 接近 89.4° 时开始夹住 */

/* =============================================================================
 * 姿态互补滤波更新 ComplementaryFilter-互补滤波
 *
 * 
 * - gyro 积分：roll+=gx*dt, pitch+=gy*dt, yaw+=gz*dt
 * - ZYX 欧拉角运动学方程，把机体系角速度(p,q,r) -> 欧拉角变化率(phi_dot,theta_dot,psi_dot)
 * ============================================================================= */
void bmi088_complementaryfilter_2(int16_t gyro_raw_x,
                                                int16_t gyro_raw_y,
                                                int16_t gyro_raw_z,
                                                int16_t acc_raw_x,
                                                int16_t acc_raw_y,
                                                int16_t acc_raw_z,
                                                float dt_seconds)
{
    float gyro_dps_x, gyro_dps_y, gyro_dps_z;
    float acc_g_x, acc_g_y, acc_g_z;

    //数据转换
    bmi088_datatransformation(gyro_raw_x, gyro_raw_y, gyro_raw_z,acc_raw_x,  acc_raw_y,  acc_raw_z,&gyro_dps_x, &gyro_dps_y, &gyro_dps_z,&acc_g_x, &acc_g_y, &acc_g_z);

    //零偏校准
    gyro_dps_x -= bmi088_biascalibration.gyro_bias_x;
    gyro_dps_y -= bmi088_biascalibration.gyro_bias_y;
    gyro_dps_z -= bmi088_biascalibration.gyro_bias_z;

    acc_g_x -= bmi088_biascalibration.acc_bias_x;
    acc_g_y -= bmi088_biascalibration.acc_bias_y;
    acc_g_z -= bmi088_biascalibration.acc_bias_z;


    //Alpha 的“启动强/后期弱”

    /* 计时：决定 Alpha 强弱 */
    bmi088_filter_elapsed_ms += dt_seconds * 1000.0f;

    float Alpha = BMI088_ALPHA_WEAK;
    if (bmi088_filter_elapsed_ms < BMI088_ALPHA_SWITCH_TIME_MS)
    {
        Alpha = BMI088_ALPHA_STRONG;
    }

    /* -------------------------------------------------------------------------
     * 1) 用 ZYX 欧拉角运动学方程进行 gyro 预测（单位内部用 rad）
     *    角定义：roll=phi(绕x), pitch=theta(绕y), yaw=psi(绕z)
     *    BMI088 gyro：p=wx, q=wy, r=wz（机体系角速度）
     * ------------------------------------------------------------------------- */
    float phi   = bmi088_roll_angle_deg  * DEG2RAD;   /* roll */
    float theta = bmi088_pitch_angle_deg * DEG2RAD;   /* pitch */
    float psi   = bmi088_yaw_angle_deg   * DEG2RAD;   /* yaw */

    float p = gyro_dps_x * DEG2RAD;  /* rad/s */
    float q = gyro_dps_y * DEG2RAD;
    float r = gyro_dps_z * DEG2RAD;

    float sphi = sinf(phi);
    float cphi = cosf(phi);

    float ctheta = cosf(theta);
    if (fabsf(ctheta) < BMI088_COS_THETA_EPS)
    {
        ctheta = (ctheta >= 0.0f) ? BMI088_COS_THETA_EPS : -BMI088_COS_THETA_EPS;
    }
    float ttheta = sinf(theta) / ctheta;

    /* ZYX Euler kinematics:
       phi_dot   = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta)
       theta_dot = q*cos(phi) - r*sin(phi)
       psi_dot   = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta)
    */
    float phi_dot   = p + q * sphi * ttheta + r * cphi * ttheta;
    float theta_dot = q * cphi - r * sphi;
    float psi_dot   = (q * sphi + r * cphi) / ctheta;

    float phi_g   = phi   + phi_dot   * dt_seconds;
    float theta_g = theta + theta_dot * dt_seconds;
    float psi_g   = psi   + psi_dot   * dt_seconds;

    float gyroscope_integrated_roll_deg  = phi_g   * RAD2DEG;
    float gyroscope_integrated_pitch_deg = theta_g * RAD2DEG;
    float gyroscope_integrated_yaw_deg   = psi_g   * RAD2DEG;

    /* yaw 更新（和你原来一样：先用 gyro 预测，然后 wrap） */
    bmi088_yaw_angle_deg = gyroscope_integrated_yaw_deg;
    if (bmi088_yaw_angle_deg > 180.0f)  bmi088_yaw_angle_deg -= 360.0f;
    if (bmi088_yaw_angle_deg < -180.0f) bmi088_yaw_angle_deg += 360.0f;

    /* 2) 判断 acc 是否可信（幅值门限） */
    float accelerometer_norm = sqrtf(acc_g_x * acc_g_x +
                                     acc_g_y * acc_g_y +
                                     acc_g_z * acc_g_z);

    uint8_t accelerometer_is_valid_for_correction = 0;
    if (accelerometer_norm > BMI088_ACC_NORM_MIN_G && accelerometer_norm < BMI088_ACC_NORM_MAX_G)
    {
        accelerometer_is_valid_for_correction = 1;
    }

    /* 3) 按你 MPU6050 的思路：用 yaw 把 ax/ay 旋转到对齐坐标（AX2/AY2） */
    float yaw_rad = bmi088_yaw_angle_deg * DEG2RAD;
    float cy = cosf(yaw_rad);
    float sy = sinf(yaw_rad);

    float accelerometer_g_x2 = acc_g_x * cy - acc_g_y * sy;
    float accelerometer_g_y2 = acc_g_x * sy + acc_g_y * cy;

    /* 4) 用 acc 算 a_roll / a_pitch（仍然输出度） */
    /* Roll：atan2(AY2, AZ) */
    float accelerometer_roll_angle_deg =
        atan2f(accelerometer_g_y2, acc_g_z) * RAD2DEG;

    /* Pitch：atan2(-AX2, sqrt(AY2^2 + AZ^2)) */
    float accelerometer_pitch_angle_deg =
        atan2f(-accelerometer_g_x2,
               sqrtf(accelerometer_g_y2 * accelerometer_g_y2 + acc_g_z * acc_g_z)) * RAD2DEG;

    /* 5) 互补滤波（模仿你 6050：Angle = Alpha*a + (1-Alpha)*g） */
    if (accelerometer_is_valid_for_correction)
    {
        bmi088_roll_angle_deg  = Alpha * accelerometer_roll_angle_deg  + (1.0f - Alpha) * gyroscope_integrated_roll_deg;
        bmi088_pitch_angle_deg = Alpha * accelerometer_pitch_angle_deg + (1.0f - Alpha) * gyroscope_integrated_pitch_deg;
    }
    else
    {
        /* acc 不可信时只用 gyro */
        bmi088_roll_angle_deg  = gyroscope_integrated_roll_deg;
        bmi088_pitch_angle_deg = gyroscope_integrated_pitch_deg;
    }

    /* 6) RealPitch（保持你原来的定义：-atan2(AX, AZ) + gyro_y 积分 + 互补） */
    float accelerometer_real_pitch_angle_deg =
        -atan2f(acc_g_x, acc_g_z) * RAD2DEG;

    float gyroscope_integrated_real_pitch_deg =
        bmi088_real_pitch_angle_deg + gyro_dps_y * dt_seconds;

    if (accelerometer_is_valid_for_correction)
    {
        float Alpha_Real = BMI088_REAL_PITCH_ALPHA;
        bmi088_real_pitch_angle_deg =
            Alpha_Real * accelerometer_real_pitch_angle_deg +
            (1.0f - Alpha_Real) * gyroscope_integrated_real_pitch_deg;
    }
    else
    {
        bmi088_real_pitch_angle_deg = gyroscope_integrated_real_pitch_deg;
    }
}

void bmi088_complementaryfilter_1(int16_t gyro_raw_x,int16_t gyro_raw_y,int16_t gyro_raw_z,int16_t acc_raw_x,int16_t acc_raw_y,int16_t acc_raw_z,float dt_seconds)
{
    float gyro_dps_x, gyro_dps_y, gyro_dps_z;
    float acc_g_x, acc_g_y, acc_g_z;

    // 数据转换
    bmi088_datatransformation(gyro_raw_x, gyro_raw_y, gyro_raw_z,acc_raw_x,  acc_raw_y,  acc_raw_z,&gyro_dps_x, &gyro_dps_y, &gyro_dps_z,&acc_g_x, &acc_g_y, &acc_g_z);

    // 零偏校准（扣偏）
    gyro_dps_x -= bmi088_biascalibration.gyro_bias_x;
    gyro_dps_y -= bmi088_biascalibration.gyro_bias_y;
    gyro_dps_z -= bmi088_biascalibration.gyro_bias_z;

    acc_g_x -= bmi088_biascalibration.acc_bias_x;
    acc_g_y -= bmi088_biascalibration.acc_bias_y;
    acc_g_z -= bmi088_biascalibration.acc_bias_z;

    // Alpha 的“启动强/后期弱”
    bmi088_filter_elapsed_ms += dt_seconds * 1000.0f;

    float Alpha = BMI088_ALPHA_WEAK;
    if (bmi088_filter_elapsed_ms < BMI088_ALPHA_SWITCH_TIME_MS)
    {
        Alpha = BMI088_ALPHA_STRONG;
    }

    /* -------------------------------------------------------------------------
     * 1) 用 gyro 直接积分得到预测角（单位：度）
     *    （回到你最早的写法，不用欧拉角运动学方程）
     * ------------------------------------------------------------------------- */
    float gyroscope_integrated_roll_deg  = bmi088_roll_angle_deg  + gyro_dps_x * dt_seconds;
    float gyroscope_integrated_pitch_deg = bmi088_pitch_angle_deg + gyro_dps_y * dt_seconds;
    float gyroscope_integrated_yaw_deg   = bmi088_yaw_angle_deg   + gyro_dps_z * dt_seconds;

    // yaw 更新 + wrap
    bmi088_yaw_angle_deg = gyroscope_integrated_yaw_deg;
    if (bmi088_yaw_angle_deg > 180.0f)  bmi088_yaw_angle_deg -= 360.0f;
    if (bmi088_yaw_angle_deg < -180.0f) bmi088_yaw_angle_deg += 360.0f;

    /* 2) 判断 acc 是否可信（幅值门限） */
    float accelerometer_norm = sqrtf(acc_g_x * acc_g_x +
                                     acc_g_y * acc_g_y +
                                     acc_g_z * acc_g_z);

    uint8_t accelerometer_is_valid_for_correction = 0;
    if (accelerometer_norm > BMI088_ACC_NORM_MIN_G && accelerometer_norm < BMI088_ACC_NORM_MAX_G)
    {
        accelerometer_is_valid_for_correction = 1;
    }

    /* 3) 用 yaw 把 ax/ay 旋转到对齐坐标（AX2/AY2） */
    float yaw_rad = bmi088_yaw_angle_deg * DEG2RAD;
    float cy = cosf(yaw_rad);
    float sy = sinf(yaw_rad);

    float accelerometer_g_x2 = acc_g_x * cy - acc_g_y * sy;
    float accelerometer_g_y2 = acc_g_x * sy + acc_g_y * cy;

    /* 4) 用 acc 算 a_roll / a_pitch（输出度） */
    float accelerometer_roll_angle_deg =
        atan2f(accelerometer_g_y2, acc_g_z) * RAD2DEG;

    float accelerometer_pitch_angle_deg =
        atan2f(-accelerometer_g_x2,
               sqrtf(accelerometer_g_y2 * accelerometer_g_y2 + acc_g_z * acc_g_z)) * RAD2DEG;

    /* 5) 互补滤波 */
    if (accelerometer_is_valid_for_correction)
    {
        bmi088_roll_angle_deg  = Alpha * accelerometer_roll_angle_deg  + (1.0f - Alpha) * gyroscope_integrated_roll_deg;
        bmi088_pitch_angle_deg = Alpha * accelerometer_pitch_angle_deg + (1.0f - Alpha) * gyroscope_integrated_pitch_deg;
    }
    else
    {
        bmi088_roll_angle_deg  = gyroscope_integrated_roll_deg;
        bmi088_pitch_angle_deg = gyroscope_integrated_pitch_deg;
    }

    /* 6) RealPitch（保持你原来的定义） */
    float accelerometer_real_pitch_angle_deg =
        -atan2f(acc_g_x, acc_g_z) * RAD2DEG;

    float gyroscope_integrated_real_pitch_deg =
        bmi088_real_pitch_angle_deg + gyro_dps_y * dt_seconds;

    if (accelerometer_is_valid_for_correction)
    {
        float Alpha_Real = BMI088_REAL_PITCH_ALPHA;
        bmi088_real_pitch_angle_deg =
            Alpha_Real * accelerometer_real_pitch_angle_deg +
            (1.0f - Alpha_Real) * gyroscope_integrated_real_pitch_deg;
    }
    else
    {
        bmi088_real_pitch_angle_deg = gyroscope_integrated_real_pitch_deg;
    }
}