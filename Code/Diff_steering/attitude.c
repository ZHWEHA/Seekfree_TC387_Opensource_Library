#include "attitude.h"
#include "zf_common_headfile.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

//滤波参数
#define TWO_KP      1.0f        //比例增益  ->2*Kp
#define TWO_KI      0.02f       //积分增益 -> 2*Ki  非零以抑制漂移
#define INTEGRAL_LIMIT 1.0f     //积分限幅,避免积分饱和

//状态变量
static float gyro_z_offset = 0.0f;
static float yaw_rate = 0.0f;

//四元数
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

//积分误差
static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

//磁力计校准
static float mag_offset[3] = {0,0,0};
static float mag_scale[3] = {1,1,1};

//保留磁场方向
static float bx = 1.0f, bz = 0.0f;

//内部函数
static void MahonyAHRSupdate(float gx, float gy, float gz,
float ax, float ay, float az,
float mx, float my, float mz,
float dt);

static float GetYawFromQuat(void);
static float GetPitchFromQuat(void);
static float GetRollFromQuat(void);

//初始化
void Attitude_Init(void){
    if (imu963ra_init() != 0){
        while(1);
    }

    //陀螺仪零偏
    float sum = 0;
    for (int i = 0; i < 100; i++){
        imu963ra_get_gyro();
        sum += imu963ra_gyro_transition(imu963ra_gyro_z);
        system_delay_us(1000);
    }
    gyro_z_offset = sum / 100.0f;

    //重置
    q0 = 1.0f; q1 = q2 = q3 = 0.0f;
    integralFBx = integralFBy = integralFBz = 0.0f;
}

//状态更新
void Attitude_Update(float dt){
    if (dt <= 0.0f || dt > 0.1f) return; //防止异常dt

    imu963ra_get_acc();
    imu963ra_get_gyro();
    imu963ra_get_mag();

    float ax = imu963ra_acc_transition(imu963ra_acc_x) * ATTITUDE_ACC_SIGN_X;
    float ay = imu963ra_acc_transition(imu963ra_acc_y) * ATTITUDE_ACC_SIGN_Y;
    float az = imu963ra_acc_transition(imu963ra_acc_z) * ATTITUDE_ACC_SIGN_Z;

    float gx = imu963ra_gyro_transition(imu963ra_gyro_x) * ATTITUDE_GYR_SIGN_X * M_PI / 180.0f;
    float gy = imu963ra_gyro_transition(imu963ra_gyro_y) * ATTITUDE_GYR_SIGN_Y * M_PI / 180.0f;
    float gz = (imu963ra_gyro_transition(imu963ra_gyro_z) - gyro_z_offset) * ATTITUDE_GYR_SIGN_Z * M_PI / 180.0f;

    float mx = (imu963ra_mag_transition(imu963ra_mag_x) - mag_offset[0]) * mag_scale[0] * ATTITUDE_MAG_SIGN_X;
    float my = (imu963ra_mag_transition(imu963ra_mag_y) - mag_offset[1]) * mag_scale[1] * ATTITUDE_MAG_SIGN_Y;
    float mz = (imu963ra_mag_transition(imu963ra_mag_z) - mag_offset[2]) * mag_scale[2] * ATTITUDE_MAG_SIGN_Z;

    //磁力计权重
    float acc_norm = sqrtf(ax*ax + ay*ay + az*az);
    float gyro_norm = sqrtf(gx*gx + gy*gy + gz*gz) * 180.0f / M_PI;

    float mag_weight = 1.0f;

    if (fabsf(acc_norm - 9.81f) > 0.5f)
        mag_weight *= 0.3f;

    if (gyro_norm > 20.0f)
        mag_weight *= 0.3f;

    mx *= mag_weight;
    my *= mag_weight;
    mz *= mag_weight;

    MahonyAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz, dt);

    yaw_rate = gz * 180.0f / M_PI;
}

//互补滤波算法
static void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt){
    float norm;
    float vx, vy, vz;
    float wx, wy, wz;
    float ex, ey, ez;

    //归一化加速度
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 1e-6f)
    return;
    ax /= norm;
    ay /= norm;
    az /= norm;

    //归一化磁力计
    if (mx != 0 || my != 0 || mz != 0){
        norm = sqrtf(mx*mx + my*my + mz*mz);
        if (norm > 1e-6f){
            mx /= norm; my /= norm; mz /= norm;
        }

        float hx = 2.0f * (mx*(0.5f - q2*q2 - q3*q3) + my*(q1*q2 - q0*q3) + mz*(q1*q3 + q0*q2));
        float hy = 2.0f * (mx*(q1*q2 + q0*q3) + my*(0.5f - q1*q1 - q3*q3) + mz*(q2*q3 - q0*q1));

        bx = sqrtf(hx*hx + hy*hy);
        bz = 2.0f * (mx*(q1*q3 - q0*q2) + my*(q2*q3 + q0*q1) + mz*(0.5f - q1*q1 - q2*q2)); 
    }

    vx = 2.0f*(q1*q3 - q0*q2);
    vy = 2.0f*(q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    wx = 2.0f*(bx*(0.5f - q2*q2 - q3*q3) + bz*(q1*q3 - q0*q2));
    wy = 2.0f*(bx*(q1*q2 - q0*q3) + bz*(q0*q1 + q2*q3));
    wz = 2.0f*(bx*(q0*q2 + q1*q3) + bz*(0.5f - q1*q1 - q2*q2));

    ex = (ay*vz - az*vy) + (my*wz - mz*wy);
    ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
    ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

    //积分
    integralFBx += TWO_KI * ex * dt;
    integralFBy += TWO_KI * ey * dt;
    integralFBz += TWO_KI * ez * dt;

    //限幅
    if (integralFBx > INTEGRAL_LIMIT) integralFBx = INTEGRAL_LIMIT;
    if (integralFBx < -INTEGRAL_LIMIT) integralFBx = -INTEGRAL_LIMIT;

    if (integralFBy > INTEGRAL_LIMIT) integralFBy = INTEGRAL_LIMIT;
    if (integralFBy < -INTEGRAL_LIMIT) integralFBy = -INTEGRAL_LIMIT;

    if (integralFBz > INTEGRAL_LIMIT) integralFBz = INTEGRAL_LIMIT;
    if (integralFBz < -INTEGRAL_LIMIT) integralFBz = -INTEGRAL_LIMIT;

    gx += integralFBx + TWO_KP * ex;
    gy += integralFBy + TWO_KP * ey;
    gz += integralFBz + TWO_KP * ez;

    //四元数更新
    float dq0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    float dq1 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    float dq2 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    float dq3 = 0.5f * ( q0*gz + q1*gy - q2*gx);

    q0 += dq0 * dt;
    q1 += dq1 * dt;
    q2 += dq2 * dt;
    q3 += dq3 * dt;

    //归一化
    norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm < 1e-6f)
    return;

    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
}

//欧拉角
static float GetYawFromQuat(void){
    return atan2f(2.0f*(q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3) * 180.0f / M_PI;
}

static float GetPitchFromQuat(void){
    return asinf(2.0f*(q0*q2 - q1*q3)) * 180.0f / M_PI;
}

static float GetRollFromQuat(void){
    return atan2f(2.0f*(q0*q1 + q2*q3), q0*q0 - q1*q1 - q2*q2 + q3*q3) * 180.0f / M_PI;
}

//外部接口

//获取偏航角（Yaw）
//含义：绕Z轴旋转角（航向角）
//范围：-180° ~ 180°
//参考：磁北方向
float Attitude_GetYaw(void){
    return GetYawFromQuat();
}

//获取俯仰角（Pitch）
//含义：绕Y轴旋转（前后抬头/低头）
//范围：约 -90° ~ 90°
float Attitude_GetPitch(void){
    return GetPitchFromQuat();
}

//获取横滚角（Roll）
//含义：绕X轴旋转（左右倾斜）
//范围：-180° ~ 180°
float Attitude_GetRoll(void){ 
    return GetRollFromQuat();
}

//获取偏航角速度（Yaw Rate）
//含义：绕Z轴的角速度
//单位：度/秒
//来源：陀螺仪Z轴（已去零偏 + 滤波修正）
float Attitude_GetYawRate(void){ 
    return yaw_rate;
}