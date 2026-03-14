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


//陀螺仪零偏
static float gyro_z_offset = 0.0f;
//当前偏航角速度（度/秒）
static float yaw_rate = 0.0f;
//四元数
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
//滤波器积分误差
static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
//磁力计校准参数 -> 硬铁偏移、软铁缩放
static float mag_offset[3] = {0, 0, 0};
static float mag_scale[3]  = {1, 1, 1};

//辅助函数声明
static void MahonyAHRSupdate(float gx, float gy, float gz,float ax, float ay, float az,float mx, float my, float mz,float dt);
static float GetYawFromQuat(void);
static float GetPitchFromQuat(void);
static float GetRollFromQuat(void);

//公共函数
void Attitude_Init(void) {
    //初始化IMU
    if (imu963ra_init() != 0) {
        while(1);       //可添加初始化错误提示
    }

    // 陀螺仪零偏校准 -> 静止时采集100次
    const int samples = 100;
    float sum = 0;
    for (int i = 0; i < samples; i++) {
        imu963ra_get_gyro();
        sum += imu963ra_gyro_transition(imu963ra_gyro_z);
        system_delay_us(1000);   //1ms间隔
    }
    gyro_z_offset = sum / samples;

    //重置滤波器状态
    q0 = 1.0f; q1 = q2 = q3 = 0.0f;
    integralFBx = integralFBy = integralFBz = 0.0f;
}

void Attitude_Update(float dt) {
    //读取原始数据
    imu963ra_get_acc();
    imu963ra_get_gyro();
    imu963ra_get_mag();

    //转换为物理值并应用坐标系符号
    float ax = imu963ra_acc_transition(imu963ra_acc_x) * ATTITUDE_ACC_SIGN_X;
    float ay = imu963ra_acc_transition(imu963ra_acc_y) * ATTITUDE_ACC_SIGN_Y;
    float az = imu963ra_acc_transition(imu963ra_acc_z) * ATTITUDE_ACC_SIGN_Z;

    float gx = imu963ra_gyro_transition(imu963ra_gyro_x) * ATTITUDE_GYR_SIGN_X * M_PI / 180.0f;
    float gy = imu963ra_gyro_transition(imu963ra_gyro_y) * ATTITUDE_GYR_SIGN_Y * M_PI / 180.0f;
    float gz = (imu963ra_gyro_transition(imu963ra_gyro_z) - gyro_z_offset) * ATTITUDE_GYR_SIGN_Z * M_PI / 180.0f;

    //磁力计数据应用校准和符号
    float mx = (imu963ra_mag_transition(imu963ra_mag_x) - mag_offset[0]) * mag_scale[0] * ATTITUDE_MAG_SIGN_X;
    float my = (imu963ra_mag_transition(imu963ra_mag_y) - mag_offset[1]) * mag_scale[1] * ATTITUDE_MAG_SIGN_Y;
    float mz = (imu963ra_mag_transition(imu963ra_mag_z) - mag_offset[2]) * mag_scale[2] * ATTITUDE_MAG_SIGN_Z;

    //磁力计信任度判断：当加速度计模长接近1g且角速度较小时，才完全使用磁力计
    float acc_norm = sqrtf(ax*ax + ay*ay + az*az);
    float gyro_norm = sqrtf(gx*gx + gy*gy + gz*gz) * 180.0f / M_PI;         //转为度/秒
    const float ACC_REF = 9.81f;        //设加速度计单位是m/s^2，根据实际调整
    const float ACC_TOL = 0.5f;          //容忍范围
    const float GYRO_TOL = 20.0f;       //角速度容忍范围   度/秒

    if (fabsf(acc_norm - ACC_REF) > ACC_TOL || gyro_norm > GYRO_TOL) {
        //磁场不可信，仅使用加速度计修正 -> 将磁力计数据置零
        mx = my = mz = 0.0f;
    }

    //运行Mahony滤波
    MahonyAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz, dt);

    //保存偏航角速度
    yaw_rate = gz * 180.0f / M_PI;   //正值=左转
}

float Attitude_GetYaw(void) {
    return GetYawFromQuat();
}

float Attitude_GetPitch(void) {
    return GetPitchFromQuat();
}

float Attitude_GetRoll(void) {
    return GetRollFromQuat();
}

float Attitude_GetYawRate(void) {
    return yaw_rate;
}

void Attitude_CalibrateMag(uint16_t samples) {
    //硬铁校准：采集最大最小值，取中点作为偏移
    float minx = 1e6, maxx = -1e6;
    float miny = 1e6, maxy = -1e6;
    float minz = 1e6, maxz = -1e6;

    for (uint16_t i = 0; i < samples; i++) {
        imu963ra_get_mag();
        float mx = imu963ra_mag_transition(imu963ra_mag_x);
        float my = imu963ra_mag_transition(imu963ra_mag_y);
        float mz = imu963ra_mag_transition(imu963ra_mag_z);

        if (mx < minx) minx = mx;
        if (mx > maxx) maxx = mx;
        if (my < miny) miny = my;
        if (my > maxy) maxy = my;
        if (mz < minz) minz = mz;
        if (mz > maxz) maxz = mz;

        system_delay_ms(10);
    }

    mag_offset[0] = (maxx + minx) * 0.5f;
    mag_offset[1] = (maxy + miny) * 0.5f;
    mag_offset[2] = (maxz + minz) * 0.5f;

    //计算缩放因子 -> 使各轴范围归一化到相同长度，处理为各轴范围的平均
    float rangex = maxx - minx;
    float rangey = maxy - miny;
    float rangez = maxz - minz;
    float avg_range = (rangex + rangey + rangez) / 3.0f;
    mag_scale[0] = avg_range / rangex;
    mag_scale[1] = avg_range / rangey;
    mag_scale[2] = avg_range / rangez;
}

//内部静态函数

static void MahonyAHRSupdate(float gx, float gy, float gz,
                             float ax, float ay, float az,
                             float mx, float my, float mz,
                             float dt) {
    float norm;
    float hx, hy, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;

    //归一化加速度计
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return;
    norm = 1.0f / norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    //归一化磁力计（如果数据有效的话zzz）
    if (mx != 0.0f || my != 0.0f || mz != 0.0f) {
        norm = sqrtf(mx * mx + my * my + mz * mz);
        if (norm != 0.0f) {
            norm = 1.0f / norm;
            mx *= norm;
            my *= norm;
            mz *= norm;
        }
    }

    //预计算四元数乘积项
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    //磁力计转换到坐标系 -> 计算期望磁场方向
    if (mx != 0.0f || my != 0.0f || mz != 0.0f) {
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrtf(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
    } else {
        //无磁力计时，使用上一时刻的bx,bz或设为0
        bx = 0.0f;
        bz = 0.0f;
    }

    //重力方向估计
    vx = 2.0f * (q1q3 - q0q2);
    vy = 2.0f * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    //磁场方向估计 -> 从坐标系转回机体系
    if (bx != 0.0f || bz != 0.0f) {
        wx = 2.0f * (bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2));
        wy = 2.0f * (bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3));
        wz = 2.0f * (bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2));
    } else {
        wx = wy = wz = 0.0f;
    }

    //误差 = 测量向量与估计向量的叉积
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

    //积分误差（带限幅）
    if (TWO_KI > 0.0f) {
        integralFBx += TWO_KI * ex * dt;
        integralFBy += TWO_KI * ey * dt;
        integralFBz += TWO_KI * ez * dt;

        //积分限幅
        float limit = INTEGRAL_LIMIT;
        if (integralFBx > limit) integralFBx = limit;
        if (integralFBx < -limit) integralFBx = -limit;
        if (integralFBy > limit) integralFBy = limit;
        if (integralFBy < -limit) integralFBy = -limit;
        if (integralFBz > limit) integralFBz = limit;
        if (integralFBz < -limit) integralFBz = -limit;

        gx += integralFBx;
        gy += integralFBy;
        gz += integralFBz;
    } else {
        integralFBx = integralFBy = integralFBz = 0.0f;
    }

    //比例反馈
    gx += TWO_KP * ex;
    gy += TWO_KP * ey;
    gz += TWO_KP * ez;

    //四元数微分方程 -> 使用一阶欧拉
    float dq0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float dq1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    float dq2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    float dq3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    q0 += dq0 * dt;
    q1 += dq1 * dt;
    q2 += dq2 * dt;
    q3 += dq3 * dt;

    //归一化四元数
    norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm == 0.0f) return;
    norm = 1.0f / norm;
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;
}

static float GetYawFromQuat(void) {
    return atan2f(2.0f * (q1 * q2 + q0 * q3),
                  q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 180.0f / M_PI;
}

static float GetPitchFromQuat(void) {
    return asinf(2.0f * (q0 * q2 - q1 * q3)) * 180.0f / M_PI;
}

static float GetRollFromQuat(void) {
    return atan2f(2.0f * (q0 * q1 + q2 * q3),
                  q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * 180.0f / M_PI;
}
