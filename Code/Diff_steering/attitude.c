#include "attitude.h"
#include "zf_common_headfile.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

//陀螺仪零偏
static float gyro_z_offset = 0.0f;
//当前偏航角速度
static float yaw_rate = 0.0f;
//融合后的偏转角
static float yaw_angle = 0.0f;

//四元数 ->用于Mahony滤波
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float twoKp = 1.0f;      //比例增益
static float twoKi = 0.0f;      //积分增益
static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

//Mahony滤波函数
static void MahonyAHRSupdate(float gx, float gy, float gz,float ax, float ay, float az,float mx, float my, float mz, float dt){
    float norm;
    float hx, hy, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
    float pa, pb, pc;

    float Kp = twoKp;
    float Ki = twoKi;

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

    //归一化加速度计
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return;
    norm = 1.0f / norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    //归一化磁力计
    norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return;
    norm = 1.0f / norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    //磁力计转换坐标系
    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    bx = sqrtf(hx * hx + hy * hy);
    bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

    //重力方向估计
    vx = 2.0f * (q1q3 - q0q2);
    vy = 2.0f * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    //磁场方向估计
    wx = 2.0f * (bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2));
    wy = 2.0f * (bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3));
    wz = 2.0f * (bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2));

    //误差 -> 交叉乘积
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

    //误差 -> 积分
    if (Ki > 0.0f){
        integralFBx += Ki * ex * dt;
        integralFBy += Ki * ey * dt;
        integralFBz += Ki * ez * dt;
        gx += integralFBx;
        gy += integralFBy;
        gz += integralFBz;
    } else{
        integralFBx = 0.0f;
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }

    //使用比例反馈
    gx += Kp * ex;
    gy += Kp * ey;
    gz += Kp * ez;

    //积分四元数
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

//四元数计算偏转角
static float GetYawFromQuat(void){
    return atan2f(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 180.0f / M_PI;
}

void Attitude_Init(void){
    //初始化IMU
    if (imu963ra_init() != 0) {
        while(1);   //初始化失败，可加入提示
    }

    //陀螺仪零偏校准
    const int samples = 100;
    float sum = 0;
    for (int i = 0; i < samples; i++) {
        imu963ra_get_gyro();
        sum += imu963ra_gyro_transition(imu963ra_gyro_z);
        system_delay_us(1000);
    }
    gyro_z_offset = sum / samples;

    //重置四元数
    q0 = 1.0f; q1 = q2 = q3 = 0.0f;
    integralFBx = integralFBy = integralFBz = 0.0f;
}

void Attitude_Update(void){
    //固定 dt 为 5ms与 PIT 中断周期一致
    float dt = 0.005f;

    //读取原始数据
    imu963ra_get_acc();
    imu963ra_get_gyro();
    imu963ra_get_mag();

    //读取原始数据
    imu963ra_get_acc();
    imu963ra_get_gyro();
    imu963ra_get_mag();

    //转换为物理值
    float ax = imu963ra_acc_transition(imu963ra_acc_x);
    float ay = imu963ra_acc_transition(imu963ra_acc_y);
    float az = imu963ra_acc_transition(imu963ra_acc_z);
    float gx = imu963ra_gyro_transition(imu963ra_gyro_x) * M_PI / 180.0f; // 转弧度
    float gy = imu963ra_gyro_transition(imu963ra_gyro_y) * M_PI / 180.0f;
    float gz = (imu963ra_gyro_transition(imu963ra_gyro_z) - gyro_z_offset) * M_PI / 180.0f;
    float mx = imu963ra_mag_transition(imu963ra_mag_x);
    float my = imu963ra_mag_transition(imu963ra_mag_y);
    float mz = imu963ra_mag_transition(imu963ra_mag_z);

    //坐标系调整
    ay = -ay;   //Y左 -> Y右
    az = -az;   //Z上 -> Z下
    my = -my;
    mz = -mz;

    //运行Mahony滤波
    MahonyAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz, dt);

    //提取偏转角
    yaw_angle = GetYawFromQuat();

    //保存偏转角速度
    yaw_rate = gz * 180.0f / M_PI;   //正值 = 左转
}

float Attitude_GetYawRate(void){
    return yaw_rate;
}

float Attitude_GetYaw(void){
    return yaw_angle;
}
