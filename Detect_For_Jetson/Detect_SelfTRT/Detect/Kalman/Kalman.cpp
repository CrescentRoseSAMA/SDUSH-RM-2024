#include "Kalman.h"
#include <unistd.h>
#include <iostream>
using namespace std;
template <int dimx, int dimz>
Kalman<dimx, dimz>::Kalman()
{
}

template <int dimx, int dimz>
Kalman<dimx, dimz>::~Kalman()
{
}

template <int dimx, int dimz>
void Kalman<dimx, dimz>::Init(Vec_x _x, Mat_xx _P, Mat_xx _A, Mat_zx _H, Mat_xx _Q, Mat_zz _R)
{
    this->x = _x;
    this->P = _P;
    this->A = _A;
    this->H = _H;
    this->Q = _Q;
    this->R = _R;
    this->I.setIdentity();
}

/*
 * 这里的Kalman<dimx, dimz>::Vec_x是整一个返回类型，因为Vec_x是在类内定义的类型
 */
template <int dimx, int dimz>
typename Kalman<dimx, dimz>::Vec_x Kalman<dimx, dimz>::Predict()
{
    x = A * x;                     // 状态预测估计，不带输入
    P = A * P * A.transpose() + Q; // 预测估计误差协方差矩阵
    return x;
}

template <int dimx, int dimz>
typename Kalman<dimx, dimz>::Vec_x Kalman<dimx, dimz>::Predict(Mat_xx B, Vec_x u)
{
    x = A * x + B * u; // 状态预测估计，带输入
    cout << "X" << x << endl;
    P = A * P * A.transpose() + Q; // 预测估计误差协方差矩阵
    return x;
}

template <int dimx, int dimz>
void Kalman<dimx, dimz>::Update(Vec_z _z)
{
    K = P * H.transpose() * (H * P * H.transpose() + R).inverse(); // 计算卡尔曼增益
    x = x + K * (_z - H * x);                                      // 更新最优状态值
    P = (I - K * H) * P;
}

template <int dimx, int dimz>
typename Kalman<dimx, dimz>::Vec_x Kalman<dimx, dimz>::Get_Best_x()
{
    return x;
}