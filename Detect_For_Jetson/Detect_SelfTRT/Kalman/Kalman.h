#ifndef _KALMAN_H_
#define _KALMAN_H_
#include <Eigen/Core>
#include <Eigen/Dense>
/*
 * Kalman滤波模板类，不考虑控制量B以及u
 * 模板dimx = 状态量维度，dimz = 测量量维度
 */
template <int dimx, int dimz>
class Kalman
{
    /*
     * 常用矩阵定义
     */
public:
    using Vec_x = Eigen::Matrix<double, dimx, 1>;
    using Vec_z = Eigen::Matrix<double, dimz, 1>;
    using Mat_xx = Eigen::Matrix<double, dimx, dimx>;
    using Mat_zx = Eigen::Matrix<double, dimz, dimx>;
    using Mat_xz = Eigen::Matrix<double, dimx, dimz>;
    using Mat_zz = Eigen::Matrix<double, dimz, dimz>;

private:
    Vec_x x;  // 状态量，dimx * 1 维度列向量
    Vec_z z;  // 测量量，dimz * 1 维度列向量
    Mat_xx A; // 状态转移矩阵， dimx * dimx 维度矩阵
    Mat_zx H; // 观测转移矩阵， dimz * dimx 维度矩阵
    Mat_xx P; // 状态量的误差协方差矩阵， dimx * dimx 维度矩阵
    Mat_xx Q; // 状态噪声协方差矩阵， dimx * dimx 维度矩阵
    Mat_zz R; // 测量噪声协方差矩阵， dimz * dimz 维度矩阵
    Mat_xz K; // 卡尔曼增益， dimx * dimz 维度矩阵
    Mat_xx I; // 单位矩阵， dimx * dimx 维度矩阵

public:
    Kalman();
    ~Kalman();
    void Init(Vec_x _x, Mat_xx _P, Mat_xx A_, Mat_zx H_, Mat_xx Q_, Mat_zz R_);
    Vec_x Predict();
    Vec_x Predict(Mat_xx B, Vec_x u);
    void Update(Vec_z _z);
    Vec_x Get_Best_x();
};

template class Kalman<2, 2>;
template class Kalman<4, 2>;
template class Kalman<4, 4>;
#endif