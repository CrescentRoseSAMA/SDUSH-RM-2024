#include "Robot.h"
#include <cstdio>
using namespace std;
using namespace cv;

Robot::Robot(const string &camera_name, int idx)
{
    this->solver = AngleSolver(camera_name);
    this->center_r = 0;
    this->robot_id = idx;
}

/*
 * 解算装甲板返回解算后的相机坐标系值，同时在允许情况下估计装甲板中心到中心的距离r
 */

void Robot::Solve(const vector<bbox_t> &Res)
{
    for (auto Armor : Res)
    {
        auto data = solver.Solve(Armor.pts);
        Pack.push_back(data);
    }
    if (Pack.size() >= 2)
    {
        DataPack Armor_1 = Pack[0];
        DataPack Armor_2 = Pack[1];
        double Len = Euclid_vec(Armor_1.tvec, Armor_2.tvec);
        Mat Armor_1_z_unitvec = (Mat_<double>(3, 1) << 0, 0, 1);
        Mat Armor_2_z_unitvec = Armor_1_z_unitvec.clone();
        Mat Unitvec1_P_c = Armor_1.R * Armor_1_z_unitvec + Armor_1.tvec;
        Mat Unitvec2_P_c = Armor_2.R * Armor_2_z_unitvec + Armor_2.tvec;
        Unitvec1_P_c -= Armor_1.tvec;
        Unitvec2_P_c -= Armor_2.tvec;
        double angle = Angle_vec(Unitvec1_P_c, Unitvec2_P_c);
        center_r = Len / 2 * sin(angle / 2);
    }
    else
        cout << "=====装甲板不满两个，无法估计中心=====\n";
}

/*
 * 手动释放资源
 */
void Robot::Release()
{
    Pack.clear();
}

/*
 *  反投影中心到图像上
 */
void Robot::Center_to_Pixel(Mat &img)
{
    Mat Z_Unitvec = (Mat_<double>(3, 1) << 0, 0, center_r);
    solver.Reprojection(img, Z_Unitvec);
}

/*
 * 求n维向量模长
 */
double Robot::Len_vec(const Mat &Vec)
{
    int rows = Vec.size().height; // 不妨注意一下像素系中横轴为u(x),纵轴为v(y),而size类中height-》y，width-》x
    int cols = Vec.size().width;
    double pow_sum = 0;
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            pow_sum += pow(Vec.at<double>(i, j), 2);
    return sqrt(pow_sum);
}

/*
 *  求n维向量点积
 */
double Robot::Dot_vec(const Mat &Vec1, const Mat &Vec2)
{
    int rows = Vec1.size().height;
    int cols = Vec1.size().width;
    double res = 0;
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            res += Vec1.at<double>(i, j) * Vec2.at<double>(i, j);
    return res;
}

/*
 *  求两点间的欧式距离
 */
double Robot::Euclid_vec(const Mat &Vec1, const Mat &Vec2)
{
    double pow_sum;
    int rows = Vec1.size().height;
    int cols = Vec1.size().width;
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            pow_sum += pow(Vec1.at<double>(i, j) - Vec2.at<double>(i, j), 2);
    return sqrt(pow_sum);
}

/*
 * 求两向量间夹角
 */
double Robot::Angle_vec(const Mat &Vec1, const Mat &Vec2)
{
    return acos(Dot_vec(Vec1, Vec2) / (Len_vec(Vec1) * Len_vec(Vec2))); // a * b = |a|*|b|*cos<a,b> --> <a,b> = arccos(a*b/(|a|*|b|))
}