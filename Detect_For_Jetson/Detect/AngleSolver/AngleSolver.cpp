#include "AngleSolver.hpp"
#include <cmath>
#include <cstring>
#define DEGUB 0
using namespace std;

AngleSolver::AngleSolver(string Name)
{
    Camera_Name = Name;
    X_Pose = Y_Pose = Z_Pose = 0;
    Point_3D.clear();
    Point_2D.clear();
    FileStorage fs(Adress_Of_CameraParam, FileStorage::READ);
    fs[Camera_Name + "_Camera_Matrix"] >> Camera_Matrix;
    fs[Camera_Name + "_DistCoeffs"] >> DistCoeffs;
    // 相机的内参矩阵以及畸变矩阵的录入
    fs.release();
    Pitch = Yaw = Distance = 0;
}

AngleSolver::~AngleSolver()
{
    // NULL,无内存申请操作，析构函数无需释放内存
}

/*注意世界坐标与相机坐标*/
/*  相机坐标与世界坐标均采用如下示意
            ^ z轴(物体前方)
           /
          /
         /
        /——————————> x
        |
        |
        |
        V
        y轴

世界坐标位置可以随便，但是注意要和像素坐标对应点位置一致
这里采用从左上角点开始的顺时针来存入
pts[0] -> tl
pts[1] -> tr
pts[2] -> br
pts[3] -> bl
*/
void AngleSolver::Get_Point_3D(const ArmorType type)
{
    double N;
    double M;
    if (type == Big)
    {
        N = Big_Armor_Size.first / 2;
        M = Big_Armor_Size.second / 2;
    }
    else if (type == Small)
    {
        N = Small_Armor_Size.first / 2;
        M = Small_Armor_Size.second / 2;
    }
    else
    {
#if DEBUG
        cout << "AngleSolver::Get_Point_3D ,Error Type of Armor" << endl;
        return;
#endif
    }
    // 以右手笛卡尔坐标系为参照，z轴指向人前
    Point_3D.push_back(Point3f(-N, -M, 0.0)); // 左上tl
    Point_3D.push_back(Point3f(N, -M, 0.0));  // 右上tr
    Point_3D.push_back(Point3f(N, M, 0.0));   // 右下br
    Point_3D.push_back(Point3f(-N, M, 0.0));  // 左下bl
}

/*注意像素坐标*/
/*

        |——————————> x轴
        |
        |
        |
        V
        y轴


根据TRTModule.cpp中Detect类的返回的检测框给出的顺序
2D点数组中的位置为
pts[0] -> tl
pts[1] -> bl
pts[2] -> br
pts[3] -> tr
*/
void AngleSolver::Get_Point_2D(const Point2f Point2D[4])
{
#if DEBUG
    if (Point2D.size() != 4)
    {
        cout << "AngleSolver::Get_Point_2D, Point2D 's Points are to few" << endl;
        return;
    }
#endif
    // 像素坐标系点的存储顺序要和世界坐标系上点的存储顺序一致，否则会出现较大的误差
    Point_2D.push_back(Point2f(Point2D[0])); // tl
    Point_2D.push_back(Point2f(Point2D[3])); // tr
    Point_2D.push_back(Point2f(Point2D[2])); // br
    Point_2D.push_back(Point2f(Point2D[1])); // bl
}
void AngleSolver::Get_Angle_And_Distance()
{
#if DEBUG
    if (Point_3D.size() != 4 || Point_2D.size() != 4)
    {
        cout << "AngleSolver::Get_Angle_And_Distance, Point_3D or 2D 's Points are to few" << endl;
        return;
    }
    if (Camera_Matrix.empty() || DistCoeffs.empty())
    {
        cout << "AngleSolver::Get_Angle_And_Distance, Camera_Calibrate Error" << endl;
        return;
    }
#endif
    // PnP解算，采用重投影迭代法
    solvePnP(Point_3D, Point_2D, Camera_Matrix, DistCoeffs, Rvec, Tvec, false, SOLVEPNP_ITERATIVE);
    X_Pose = Tvec.at<double>(0, 0);
    Y_Pose = Tvec.at<double>(0, 1);
    Z_Pose = Tvec.at<double>(0, 2);
    // Pitch，Yaw角计算
    Pitch = -atan(Y_Pose / sqrt(X_Pose * X_Pose + Z_Pose * Z_Pose)) * 180 / CV_PI; // y/sqrt(z*z+x*x)
    Yaw = atan(X_Pose / Z_Pose) * 180 / CV_PI;
    // 空间距离(欧几里德距离)
    Distance = sqrt(X_Pose * X_Pose + Y_Pose * Y_Pose + Z_Pose * Z_Pose);
}
/*
注意相机坐标
            ^z轴(相机前方)
           /
          /
         /
        /——————————> x轴
        |
        |
        |
        V
        y轴
*/
void AngleSolver::Angle_Compensate()
{
    // 相机坐标系转换到云台或者枪口坐标系下的y轴补偿或者z轴补偿
    // 假定 y_Dist+Y_of_Gun=Y_pose
    // 以及 z_Dist+Z_of_Gun=Z_pose
    Y_Pose -= (Y_Distance_Between_Gun_And_Camera.at(Camera_Name));
    Z_Pose -= (Z_Distance_Between_Gun_And_Camera.at(Camera_Name));
    // 转化到枪口坐标系，重新计算pitch，yaw角度以及距离Distence
    Pitch = -atan(Y_Pose / sqrt(X_Pose * X_Pose + Z_Pose * Z_Pose));
    Yaw = atan(X_Pose / Z_Pose);
    Distance = sqrt(X_Pose * X_Pose + Y_Pose * Y_Pose + Z_Pose * Z_Pose);
}

void AngleSolver::Angle_Solve(const Point2f Point[4])
{
    ArmorType Type = Get_Armor_Type(Point);
    Get_Point_3D(Type); // 世界坐标获取

    Get_Point_2D(Point); // 像素坐标获取

    Get_Angle_And_Distance(); // 相机坐标系下的角度解算以及距离计算

    // Angle_Compensate(); // 偏移量补偿，将相机坐标系下的yaw，pitch角转化为枪口坐标系下的yaw，pitch以及Distance
}

void AngleSolver::Get_Datapack(DataPack &Data)
{
    Data.Tvec[0] = Tvec.at<double>(0, 0); // x
    Data.Tvec[1] = Tvec.at<double>(0, 1); // y
    Data.Tvec[2] = Tvec.at<double>(0, 2); // z
    Data.Angles[0] = Pitch;
    Data.Angles[1] = Yaw;
    Data.dist = Distance;
    clear(); // 清空两个坐标系
}

void AngleSolver::Tvec_Print() const
{
    cout << Tvec << endl;
}

void AngleSolver::clear()
{
    Point_2D.clear();
    Point_3D.clear();
}

ArmorType AngleSolver::Get_Armor_Type(const Point2f pos[4])
{
    double W = pos[3].x - pos[0].x; // tr - tl 长度
    double H = pos[1].y - pos[0].y; // bl - tl 宽度
    double Ratio = W / H;
    return fabs(Ratio - 3.9655) > fabs(Ratio - 2.3150) ? Small : Big;
}

void AngleSolver::Reprojection(Mat &img) // 重投影测试
{
    Mat R;
    Rodrigues(Rvec, R);         // 罗德里格斯公式旋转向量转旋转矩阵
    vector<Point2d> Projection; // 重投影点存储
    for (auto &x : Point_3D)    // 将世界坐标系点通过PNP算出的旋转矩阵和平移向量以及相机内参转为像素坐标系下的点
    {
        Mat res = (Mat_<double>(3, 1) << x.x, x.y, x.z);                           // 世界坐标系矩阵（3*1）
        Mat ans = R * res + Tvec;                                                  // 世界坐标转相机坐标下
        ans /= ans.at<double>(2, 0);                                               // 转换到归一化坐标系下，但为矫正畸变
        Mat Puv = Camera_Matrix * ans;                                             // 归一化坐标系转换到像素坐标系下
        Projection.push_back(Point2d(Puv.at<double>(0, 0), Puv.at<double>(1, 0))); // 角点存储
    }
    line(img, Projection[0], Projection[2], Scalar(0, 0, 255), 3);
    line(img, Projection[1], Projection[2], Scalar(0, 0, 255), 3);
    line(img, Projection[1], Projection[3], Scalar(0, 0, 255), 3);
    line(img, Projection[3], Projection[0], Scalar(0, 0, 255), 3); // 四点划线
}
/*
    todo...
    1.弹丸重力或者空气阻力补偿
    2.卡尔曼滤波预测
*/