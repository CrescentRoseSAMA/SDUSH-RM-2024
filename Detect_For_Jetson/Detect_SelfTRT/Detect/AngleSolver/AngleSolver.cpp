#include "AngleSolver.hpp"
#include <opencv4/opencv2/opencv.hpp>
#include <cmath>
#include <string>
using namespace cv;
using namespace std;

/*
 *  @brief: 构造函数，传入相机名称，初始化相机参数
 *
 *  @param name: 相机名称
 *
 *  @return: none
 *
 */
AngleSolver::AngleSolver(const string &name)
{
    Camera_Name = name;
    X = Y = Z = 0;
    P_uv.clear();
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

/*
 *  @brief: 通过像素坐标长宽比判断装甲板类型
 *
 *  @param pos: 装甲板四个顶点像素坐标
 *
 *  @return: none
 *
 */

void AngleSolver::Get_Armor_Type(const cv::Point2f pos[4])
{
    double W = pos[3].x - pos[0].x; // tr - tl 长度
    double H = pos[1].y - pos[0].y; // bl - tl 宽度
    double Ratio = W / H;
    if (Decide.cnt < 50)
        Type = Decide.Decide(Ratio);
    else
        Type = Decide.Final_Type();

    /*
     * 或许可以考虑只比较长度而不比较宽度？
     */
}

/*
 *  @brief: 获取装甲板四个顶点的世界坐标
 *
 *  @param: none
 *
 *  @return: P_w: 装甲板四个顶点的世界坐标
 *
 *  @note:
 *   相机坐标与世界坐标均采用如下示意
 *          ^ z轴(物体前方)
 *          /
 *        /
 *        /
 *       /——————————> x
 *       |
 *       |
 *       |
 *       V
 *       y轴
 *
 *   世界坐标位置可以随便，但是注意要和像素坐标对应点位置一致
 *   这里采用从左上角点开始的顺时针来存入
 *   pts[0] -> tl
 *   pts[1] -> tr
 *   pts[2] -> br
 *   pts[3] -> bl
 */

vector<Point3f> AngleSolver::Get_P_w()
{
    vector<Point3f> P_w;
    double N;
    double M;
    if (Type == Big)
    {
        N = Big_Armor_Size.first / 2.0;
        M = Big_Armor_Size.second / 2.0;
    }
    else if (Type == Small)
    {
        N = Small_Armor_Size.first / 2.0;
        M = Small_Armor_Size.second / 2.0;
    }
    // 以右手笛卡尔坐标系为参照，z轴指向人前
    P_w.push_back(cv::Point3f(-N, -M, 0.0)); // 左上tl
    P_w.push_back(cv::Point3f(N, -M, 0.0));  // 右上tr
    P_w.push_back(cv::Point3f(N, M, 0.0));   // 右下br
    P_w.push_back(cv::Point3f(-N, M, 0.0));  // 左下bl
    return P_w;
}

/*
 *  @brief: 获取装甲板四个顶点的像素坐标
 *
 *  @param P_uv_: 识别到的装甲板四个顶点的像素坐标
 *
 *  @return: none
 *
 *  @note:
 *     注意像素坐标如下
 *
 *       原点(图像左上)|——————————> x轴
 *                   |
 *                   |
 *                   |
 *                   V
 *                   y轴
 *
 *
 *   根据TRTModule.cpp中Detect类的返回的检测框给出的顺序
 *   2D点数组中的位置为
 *   pts[0] -> tl
 *   pts[1] -> bl
 *   pts[2] -> br
 *   pts[3] -> tr
 */
void AngleSolver::Get_P_uv(const cv::Point2f P_uv_[4])
{
    /*
     * 像素坐标系点的存储顺序要和世界坐标系上点的存储顺序一致，否则会出现较大的误差
     */
    P_uv.clear();
    P_uv.push_back(cv::Point2f(P_uv_[0])); // tl
    P_uv.push_back(cv::Point2f(P_uv_[3])); // tr
    P_uv.push_back(cv::Point2f(P_uv_[2])); // br
    P_uv.push_back(cv::Point2f(P_uv_[1])); // bl
}

/*
 *  @brief: 角度解算函数, 计算装甲板四个顶点的世界坐标，并计算相机与装甲板的距离，俯仰角，偏航角
 *
 *  @param: none
 *
 *  @return: none
 *
 */

void AngleSolver::Pnp_Solve()
{
    auto P_w = Get_P_w();
    // PnP解算，采用重投影迭代法
    solvePnP(P_w, P_uv, Camera_Matrix, DistCoeffs, Rvec, Tvec, false, SOLVEPNP_ITERATIVE);
    X = Tvec.at<double>(0, 0);
    Y = Tvec.at<double>(1, 0);
    Z = Tvec.at<double>(2, 0);
    // Pitch，Yaw角计算, 为角度值( * 180 / pie)
    Pitch = -atan(Y / sqrt(X * X + Z * Z)) * 180 / CV_PI; // y/sqrt(z*z+x*x)
    Yaw = atan(X / Z) * 180 / CV_PI;
    // 直线距离
    Distance = sqrt(X * X + Y * Y + Z * Z);
}

/*
 *  @brief: 角度解算中心函数, 整合各个步骤，并返回数据打包
 *
 *  @param P_uv_: 识别到的装甲板四个顶点的像素坐标
 *
 *  @return: DataPack结构体，为各个数据的打包
 *
 */

DataPack AngleSolver::Solve(const cv::Point2f P_uv_[4])
{
    Get_P_uv(P_uv_);

    if (!Decide.Decided)
        Get_Armor_Type(P_uv_);

    Pnp_Solve();

    // Offset_Compensate();

    return Pack();
}

/*
 *  @brief: 偏移补偿函数, 补偿相机与枪口的偏移,并重新计算俯仰角，偏航角，距离
 *
 *  @param: none
 *
 *  @return: none
 *
 */

void AngleSolver::Offset_Compensate()
{
    /*相机坐标系转换到云台或者枪口坐标系下的y轴补偿或者z轴补偿
     * 假定 y_Dist+Y_of_Gun=Y_pose
     * 以及 z_Dist+Z_of_Gun=Z_pose
     */
    Y -= (Y_Distance_Between_Gun_And_Camera.at(Camera_Name));
    Z -= (Z_Distance_Between_Gun_And_Camera.at(Camera_Name));
    // 转化到枪口坐标系，重新计算pitch，yaw角度以及距离Distence
    Pitch = -atan(Y / sqrt(X * X + Z * Z));
    Yaw = atan(X / Z);
    Distance = sqrt(X * X + Y * Y + Z * Z);
}

/*
 *  @brief: 重力补偿函数, 补偿重力的影响,并重新计算俯仰角，偏航角，距离
 *
 *  @param: none
 *
 *  @return: none
 *
 *  @note: 目前未启用，到时候应传入弹丸速度，原理为斜抛运动模型，估计重力影响下弹丸的偏移，未测试过
 *
 */

void AngleSolver::Gravity_Compensate()
{
    // 考虑重力影响下弹丸偏移的补偿
    double v = 0; // Bullet_Speed;
    double d = Distance;
    double Target_y = Y;
    double Temp_y = Target_y;
    double Acc = 30;                         // 误差估计
    double Temp_Pitch = Pitch * CV_PI / 180; // 角度转化弧度
    // 通过不断迭代来补偿抬高枪口，直到误差小于规定尺度结束补偿
    while (Acc >= 0.01)
    {
        // 飞行时间
        double t = v * cos(Temp_Pitch) / (d * cos(Temp_Pitch));
        // 抛物线模型与匀速直线模型落点的y轴差距
        double Height = v * sin(Temp_Pitch) * t - 0.5 * 9.8 * t * t;
        double Delta_Height = Target_y - Height;
        // 差距补偿
        Temp_y += Delta_Height;
        // 重新计算俯仰角度,枪管抬高
        Temp_Pitch = -atan(Temp_y / sqrt(X * X + Z * Z));
        // 更新误差估计
        Acc = Delta_Height;
    }
    // 更新计算俯仰角的y轴坐标以及pitch角
    Z = Temp_y;
    Pitch = Temp_Pitch * 180 / CV_PI;
}

/*
 *  @brief: 数据打包函数, 打包各个数据，并返回
 *
 *  @param: none
 *
 *  @return: DataPack结构体，为各个数据的打包
 */

DataPack AngleSolver::Pack()
{
    DataPack Package;
    Package.P_c[0] = X;
    Package.P_c[1] = Y;
    Package.P_c[2] = Z;
    Package.Angles[0] = Pitch;
    Package.Angles[1] = Yaw;
    Package.dist = Distance;
    Rodrigues(Rvec, Package.R);
    Package.tvec = Tvec.clone();
    Package.type = Type;
    return Package;
}

/*
 *  @brief: 装甲板重投影到像素坐标下
 *
 *  @param img: 要绘制重投影的图像
 *
 *  @return: none
 *
 */
void AngleSolver::Armor_Reprojection(Mat &img)
{
    auto P_w = Get_P_w();
    Mat R;
    Rodrigues(Rvec, R);         // 罗德里格斯公式旋转向量转旋转矩阵
    vector<Point2f> Projection; // 重投影点存储
    for (auto &x : P_w)         // 将世界坐标系点通过PNP算出的旋转矩阵和平移向量以及相机内参转为像素坐标系下的点
    {
        cv::Mat res = (cv::Mat_<double>(3, 1) << x.x, x.y, x.z);                       // 世界坐标系矩阵（3*1）
        cv::Mat ans = R * res + Tvec;                                                  // 世界坐标转相机坐标下
        ans /= ans.at<double>(2, 0);                                                   // 转换到归一化坐标系下，但为矫正畸变
        cv::Mat Puv = Camera_Matrix * ans;                                             // 归一化坐标系转换到像素坐标系下
        Projection.push_back(cv::Point2d(Puv.at<double>(0, 0), Puv.at<double>(1, 0))); // 角点存储
    }
    cv::line(img, Projection[0], Projection[2], cv::Scalar(0, 0, 255), 3);
    cv::line(img, Projection[1], Projection[2], cv::Scalar(0, 0, 255), 3);
    cv::line(img, Projection[1], Projection[3], cv::Scalar(0, 0, 255), 3);
    cv::line(img, Projection[3], Projection[0], cv::Scalar(0, 0, 255), 3); // 四点划线
}

/*
 *  @brief: 重投影函数, 绘制重投影点
 *
 *  @param img: 要绘制重投影的图像
 *  @param P_w: 要绘制的点的世界坐标
 *
 *  @return: none
 *
 *  @note:
 * 基本公式如下
 * P_c = R_c2w * P_w + tvec
 * P_uv = 1/Z * K * P_c
 */

void AngleSolver::Reprojection(Mat &img, const Point3f &P_w)
{
    Mat R;
    Rodrigues(Rvec, R);                                  // 罗德里格斯旋转向量转旋转矩阵
    Mat P = (Mat_<double>(3, 1) << P_w.x, P_w.y, P_w.z); // 世界坐标矩阵
    Mat ans = R * P + Tvec;                              // 世界坐标转相机坐标
    ans /= ans.at<double>(2, 0);                         // 转归一化坐标
    Mat Puv = Camera_Matrix * ans;                       // 转化到像素坐标下
    Point2f P_uv_ = Point2f(Puv.at<double>(0, 0), Puv.at<double>(1, 0));
    circle(img, P_uv_, 3, Scalar(128, 0, 128), 3);
}

void AngleSolver::Reprojection(Mat &img, const Mat &P_w)
{
    Mat R;
    Rodrigues(Rvec, R);            // 罗德里格斯旋转向量转旋转矩阵
    Mat ans = R * P_w + Tvec;      // 世界坐标转相机坐标
    ans /= ans.at<double>(2, 0);   // 转归一化坐标
    Mat Puv = Camera_Matrix * ans; // 转化到像素坐标下
    Point2f P_uv_ = Point2f(Puv.at<double>(0, 0), Puv.at<double>(1, 0));
    circle(img, P_uv_, 3, Scalar(128, 0, 128), 3);
}
/*
 *  @brief: 打印平移向量函数, 打印平移向量
 *
 *  @param: none
 *
 *  @return: none
 */

void AngleSolver::Tvec_Print() const
{
    cout << "平移向量为：" << Tvec << endl;
}