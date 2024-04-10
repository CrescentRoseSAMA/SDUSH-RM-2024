#ifndef _ANGLESOLVER_H_
#define _ANGLESOLVER_H_
#include <algorithm>
#include <opencv4/opencv2/opencv.hpp>
#include <vector>
/**************************************
Pitch -- Y
Yaw   -- X
**************************************/

/*
 *装甲板类型定义，装甲板尺寸类型定义，角度解算数据包定义
 */
typedef std::pair<int, int> ArmorSize;
typedef struct Pack
{
    double Tvec[3];   // 平移向量 x,y,z
    double Angles[2]; // 角度 pitch/yaw
    double dist;      // 距离
} DataPack;
/*
 *大小装甲板世界坐标基准，first = 长 ， second = 宽，对应像素坐标下框起来的部分,单位为mm
 */
const ArmorSize Big_Armor_Size(230, 127);
const ArmorSize Small_Armor_Size(132, 54);

/*
 *装甲板类型定义
 */
enum ArmorType
{
    Big = 1,
    Small
};

/*
 *矫正参数
 */

const std::unordered_map<std::string, double> Y_Distance_Between_Gun_And_Camera{
    {"Infantry_big", 0}, {"Infantry_small", 0}, {"Sentry", 0}, {"Hero", 0}, {"MV-1", 0}}; // 枪口与相机间的Y轴坐标
const std::unordered_map<std::string, double> Z_Distance_Between_Gun_And_Camera{
    {"Infantry_big", 0}, {"Infantry_small", 0}, {"Sentry", 0}, {"Hero", 0}, {"MV-1", 0}}; // 枪口与相机间的Z轴坐标

/*
 *相机内参矩阵以及畸变矩阵存储位置
 */

const std::string Adress_Of_CameraParam = "../Assets/CameraParam.yaml";

/*
 *敌方与友方颜色定义
 */
enum Color
{
    Blue = 0,
    Red,
    Gray
}; // 定义参考上交TRTModule.hpp中的color.id设置
const int Friend_Color = Blue;
const int Enemy_Color = Red;

class AngleSolver
{
public:
    std::string Camera_Name;
    std::vector<cv::Point3f> Point_3D;
    std::vector<cv::Point2f> Point_2D;
    cv::Mat Camera_Matrix;
    cv::Mat DistCoeffs;
    cv::Mat Rvec;
    cv::Mat Tvec;
    double X_Pose;
    double Y_Pose;
    double Z_Pose;
    double Pitch;
    double Yaw;
    double Distance;

public:
    AngleSolver(std::string Name = (std::string) "Auto_Name"); // 构造函数，传入相机名称，通过相机名称选取对应内参,无参数选取名称为Auto_Name的一组内参

    ~AngleSolver();

    ArmorType Get_Armor_Type(const cv::Point2f pos[4]); // 根据像素坐标判断装甲板类型

    void Get_Point_3D(ArmorType type); // 获取世界坐标系，参数为装甲板的类型，其中Big=1，Small=2

    void Get_Point_2D(const cv::Point2f Point2D[4]); // 获取像素坐标系

    void Get_Angle_And_Distance(); // 获取角度以及直线距离

    void Angle_Solve(const cv::Point2f Point2D[4]); // 角度解算，传入装甲板的类型以及像素坐标系下的vector数组

    void Angle_Compensate(); // 偏移量补偿

    void Gravity_Compensate(); // 重力补偿

    void Tvec_Print() const; // 打印平移向量用于Debug

    void Get_Datapack(DataPack &Data); // 获取数据包,(x,y,z,pitch,yaw,distance)

    void Reprojection(cv::Mat &img); // 重投影测试

    void clear();

    /*
        todo .....
        1.重力补偿以及空气阻力补偿
        2.卡尔曼滤波预测
    */
};

#endif