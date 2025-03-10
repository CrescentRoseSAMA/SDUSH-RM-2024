#ifndef _ANGLESOLVER_H_
#define _ANGLESOLVER_H_
#include <algorithm>
#include <opencv4/opencv2/opencv.hpp>
#include <vector>
#include <cstdio>
/**************************************
Pitch -- Y
Yaw   -- X
**************************************/

/*
 *  装甲板尺寸类型定义
 */
typedef std::pair<double, double> ArmorSize;

/*
 *  大小装甲板世界坐标基准，first = 长 ， second = 宽，对应像素坐标下框起来的部分,单位为mm
 */
const ArmorSize Big_Armor_Size(230.0, 58.0);
const ArmorSize Small_Armor_Size(132.0, 54.0);

/*
 *装甲板类型定义
 */
enum ArmorType
{
    Big = 1,
    Small
};

/*
 * 决定大小装甲板的结构定义
 */
typedef struct Type_Decide
{
    bool Decided;
    int cnt;
    int Big_Times;
    int Small_Times;
    double Big_Ratio = Big_Armor_Size.first / Big_Armor_Size.second;
    double Small_Ratio = Small_Armor_Size.first / Small_Armor_Size.second;
    Type_Decide()
    {
        cnt = 0;
        Big_Times = 0;
        Small_Times = 0;
        Decided = false;
    }

    ArmorType Decide(double Ratio)
    {
        cnt++;
        return fabs(Ratio - Big_Ratio) > fabs(Ratio - Small_Ratio) ? (Small_Times++, Small) : (Big_Times++, Big);
    }

    ArmorType Final_Type()
    {
        return Big_Times > Small_Times ? Decided = true, Big : Decided = true, Small;
    }
} Type_Decide;

/*
 *矫正参数
 */
const std::unordered_map<std::string, double> Y_Distance_Between_Gun_And_Camera{
    {"Infantry_big", 0},
    {"Infantry_small", 0},
    {"Sentry", 0},
    {"Hero", 0},
    {"MV-1", 0}}; // 枪口与相机间的Y轴坐标
const std::unordered_map<std::string, double> Z_Distance_Between_Gun_And_Camera{
    {"Infantry_big", 0},
    {"Infantry_small", 0},
    {"Sentry", 0},
    {"Hero", 0},
    {"MV-1", 0}}; // 枪口与相机间的Z轴坐标

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
    Gray,
    Purple,
}; // 定义参考上交TRTModule.hpp中的color.id设置
const Color Friend_Color = Blue;
const Color Enemy_Color = Red;

/*
 * 角度解算数据包定义
 */
typedef struct Pack
{
    double P_c[3];    // 相机坐标系下坐标 x,y,z
    double Angles[2]; // 角度 pitch/yaw
    double dist;      // 距离
    cv::Mat R;        // 旋转矩阵
    cv::Mat tvec;     // 平移向量
    Color color;      // 敌方装甲板颜色
    int id;           // 该装甲板的id
    ArmorType type;   // 装甲板类型(大小)
} DataPack;
class AngleSolver
{
public:
    std::string Camera_Name;
    ArmorType Type;
    Type_Decide Decide;
    std::vector<cv::Point2f> P_uv; // tmp var， need empty for every loop
    cv::Mat Camera_Matrix;
    cv::Mat DistCoeffs;
    cv::Mat Rvec;
    cv::Mat Tvec;
    double X;
    double Y;
    double Z;
    double Pitch;
    double Yaw;
    double Distance;

public:
    AngleSolver(){};

    ~AngleSolver();

    AngleSolver(const std::string &name);

    void Get_Armor_Type(const cv::Point2f pos[4]);
    void Get_Armor_Type(const std::vector<float> &pos_xyxyxyxy);

    std::vector<cv::Point3f> Get_P_w();

    void Get_P_uv(const cv::Point2f P_uv[4]);
    void Get_P_uv(const std::vector<float> &P_uv_xyxyxyxy);

    void Pnp_Solve();

    DataPack Solve(const cv::Point2f Point2D[4]);
    DataPack Solve(const std::vector<float> &Point2D_xyxyxyxy);

    void Offset_Compensate();
    void Gravity_Compensate();

    DataPack Pack();

    void Armor_Reprojection(cv::Mat &img);
    void Reprojection(cv::Mat &img, const cv::Point3f &P_w);

    void Reprojection(cv::Mat &img, const cv::Mat &P_w);

    void Tvec_Print() const;
    /*
        todo .....
        1.重力补偿以及空气阻力补偿
        2.卡尔曼滤波预测
    */
};

#endif