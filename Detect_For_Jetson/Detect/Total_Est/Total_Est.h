#ifndef __TOTAL_EST__
#define __TOTAL_EST__
#include <queue>
#include <opencv4/opencv2/opencv.hpp>
#include "../TRTModule/TRTModule.hpp"

/*
 * 整车状态估计类
 */
struct Pose_Box
{
    cv::Mat R;    // 旋转矩阵，是世界系变换到相机系的旋转矩阵
    cv::Mat tvec; // 平移向量
};

class Total_Est
{
private:
    std::queue<Pose_Box> Armor_Queue; // 存储装甲板的信息，队列存储
    double r;                         // 存储装甲板中心与整车中心的距离即半径
    bool can_update;                  // 决定是否可以更新
public:
    Total_Est();
    ~Total_Est();
    void Check_can_update();
    void Push(Pose_Box para); // 向队列中增加一个装甲板信息
    bool Update();            // 更新一次r
    cv::Point3d Estimate_Pc(Pose_Box Est);

    double Get_Len3d(cv::Mat Vec);
    double Get_Dot3d(cv::Mat Vec1, cv::Mat Vec2);
    double Get_Dist_Between_Vectors3d(cv::Mat Vec1, cv::Mat Vec2);
    double Get_Angle_Of_Vectors3d(cv::Mat Vec1, cv::Mat Vec2);
};

#endif