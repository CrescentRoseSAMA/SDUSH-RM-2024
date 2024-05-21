#include <opencv4/opencv2/opencv.hpp>
#include "Total_Est.h"
#include <cmath>

Total_Est::Total_Est()
{
    r = 0;
    can_update = false;
}

Total_Est::~Total_Est()
{
}

void Total_Est::Check_can_update()
{
    if (Armor_Queue.size() >= 2)
        can_update = true;
    else
        can_update = false;
}

void Total_Est::Push(Pose_Box para)
{
    Armor_Queue.push(para);
    Check_can_update();
}

bool Total_Est::Update()
{
    if (!can_update)
        return can_update;
    else
    {
        Pose_Box A1 = Armor_Queue.back();
        Armor_Queue.pop();
        Pose_Box A2 = Armor_Queue.back();
        Armor_Queue.pop();
        Check_can_update();
        double Length = Get_Dist_Between_Vectors3d(A1.tvec, A2.tvec);
        double angle = Get_Angle_Of_Vectors3d(A1.tvec, A2.tvec);
        r = Length / 2 * sin(angle / 2); // 等腰三角形模型
    }
    return can_update;
}

cv::Point3d Total_Est::Estimate_Pc(Pose_Box Est)
{
    cv::Mat Pw = (cv::Mat_<double>(3, 1) << 0, 0, r); // 世界坐标下z轴加r
    cv::Mat Pc = Est.R * Pw * Est.tvec;               // Pc = R * Pw + t转换到相机坐标下
    return cv::Point3d(Pc.at<double>(0, 0), Pc.at<double>(0, 1), Pc.at<double>(0, 2));
}

double Total_Est::Get_Len3d(cv::Mat Vec)
{
    return sqrt(pow(Vec.at<double>(0, 0), 2) + pow(Vec.at<double>(0, 1), 2) + pow(Vec.at<double>(0, 2), 2));
}

double Total_Est::Get_Dot3d(cv::Mat Vec1, cv::Mat Vec2)
{
    return Vec1.at<double>(0, 0) * Vec2.at<double>(0, 0) +
           Vec1.at<double>(0, 1) * Vec2.at<double>(0, 1) +
           Vec1.at<double>(0, 2) * Vec2.at<double>(0, 2);
}

double Total_Est::Get_Dist_Between_Vectors3d(cv::Mat Vec1, cv::Mat Vec2)
{
    double len = sqrt(pow(Vec1.at<double>(0, 0) - Vec2.at<double>(0, 0), 2) +
                      pow(Vec1.at<double>(0, 1) - Vec2.at<double>(0, 1), 2) +
                      pow(Vec1.at<double>(0, 2) - Vec2.at<double>(0, 2), 2));
    return len;
}

double Total_Est::Get_Angle_Of_Vectors3d(cv::Mat Vec1, cv::Mat Vec2)
{
    return acos(Get_Dot3d(Vec1, Vec2) / (Get_Len3d(Vec1) * Get_Len3d(Vec2))); // a * b = |a|*|b|*cos<a,b> --> <a,b> = arccos(a*b/(|a|*|b|))
}