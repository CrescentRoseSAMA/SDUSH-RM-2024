#ifndef _UTILS_H_
#define _UTILS_H_
#include <chrono>
#include <iostream>
#include <cmath>
#include "../AngleSolver/AngleSolver.hpp"
#include "../TRTModule/TRTModule.hpp"
#include "../MvCamera/MvCamera.h"

/*
 * 计时类
 */
class Timer
{
private:
    std::chrono::time_point<std::chrono::steady_clock> start; // 时间戳起点
    std::chrono::time_point<std::chrono::steady_clock> end;   // 时间戳终点
    int Timer_ID;

public:
    Timer(int ID) { Timer_ID = ID; };
    Timer() { Timer_ID = 0; };
    ~Timer();
    void Start(); // 放置起始时间点，应用于测速段之前
    void End();   // 放置结束时间点并打印起始到结束的时间，应用于测速段之后,每次调用Point_Count加1
};

/*
 * 常用函数接口
 */

double Get_Area(cv::Point2f pts[4]);                                               // 通过海伦公式计算不规则装甲板的面积
bool Find_Best_Armor(std::vector<bbox_t> &res, bbox_t &Armor, string Camera_Name); // 选择当前最佳装甲板，以像素面积以及敌友方进行筛选
void Plot_Box(std::vector<bbox_t> &res, cv::Mat &img);                             // 在当前帧图像上画出识别到的装甲板
#endif