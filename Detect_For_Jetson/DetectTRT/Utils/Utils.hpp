#ifndef _UTILS_H_
#define _UTILS_H_
#include <chrono>
#include <iostream>
#include <cmath>
#include "../TRTFrame/TRTFrame.hpp"
#include "../AngleSolver/AngleSolver.hpp"
#include "../MvCamera/MvCamera.h"

const cv::Scalar colors[4]{cv::Scalar(255, 0, 0), cv::Scalar(0, 0, 255), cv::Scalar(128, 128, 128), cv::Scalar(128, 0, 128)};
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
    long long Get_Duration();
};

/*
 * 对推理出的结果按id进行分类
 */
struct Data_Classifier
{
    bool id_exist[10];
    int id_mapto_boxid[10];
    std::vector<std::vector<BoxInfo>> Box;
    Data_Classifier() : id_exist{false}, id_mapto_boxid{-1} {};
    void Clsassify(std::vector<BoxInfo> &box)
    {
        for (auto &x : box)
        {
            if (!id_exist[x.classes[1].first])
            {
                id_exist[x.classes[1].first] = true;
                std::vector<BoxInfo> tmp{x};
                Box.emplace_back(tmp);
                id_mapto_boxid[x.classes[1].first] = Box.size() - 1;
            }
            else
                Box[id_mapto_boxid[x.classes[1].first]].emplace_back(x);
        }
    }

    auto begin() { return Box.begin(); };
    auto end() { return Box.end(); };
};

/*
 * 常用函数接口
 */
void Show_Data(DataPack &pack, cv::Mat &des, cv::Scalar fontcolor = cv::Scalar(0, 128, 255), int fontsize = 1, int thickness = 1);
double Get_Area(cv::Point2f pts[4]);                                                                // 通过海伦公式计算不规则装甲板的面积
DataPack &Find_Best_Armor(std::vector<DataPack> &pack, const std::string &Camera_Name, bool &flag); // 选择当前最佳装甲板，以距离以及敌友方进行筛选
#endif