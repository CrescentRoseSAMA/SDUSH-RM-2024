#include "TRTModule/TRTModule.hpp"
#include "AngleSolver/AngleSolver.hpp"
#include "MvCamera/MvCamera.h"
#include "Serial/Serial.h"
#include "Kalman/Kalman.h"
#include "Robot/Robot.h"
#include <opencv4/opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include "Utils/Utils.h"
const std::string onnx_file4 = "../model-cache/model-opt-4.onnx";
const std::string onnx_file3 = "../model-cache/model-opt-3.onnx";
const std::string temp_file = "../model-cache/yolov5s.onnx";
using namespace cv;
using namespace std;

/*
 * Mat类的size成员返回矩阵的（宽，高），即返回一个以像素坐标为基准的size类。
 * 其中 宽(Width) = cols（列数），高(height) = rows（行数）
 */
vector<bbox_t> k;
int main()
{
#if 1
    Timer cnt;
    TRTModule Detector(onnx_file4);
    VideoCapture Cap(0);
    string name = "Auto_Name";
    Robot Bot[10];
    for (int i = 0; i < 10; i++)
        Bot[i] = Robot(name, i);
    Serial Seri;
    Seri.uart_setup();
    Mat img;
    while (true)
    {
        vector<vector<bbox_t>> Armor(10);
        bool Idx[10]{false};
        Cap.read(img);
        auto Res = Detector(img);
        if (!Res.empty())
        {
            cout << "Aim's Num : " << Res.size() << endl;
            cout << "Process Start" << endl;
            int cnt = 0;
            for (auto &x : Res)
            {
                Armor[x.tag_id].push_back(x);
                Idx[x.tag_id] = true;
            }
            for (int i = 0; i < 10; i++)
            {
                if (Idx[i])
                {
                    Bot[i].Solve(Armor[i]);
                    Armor[i].clear();
                    Bot[i].Release();
                }
            }
        }
        Plot_Box(Res, img);
        imshow("show", img);
        waitKey(10);
    }
#endif

    return 0;
}