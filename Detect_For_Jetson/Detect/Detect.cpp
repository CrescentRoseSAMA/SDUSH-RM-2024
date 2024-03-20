#include "TRTModule/TRTModule.hpp"
#include "AngleSolver/AngleSolver.hpp"
#include "MvCamera/MvCamera.h"
#include "Serial/Serial.h"
#include <opencv4/opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include "Utils/Utils.h"
using namespace cv;
using namespace std;
const string onnx_file4 = "../model-cache/model-opt-4.onnx";
const string onnx_file3 = "../model-cache/model-opt-3.onnx";

int main()
{
#if 1
    Timer TimeCount;
    TRTModule Detector(onnx_file4);
    Mv_Camera Cap;
    Serial Seri;
    AngleSolver Angle(Cap.Get_Camera_Name());
    DataPack Data;
    Mat Img;
    bbox_t Armor;
    Seri.uart_setup();
    Cap.Open_Camera();
    Cap.Set_Ae_Mode(false); // 关闭自动曝光
    Cap.Set_Ex_Time(8);     // 设置自动曝光时间为8ms
    while (true)
    {
        TimeCount.Start();
        Cap.read(Img);            // 读入当前帧图像
        auto Res = Detector(Img); // 装甲板检测
        if (!Res.empty())
        {
            bool status = Find_Best_Armor(Res, Armor); // 对装甲板像素面积进行从大到小排序，同时区分敌方与友方装甲板，从敌方中选取面积最大的一个进行打击
            if (status)
            {
                Angle.Angle_Solve(Armor.pts); // 角度解算
                Angle.Get_Datapack(Data);     // 数据包获取
                /*
                串口数据发送 x,y,z,distance,pitch,yaw.
                */
                Seri.send(Data.Tvec[0], Data.Tvec[1], Data.Tvec[2], Data.dist, Data.Angles[0], Data.Angles[1], 1);
            }
            else
                cout << "No Matching Armor !" << '\n';
        }
        Plot_Box(Res, Img); // 画出检测框
        TimeCount.End();
        imshow("Show", Img);
        waitKey(10);
    }
#endif
    return 0;
}