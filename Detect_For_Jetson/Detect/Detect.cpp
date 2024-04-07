#include "TRTModule/TRTModule.hpp"
#include "AngleSolver/AngleSolver.hpp"
#include "MvCamera/MvCamera.h"
#include "Serial/Serial.h"
#include "Kalman/Kalman.h"
#include <opencv4/opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include "Utils/Utils.h"
const std::string onnx_file4 = "../model-cache/model-opt-4.onnx";
const std::string onnx_file3 = "../model-cache/model-opt-3.onnx";

int main()
{
#if 1
    TRTModule Detector(onnx_file4);
    Mv_Camera Cap;
    Serial Seri;
    DataPack Data;
    cv::Mat Img;
    bbox_t Armor;
    std::string Camera_Name;
    Camera_Name = Cap.Get_Camera_Name(); // 获取相机名称，用于后续判断
    std::cout << "相机名称为 :" << Camera_Name << std::endl;
    AngleSolver Angle(Camera_Name); // 初始化角度解算类
    Seri.uart_setup();              // 初始化串口
    Cap.Open_Camera();              // 打开相机，使之可接收图片
    Cap.Set_Ae_Mode(false);         // 关闭自动曝光
    Cap.Set_Ex_Time(5);             // 设置曝光时间为5ms
    while (true)
    {
        Cap.read(Img);            // 读入当前帧图像
        auto Res = Detector(Img); // 装甲板检测

        if (!Res.empty())
        {

            bool status = Find_Best_Armor(Res, Armor, Camera_Name); // 对装甲板像素面积进行从大到小排序，同时区分敌方与友方装甲板，从敌方中选取面积最大的一个进行打击
            if (status)
            {

                Angle.Angle_Solve(Armor.pts); // 角度解算

                //   Angle.Reprojection(Img);
                Angle.Get_Datapack(Data); // 数据包获取
                /*
                串口数据发送 x,y,z,distance,pitch,yaw.
                */
                Seri.send(Data.Tvec[0], Data.Tvec[1], Data.Tvec[2], Data.dist, Data.Angles[0], Data.Angles[1], 1);
                //  cout << Data.Angles[0] << " " << Angle.Yaw << endl;
            }
        }
        else
        {
            Seri.send(0, 0, 0, 0, 0, 0, 0); // 最后一个参数为1表示有装甲板数据发送，为0表示无装甲板数据发送
        }
    }
#endif
#if 0
    Timer TimeCount;
    TRTModule Detector(onnx_file4);
    Mv_Camera Cap;
    Serial Seri;
    DataPack Data;
    Mat Img;
    bbox_t Armor;
    string Camera_Name;
    Camera_Name = Cap.Get_Camera_Name(); // 获取相机名称，用于后续判断
    cout << Camera_Name << endl;
    AngleSolver Angle(Camera_Name); // 初始化角度解算类
    Seri.uart_setup();              // 初始化串口
    Cap.Open_Camera();              // 打开相机，使之可接收图片
    Cap.Set_Ae_Mode(false);         // 关闭自动曝光
    Cap.Set_Ex_Time(5);             // 设置曝光时间为8ms
    while (true)
    {
        //  TimeCount.Start();
        Cap.read(Img);            // 读入当前帧图像
        auto Res = Detector(Img); // 装甲板检测
        if (!Res.empty())
        {
            bool status = Find_Best_Armor(Res, Armor, Camera_Name); // 对装甲板像素面积进行从大到小排序，同时区分敌方与友方装甲板，从敌方中选取面积最大的一个进行打击
            if (status)
            {
                Angle.Angle_Solve(Armor.pts); // 角度解算
                Angle.Reprojection(Img);
                Angle.Get_Datapack(Data); // 数据包获取
                /*
                串口数据发送 x,y,z,distance,pitch,yaw.
                */
                Angle.Tvec_Print();
                // cout << "Dist是: " << Data.dist << endl;
                Seri.send(Data.Tvec[0], Data.Tvec[1], Data.Tvec[2], Data.dist, Data.Angles[0], Data.Angles[1], 1);
                cout << Angle.Pitch << " " << Angle.Yaw << endl;
            }
            // else
            // cout << "No Matching Armor !" << '\n';
        }
        Plot_Box(Res, Img); // 画出检测框
                            //  TimeCount.End();
        imshow("Show", Img);
        waitKey(10);
    }
#endif
    return 0;
}