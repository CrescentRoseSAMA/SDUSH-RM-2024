#include "TRTFrame/TRTFrame.hpp"
#include "AngleSolver/AngleSolver.hpp"
#include "MvCamera/MvCamera.h"
#include "Serial/Serial.h"
#include "Kalman/Kalman.h"
#include "Robot/Robot.h"
#include <opencv4/opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include "Utils/Utils.hpp"
#define DEBUG true
const std::string onnx_file4 = "../model-cache/model-opt-4.onnx";
const std::string onnx_file3 = "../model-cache/model-opt-3.onnx";
const std::string temp_file = "../model-cache/0708.onnx";
const std::string onnx_file5 = "../model-cache/file.onnx";
using namespace cv;
using namespace std;

/*
 * Mat类的size成员返回矩阵的（宽，高），即返回一个以像素坐标为基准的size类。
 * 其中 宽(Width) = cols（列数），高(height) = rows（行数）
 */
const InferParam param{
    /*preprocess param*/
    .cvt_code = COLOR_BGR2RGB,
    .input_size = Size(640, 384),
    .normalize = false,
    .hwc2chw = false,

    /*postprocess param*/
    .type = xyxyxyxy,
    .conf_pos = 8,
    .box_pos = 0,
    .conf_thre = 0.6,
    .iou_thre = 0.5,
    .has_sigmoid = false,

    /*class info*/

    .classes_info{{{"Blue", "Red", "Gray", "Purple"}, 9, 4}, {{"guard", "1", "2", "3", "4", "5", "base"}, 13, 7}},
};
int main()
{

#if !DEBUG
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
        Data_Classifier Classifier;
        Cap.read(img);
        auto Res = Detector(img);
        if (!Res.empty())
        {
            cout << "Aim's Num : " << Res.size() << endl;
            cout << "Process Start" << endl;
            Classifier.Clsassify(Res);
            vector<DataPack> Pack;
            for (auto &x : Classifier)
            {
                auto p = Bot[x[0].tag_id].Solve(x);
                Pack.insert(Pack.end(), p.begin(), p.end()); // 合并;
                Bot[x[0].tag_id].Release();
            }

            bool flag = true;
            auto Armor = Find_Best_Armor(Pack, name, flag);
            Show(Armor);
            if (flag)
                Seri.send(Armor, true);
            cout << "Process End" << endl;
        }
        else
            Seri.send(0, 0, 0, 0, 0, 0, false);
        Plot_Box(Res, img);
        imshow("show", img);
        waitKey(10);
    }
#endif
#if 0
    Timer cnt;
    TRTModule Detector(onnx_file4);
    VideoCapture cap("../Assets/test.mp4");
    string name = "Auto_Name";
    Robot Bot[10];
    for (int i = 0; i < 10; i++)
        Bot[i] = Robot(name, i);
    Serial Seri;
    Seri.uart_setup();
    Mat img;
    while (true)
    {
        cap.read(img);
        cnt.Start();
        Data_Classifier Classifier;
        auto Res = Detector(img);
        if (!Res.empty())
        {

            cout << "Aim's Num : " << Res.size() << endl;
            cout << "Process Start" << endl;
            Classifier.Clsassify(Res);
            vector<DataPack> Pack;
            for (auto &x : Classifier)
            {
                auto p = Bot[x[0].tag_id].Solve(x);
                Pack.insert(Pack.end(), p.begin(), p.end()); // 合并;
                Bot[x[0].tag_id].Release();
            }

            bool flag = true;
            auto Armor = Find_Best_Armor(Pack, name, flag);
            Show_Data(Armor, img, Scalar(0, 0, 255), 1, 1);
            if (flag)
                Seri.send(Armor, true);
            cout << "Process End" << endl;
        }
        else
            Seri.send(0, 0, 0, 0, 0, 0, false);
        Plot_Box(Res, img);
        cnt.End();
        imshow("show", img);
        waitKey(20);
    }
#endif
#if 1
    Timer cnt;
    TRTFrame Detector(onnx_file4, param);
    VideoCapture cap("../Assets/a.mp4");
    string name = "Auto_Name";
    Robot Bot[10];
    for (int i = 0; i < 10; i++)
        Bot[i] = Robot(name, i);
    Serial Seri;
    Seri.uart_setup();
    Mat img;
    while (true)
    {
        cap.read(img);
        vector<BoxInfo> box_infos;
        cnt.Start();
        Detector.Run(img, box_infos);
        cnt.End();
        imshow("show", img);
        waitKey(10);
    }

#endif
    return 0;
}