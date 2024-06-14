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
    /*infer param*/
    .topk = false,
    .topk_num = 128,
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
#if 1
    Timer cnt;
    TRTFrame Detector(onnx_file4, param);
    VideoCapture cap("../Assets/Armor.mp4");
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
        if (!box_infos.empty())
        {
            Data_Classifier classifier;
            classifier.Clsassify(box_infos);
            vector<DataPack> Data;
            for (const auto &Armor : classifier)
            {
                auto p = Bot[Armor[0].classes[1].first].Solve(Armor);
                Data.insert(Data.end(), p.begin(), p.end());
            }
            bool flag = true;
            auto Armor = Find_Best_Armor(Data, name, flag);
            Show_Data(Armor, img, Scalar(0, 0, 255), 1, 2);
            if (flag)
                Seri.send(Armor, true);
        }
        cnt.End();
        imshow("show", img);
        waitKey(10);
    }

#endif
    return 0;
}