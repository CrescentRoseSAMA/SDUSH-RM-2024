#ifndef _CAMERA_H_
#define _CAMERA_H_
#include<opencv4/opencv2/opencv.hpp>
#include<CameraApi.h>
using namespace cv;
class Camera
{
    private:
        int iCameraCounts;//相机个数参数
        int iStatus;//运行状态检测参数
        tSdkCameraCapbility tCapability;//存储相机特性的结构体
        tSdkCameraDevInfo tCameraEnumList;//相机的设备信息
        tSdkFrameHead sFrameInfo;//帧头信息，保存着帧图像中的各种参数如大小灰度通道数等
        BYTE*  inBuffer;//输入缓冲区
        int iDisplayFrames;//展示帧数参数
        int channel;//相机通道数参数
        unsigned char *outBuffer;//输出缓冲区
    public:
        int hCamera;//相机句柄，相当于相机的唯一标识
        Camera(int N=10000);
        void read(Mat &In);
        ~Camera();
};
#endif