#include "MvCamera.h"

Mv_Camera::Mv_Camera()
{
    CameraSdkInit(1);                                // 初始化SDK开发
    handle = -1;                                     // 初始化相机句柄
    tSdkCameraDevInfo infos;                         // 存储相机设备信息
    int nums = 2;                                    // 存储相机数量
    CameraEnumerateDevice(&infos, &nums);            // 枚举所有的相机设备并将其信息写入infos，相机数量写入nums
    Camera_Name = std::string(infos.acFriendlyName); // 录入相机名称
    CameraInit(&infos, -1, -1, &handle);             // 初始化相机，此时会分配相机的句柄
}

Mv_Camera::~Mv_Camera()
{
    CameraUnInit(handle); // 反初始化相机，释放相机资源
}

void Mv_Camera::Close_Camera()
{
    CameraUnInit(handle); // 手动指定反初始化相机，释放资源
}

void Mv_Camera::Open_Camera()
{
    CameraPlay(handle);                                    // 开启相机接受图像
    CameraSetIspOutFormat(handle, CAMERA_MEDIA_TYPE_BGR8); // 设置相机ISP工作在RGB888模式
}

void Mv_Camera::read(cv::Mat &img)
{
    bool status;
    tSdkFrameHead Fhead;                                  // 图像帧头信息，保存该帧图像的一些信息
    BYTE *Buffer;                                         // 图像存储缓冲区
    CameraGetImageBuffer(handle, &Fhead, &Buffer, 100);   // 获取一帧图像
    img = cv::Mat(Fhead.iHeight, Fhead.iWidth, CV_8UC3);  // 初始化空Mat对象
    CameraImageProcess(handle, Buffer, img.data, &Fhead); // 图像处理为RGB888(即CV_8UC3,三通道8位)并录入Inbuffer中
    CameraReleaseImageBuffer(handle, Buffer);             // 释放缓冲区Buffer
}

void Mv_Camera::Set_Ae_Mode(bool Ae_State)
{
    CameraSetAeState(handle, Ae_State);
    Ae_On = Ae_State;
}

bool Mv_Camera::Get_Ae_Mode()
{
    return Ae_On;
}

void Mv_Camera::Set_Ex_Time(int time)
{
    CameraSetExposureTime(handle, time * 1e3); // 设置相机曝光时间，单位为ms
}

double Mv_Camera::Get_Ex_Time()
{
    double time;
    CameraGetExposureTime(handle, &time); // 获取相机曝光时间，单位us
    return int(time / 1e3);               // 返回相机曝光时间，单位ms
}

void Mv_Camera::Set_Camera_Name(std::string name)
{
    CameraSetFriendlyName(handle, (char *)name.c_str()); // 更改相机的名称
}

std::string Mv_Camera::Get_Camera_Name()
{
    return Camera_Name;
}