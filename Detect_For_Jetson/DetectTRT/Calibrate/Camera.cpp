#include<CameraApi.h>
#include<iostream>
#include"Camera.h"
#include<opencv4/opencv2/opencv.hpp>
using namespace std;
using namespace cv;
Camera::Camera(int N)
{
    iCameraCounts=1;
    iStatus=-1;
    channel=3;
    iDisplayFrames=N;
    CameraSdkInit(1);//SDK初始化，提示语言选择中文（1）
    iStatus=CameraEnumerateDevice(&tCameraEnumList,&iCameraCounts);//相机枚举设备，传入相机的设备信息并向其中存入数据
    if(iCameraCounts==0)//检测是否有相机连接
    {
        cout<<"No Camera";
    }else
    {
        cout<<"Status="<<iStatus<<" Count="<<iCameraCounts;
    }
    iStatus=CameraInit(&tCameraEnumList,-1,-1,&hCamera);//使用相机相关接口前要先初始化相机，传入设备信息，-1代表相机参数组使用上次退出时的参数，以及传入相机的句柄用以区分多相机
    if(iStatus!=CAMERA_STATUS_SUCCESS)//使用接口自带的常量来检测是否初始化成功
        cout<<"Camera_Init_Erorr"<<endl;
    CameraGetCapability(hCamera,&tCapability);//获取设备的特性描述的结构体，其中存储着一些设备的参数范围及特性，句柄hCamera依旧用来标识特定相机
    CameraPlay(hCamera);//设定Sdk为工作状态，使之接受相机的图像
    if(tCapability.sIspCapacity.bMonoSensor)
    {
        channel=1;//黑白相机只需要单通道
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_MONO8);
        //设定相机Isp的输出格式为8位单通道
    }else
    {
        channel=3;//彩色相机需要三通道RGB
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_BGR8);
        //设定相机Isp的输出格式为24通道真彩色
    }
    outBuffer=new unsigned char[tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3];//乘以三因为3通道
    //至此相机初始化完毕
}

void Camera::read(Mat &In)
{
    
    if(CameraGetImageBuffer(hCamera,&sFrameInfo,&inBuffer,1000)==CAMERA_STATUS_SUCCESS)
    {
        CameraReleaseImageBuffer(hCamera,inBuffer);//调用释放inbuff的内存，防止阻塞
        CameraImageProcess(hCamera,inBuffer,outBuffer,&sFrameInfo);
        Mat img(Size(sFrameInfo.iWidth,sFrameInfo.iHeight),sFrameInfo.uiMediaType==CAMERA_MEDIA_TYPE_MONO8?CV_8UC1:CV_8UC3,outBuffer);
        img.copyTo(In);
    }
}
Camera::~Camera()
{
    CameraUnInit(hCamera);//相机反初始化释放资源
    delete []outBuffer;//释放申请的输出缓冲区
}
