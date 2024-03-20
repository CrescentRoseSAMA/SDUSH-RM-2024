#ifndef _MV_CAMERA_H_
#define _MV_CAMERA_H_
#include <CameraApi.h>
#include <opencv4/opencv2/opencv.hpp>
#include <string>

/*
 * MindVision相机类，大多数接口仅考虑设备仅接入一个相机的情况
 */
class Mv_Camera
{
private:
    CameraHandle handle; // 相机句柄

    std::string Camera_Name; // 相机名称
    /*
     * 值得注意的是当自动曝光开启时，Set_Ex_Time无法稳定决定相机曝光时间，故想要手动控制曝光时间，
     * 应当关闭自动曝光，即Ae_On = false;
     */
    bool Ae_On; // 是否自动曝光(Auto Expouse),Ae_On = true开启，Ae_On = false关闭
public:
    Mv_Camera(); // 相机初始化

    ~Mv_Camera(); // 相机反初始化,即关闭相机后释放资源

    void Open_Camera(); // 打开相机准备接收图像

    void read(cv::Mat &img); // 获取一帧图像

    void Set_Ex_Time(int time); // 设置曝光时间，单位ms

    double Get_Ex_Time(); // 获取当前曝光时间，单位ms

    std::string Get_Camera_Name(); // 获取相机名称

    void Set_Camera_Name(std::string name); // 设置相机名称，重启后生效

    void Set_Ae_Mode(bool Ae_State); // 设置自动曝光与否

    bool Get_Ae_Mode(); // 获取当前曝光状态，false-手动曝光，true-自动曝光
};

/*
 * 一些布尔返回用于运行时DEBUG，还没想好报错信息怎么写，先空着，可能会爆noreturn的警告
 */
#endif