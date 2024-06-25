#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include "Camera.h"
using namespace std;
using namespace cv;
const string CameraMatrix_Route = "../../Assets/CameraMatrix.xml";
const string DistCoeffs_Route = "../../Assets/DistCoeffs.xml";
void Camera_Calibrate(const string CameraMatrix_Route,const string DistCoeffs_Route);
int main()
{
     /*
     以下实现图像采集，按p键将相机当前帧图像采集并存在imgfile路径中
     */
    #if 1
     Mat img;
     Camera cap;
     string imgfile = "./img/Calibrate0.jpg"; //存储拍摄图像的路径，文件夹命名应为img，图像命名随意
     char key = 0;
     while(true)
     { 
          cap.read(img);
          imshow("img",img); 
          key=waitKey(10);
          if(key == 'p')
          {
               imwrite(imgfile,img);
               imgfile[15]++; 
          }
     }
     #endif 
     return 0;
     #if 0
     Camera_Calibrate(CameraMatrix_Route,DistCoeffs_Route); //标定，会从上述的imgfile中读取棋盘格图像，务必保证图像完整清晰,传参为参数存储位置
     #endif
}