### 1. 环境配置

> + gcc-11/g++-11(支持c++20标准)，安装方法见[gcc-11/g++-11安装](https://www.cnblogs.com/CrescentWind/p/17978193)
> 
> + cuda,cudnn,tensorrt无需配置，jetpack已经安装好了
>
> + fmt 编译安装即可(打包在内)
>
> + 迈德威视相机linux开发SDK(官网下载安装)
>
> + vscode 工具包选择gcc-11
>
> + 初次编译应删除原始build文件夹
### 2.编译阶段会出现的问题
> + 更改TRTModule文件夹中FindTensorrt.cmake文件中的第二行
```txt
set(TensorRT_ROOT /home/ruby/Tensorrt/trt)
中的第二部分改为nano的tensorrt文件夹所在的路径，nano下应该是
/usr/src/tensorrt,该文件用于寻找tensorrt以及设置一些编译时cmake需要的变量值
`````
> + 更改TRTMoudle/TRTModule.hpp

```c++
#include "/home/ruby/Tensorrt/trt/include/NvInfer.h"
改为
#include "NvInfer.h"
```

> + NvInferRuntimeCommon.h，找不到<cuda_runtime_api.h>

解决方法
```txt
找不到，直接将头文件改为绝对路径形式
打开NvInferRuntimeCommon.h
sudo gedit/vim NvInferRuntimeCommon.h
找到cuda_runtime_api.h的路径，在nano上为/usr/local/cuda-11.
/include/cuda_runtime_api.h。
将<cuda_runtime_api.h>改为"/usr/local/cuda-11.
/include/cuda_runtime_api.h"
``` 

### 3.程序结构
文件树如下
```txt
├── AngleSolver #角度解算相关
│   ├── AngleSolver.cpp
│   ├── AngleSolver.hpp
│   └── CMakeLists.txt
├── Assets #相机内参以及畸变矩阵存储，以及一个测试视频
│   ├── Armor.mp4
│   ├── CameraMatrix.xml
│   └── DistCoeffs.xml
├── Calibrate #标定相关程序
│   ├── Camera_Calibrate.cpp #标定函数存放
│   ├── Camera.cpp
│   ├── Camera.h
│   ├── Catch&Calibrate.cpp #标定主程序
│   └── CMakeLists.txt
├── Camera #相机相关
│   ├── Camera.cpp
│   ├── Camera.h
│   └── CMakeLists.txt
├── CMakeLists.txt #总CMakeLists
├── Detect.cpp #检测主程序
├── model-cache #tensorrt onnx模型存储
│   ├── model-opt-3.onnx
│   ├── model-opt-4.cache
│   └── model-opt-4.onnx
├── Serial #预留给串口
│   └── CMakeLists.txt
└── TRTModule #tensorrt部署相关
    ├── CMakeLists.txt
    ├── FindTensorRT.cmake
    ├── TRTModule.cpp
    └── TRTModule.hpp
```

### 4.主要类及其接口与参数
>* AngleSolver类(/AngleSolver)
```c++
/*
大小装甲板的实际长度存储，要量出来更改(识别框对应部分)
*/
const ArmorSize Big_Armor_Size(230,127);
const ArmorSize Small_Armor_Size(68,30);

/*
参数：  1.装甲板类型(Small/Big)
       2.像素坐标系下的坐标
作用：  角度解算
*/
void Angle_Solve(ArmorType type,const Point2f Point2D[4]);

/*
参数： 数据包结构
作用： 将解算数据打包传出
*/
void Get_Datapack(DataPack &Data); 
```

>* Mv_Camera类(/Mv_Camera)
```c++
/*
参数： Opencv矩阵图像类
作用： 读取当前相机一帧
*/
 void read(Mat &In);
```

>* TRTModule类(/TRTModule)
```c++
/*
构造函数，传入onnx模型的路径
*/
TRTModule(const std::string &onnx_file)

/*
ps:重载了"()"运算符
参数:   Opencv矩阵图像类
作用：  识别图像中的装甲板，并将数据打包存储在一个vector中。vector的size就是该图像中装甲板的个数
返回：  类型参数为bbox_t的vector, 其中bbox_t这一结构在TRTModule.hpp中定义,包含识别到的装甲版的四点像素坐标(可直接传给Angle_Solve这一函数)，置信度，装甲板颜色，装甲板数字
*/
std::vector<bbox_t> operator()(const cv::Mat &src) const;

/*
ps：该函数并非TRTModule类的接口，直接调用即可
参数： 上述函数解算后的返回值,以及当前帧图像
作用： 在当前帧中框出检测到的装甲板
*/
void Plot_Box(std::vector<bbox_t> res , cv::Mat &img);
```
### 5.示例demo
>见`Detect.cpp`

### 6.相机标定
>相机使用前要标定,标定程序在Calibrate文件夹中

>拍摄的图片会存储在/Calibrate/imgfile中，没有就新建一个

标定步骤

> 1. 拍棋盘格照片，务必保证图片平整无褶皱，拍摄时保证拍摄的图片清晰完整，包含整个棋盘格。拍摄时应多角度多方位的拍摄，最好有15张有效图片。
> 2. 标定，调用以下函数即可，参数会自动保存到相关路径(检查一下，~~万一我相对路径写错了~~)

 ```c++
void Camera_Calibrate(const string CameraMatrix_Route,const string DistCoeffs_Route);
```
><font color = red>若标定出错或者标定完目标路径无参数，检查拍摄的图片是否清晰，是否包含整个棋盘格，然后重新调用标定函数 </font>

>标定板棋盘格图像存放在/Detect/Assets目录下，名字为标定板.jpg。为8\*8棋盘格，内角点数为7\*7。若采用其他形式，在Camera_Calibrate.cpp开头修改

### 7.杂项

> 所有文件中的相对路径均以build文件夹为相对
>
> 采用的坐标系形式在AngleSolver.cpp中用字符画出，可以与电控对一对双方相机坐标系的形式是否相同
>
> 模型初始化时若提醒Tactic Device request ........然后崩溃，建议解除nano的硬件限制或者关闭图形化使用ssh远程命令行模式再跑一遍(~~虽然自带缓存不需要这样做~~)
>
> ssh远程连接见 [ssh](https://www.cnblogs.com/CrescentWind/p/17976485)，解除硬件限制可以在jtop中实现，或者终端输入`sudo jetson_clocks`
