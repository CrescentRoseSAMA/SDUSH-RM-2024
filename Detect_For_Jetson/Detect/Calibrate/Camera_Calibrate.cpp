#include<opencv4/opencv2/opencv.hpp>
#include<iostream>
using namespace std;
using namespace cv;
void Camera_Calibrate(const string CameraMatrix_Route,const string DistCoeffs_Route)
{
      // 1. 准备标定棋盘图像
    int boardWidth = 7;  // 棋盘格横向内角点数量
    int boardHeight = 7; // 棋盘格纵向内角点数量，为黑白块总数减去1
    float squareSize = 1.f; // 棋盘格格子的大小，单位为米,随便设置，不影响相机内参计算
    Size boardSize(boardWidth, boardHeight);

    vector<vector<Point3f>> objectPoints;//世界坐标
    vector<vector<Point2f>> imagePoints;//像素坐标
    vector<Point2f> corners;//棋盘格角点存储

    // 2. 拍摄棋盘图像
    Mat image, gray;
    namedWindow("image", WINDOW_NORMAL);
    vector<String> fileNames;
    glob("../imgfile/*.jpg", fileNames);//从img文件夹中读取相机拍摄的棋盘格标定图，并将路径存储在filename中
    for (size_t i = 0; i < fileNames.size(); i++)
    {
        image = imread(fileNames[i], IMREAD_COLOR);
      
        cvtColor(image, gray, COLOR_BGR2GRAY);
        
        // 3. 读入图像数据，并提取角点,将角点的像素坐标存储在corners中
        bool found = findChessboardCorners(image, boardSize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
        if (found)
        {
            cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));//精确化角点的位置
            drawChessboardCorners(image, boardSize, corners, found);
          
            vector<Point3f> objectCorners;
            for (int j = 0; j < boardHeight; j++)
            {
                for (int k = 0; k < boardWidth; k++)
                {
                    objectCorners.push_back(Point3f(k * squareSize, j * squareSize, 0));
                }
            }
            objectPoints.push_back(objectCorners);
            imagePoints.push_back(corners);
        }else{printf("No");return ;}
    }
    //4. 标定相机
    vector<Mat> rvecs, tvecs;
    Mat Camera_Matrix,DistCoeffs;
    calibrateCamera(objectPoints, imagePoints, image.size(),Camera_Matrix,DistCoeffs, rvecs, tvecs);   
    FileStorage fs(CameraMatrix_Route,FileStorage::WRITE);
   fs<<"Camera_Matrix"<<Camera_Matrix;
    cout<<Camera_Matrix;
    fs.release();
    FileStorage  qs(DistCoeffs_Route,FileStorage::WRITE);
    qs<<"DistCoeffs"<<DistCoeffs;
    cout<<DistCoeffs;
    qs.release();
}