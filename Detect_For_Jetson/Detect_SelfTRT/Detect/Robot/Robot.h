#include "../AngleSolver/AngleSolver.hpp"
#include "../TRTModule/TRTModule.hpp"
#include <opencv4/opencv2/opencv.hpp>
#include <vector>

class Robot
{
private:
    double center_r;

    double robot_id;

    AngleSolver solver;

    std::vector<DataPack> Pack; // tmp var， need empty for every loop

public:
    Robot(const std::string &camera_name, int idx);

    Robot(){};

    ~Robot(){};

    std::vector<DataPack> Solve(const std::vector<bbox_t> &Res);

    void Center_to_Pixel(cv::Mat &img);

    void Release();

    DataPack &operator[](int idx) { return Pack[idx]; }

    /*
     *  一些向量基本运算求解
     */
    double Len_vec(const cv::Mat &Vec);
    double Dot_vec(const cv::Mat &Vec1, const cv::Mat &Vec2);
    double Euclid_vec(const cv::Mat &Vec1, const cv::Mat &Vec2);
    double Angle_vec(const cv::Mat &Vec1, const cv::Mat &Vec2);
};