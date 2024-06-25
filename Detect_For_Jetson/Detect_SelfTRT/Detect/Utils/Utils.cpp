#include "Utils.hpp"
using namespace std;
using namespace cv;
Timer::~Timer()
{
}

void Timer::Start()
{
    start = std::chrono::steady_clock::now();
    std::cout << "测速开始" << '\n';
}

void Timer::End()
{
    end = std::chrono::steady_clock::now();
    std::cout << "测速段" << Timer_ID << "结束，用时：" << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << '\n';
}

long long Timer::Get_Duration()
{
    end = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
}

/*
 * 对检测完毕的图像画出检测出来的装甲板结果
 */
void Plot_Box(vector<bbox_t> &res, Mat &img)
{
    for (auto &obj : res)
    {
        cv::line(img, obj.pts[0], obj.pts[1], colors[2], 2);
        cv::line(img, obj.pts[0], obj.pts[2], colors[2], 2);
        cv::line(img, obj.pts[1], obj.pts[3], colors[2], 2);
        cv::line(img, obj.pts[3], obj.pts[2], colors[2], 2);
        cv::putText(img, std::to_string(obj.tag_id), obj.pts[0], cv::FONT_HERSHEY_SIMPLEX, 1, colors[obj.color_id]);
    }
    resize(img, img, {1280, 960});
}

/*
 * 海伦公式计算装甲板面积
 */
double Get_Area(Point2f pts[4])
{
    double a = sqrt(pow(pts[0].x - pts[1].x, 2) + pow(pts[0].y - pts[0].y, 2));
    double b = sqrt(pow(pts[1].x - pts[2].x, 2) + pow(pts[1].y - pts[2].y, 2));
    double c = sqrt(pow(pts[2].x - pts[3].x, 2) + pow(pts[2].y - pts[3].y, 2));
    double d = sqrt(pow(pts[3].x - pts[0].x, 2) + pow(pts[3].y - pts[0].y, 2));
    double z = (a + b + c + d) / 2;
    double Size = 2 * sqrt((z - a) * (z - b) * (z - c) * (z - d));
    return Size;
}

/*
 *  打印数据包用于Debug
 */
void Show_Data(DataPack &pack, Mat &des, Scalar fontcolor, int fontsize, int thickness)
{
    string idx = to_string(pack.id);
    string pitch = to_string(int(pack.Angles[0] * 1e3) / 1e3); // 保留三位小数(直接截断)
    pitch.erase(pitch.end() - 3, pitch.end());
    string yaw = to_string(int(pack.Angles[1] * 1e3) / 1e3);
    yaw.erase(yaw.end() - 3, yaw.end());
    string dist = to_string(int(pack.dist * 1e3) / 1e3);
    dist.erase(dist.end() - 3, dist.end());
    string x = to_string(int(pack.P_c[0] * 1e3) / 1e3);
    x.erase(x.end() - 3, x.end());
    string y = to_string(int(pack.P_c[1] * 1e3) / 1e3);
    y.erase(y.end() - 3, y.end());
    string z = to_string(int(pack.P_c[2] * 1e3) / 1e3);
    z.erase(z.end() - 3, z.end());
    string color = pack.color == Blue ? "Blue" : "Red";
    string type_ = pack.type == Big ? "Big" : "Small";
    putText(des, "id:" + idx, {5, 15}, FONT_HERSHEY_PLAIN, fontsize, fontcolor, thickness);
    putText(des, "pitch:" + pitch + " " + "yaw:" + yaw + " " + "dist:" + dist, {5, 30}, FONT_HERSHEY_PLAIN, fontsize, fontcolor, thickness);
    putText(des, "x:" + x + " " + "y:" + y + " " + "z:" + z, {5, 45}, FONT_HERSHEY_PLAIN, fontsize, fontcolor, thickness);
    putText(des, "color:" + color, {45, 15}, FONT_HERSHEY_PLAIN, fontsize, colors[pack.color], thickness);
    putText(des, "type:" + type_, {135, 15}, FONT_HERSHEY_PLAIN, fontsize, fontcolor, thickness);
}

/*
 * 筛选最佳装甲板，无结果返回false, 判断依据为boundingbox面积
 */
bbox_t &Find_Best_Armor(vector<bbox_t> &res, string &Camera_Name, bool &flag)
{
    flag = true;
    std::sort(res.begin(), res.end(), [](bbox_t &res1, bbox_t &res2)
              { return Get_Area(res1.pts) > Get_Area(res2.pts); }); // 按面积进行从大到小排序
    if (Camera_Name == "Sentry")                                    // 若是哨兵，选择击打颜色来区分友方敌方
    {
        for (auto &x : res)
        {
            if (x.color_id == Enemy_Color) // 从大到小寻找敌方最大装甲板
            {
                flag = true;
                return x;
            }
        }
        flag = false;
    }
    return res[0];
}

/*
 *  筛选最佳装甲板， 无结果返回false，判断依据为欧式距离Dist
 */

DataPack &Find_Best_Armor(vector<DataPack> &pack, const string &Camera_Name, bool &flag)
{
    flag = true;
    sort(pack.begin(), pack.end(), [](DataPack &pack1, DataPack &pack2)
         { return pack1.dist > pack2.dist; });
    if (Camera_Name == "Sentry")
    {
        for (auto &x : pack)
        {
            if (x.color == Enemy_Color)
            {
                flag = true;
                return x;
            }
        }
        flag = false;
    }
    return pack[0];
}
