#include "MvCamera.h"
using namespace cv;
using namespace std;
int main()
{
    string name;
    Mv_Camera cap;
    double K[3][3];
    double Dist[5];
    FileStorage fs;
    string Path;
    char flag;
    // 设置相机名称以及检查相机名称
    name = cap.Get_Camera_Name();
    cout << "相机名称为 :" << name << endl;
    cout << "是否更改相机名称[Y/N]" << endl;
    cin >> flag;
    if (flag == 'Y' || flag == 'y')
    {
        cout << "输入相机名称" << endl;
        name.clear();
        cin >> name;
        cap.Set_Camera_Name(name);
        cap.Close_Camera();
        cout << "重启以更改" << endl;
        return 0;
    }
    cout << "请输入参数文件路径" << endl;
    cin >> Path;
    fs.open(Path, FileStorage::APPEND);
    cout << "输入内参矩阵，是否转置[Y/N]" << endl;
    cin >> flag;
    cout << "请输入" << endl;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            cin >> K[i][j];
        }
    cout << "输入畸变参数(k1,k2,k3,p1,p2)" << endl;
    for (int i = 0; i < 5; i++)
        cin >> Dist[i];
    Mat Camera_Matrix = (Mat_<double>(3, 3) << K[0][0], K[0][1], K[0][2],
                         K[1][0], K[1][1], K[1][2],
                         K[2][0], K[2][1], K[2][2]);
    Mat Distcoeffs = (Mat_<double>(5, 1) << Dist[0], Dist[1], Dist[3], Dist[4], Dist[2]);
    if (flag == 'Y' || flag == 'y')
        fs << name + "_Camera_Matrix" << Camera_Matrix.t();
    else
        fs << name + "_Camera_Matrix" << Camera_Matrix;
    fs << name + "_DistCoeffs" << Distcoeffs;
    fs.release();
    fs.open(Path, FileStorage::READ);
    fs[name + "_Camera_Matrix"] >> Camera_Matrix;
    fs[name + "_DistCoeffs"] >> Distcoeffs;
    cout << "输入的内参矩阵为\n"
         << Camera_Matrix << endl;
    cout << "输入的畸变参数为\n"
         << Distcoeffs << endl;
    fs.release();
    cout << "Over" << endl;
    return 0;
}