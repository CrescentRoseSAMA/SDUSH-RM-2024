#include "Mv_Camera.h"
using namespace cv;
using namespace std;
int main()
{
    char Flag;
    string FileName, CameraName;
    FileStorage fs;
    Mv_Camera cap;
    cout << "Need to set Camera_Name?[Y/N]" << '\n';
    cin >> Flag;
    if (Flag == 'Y' || Flag == 'y')
    {
        cout << "Please Iuput Camera_Name setting" << '\n';
        cin >> CameraName;
        cap.Set_Camera_Name(CameraName);
        CameraName = cap.Get_Camera_Name();
        cout << "Now Camera_Name is: " << CameraName << '\n';
    }
    cout << "Please Input Absolute Path Of File" << '\n';
    cin >> FileName;
    string name = cap.Get_Camera_Name();
    while (true)
    {
        fs.open(FileName, FileStorage::APPEND);
        cout << "Please Input Camera_Matrix" << '\n';
        double Camera_Matrix[3][3];
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                cin >> Camera_Matrix[i][j];
            }
        }
        cout << '\n';
        cout << "Please Input Distcoeffs" << '\n';
        double Distcoeffs[5];
        cout << '\n';
        for (int i = 0; i < 5; i++)
        {
            cin >> Distcoeffs[i];
        }
        Mat camera_matrix = (Mat_<double>(3, 3) << Camera_Matrix[0][0], Camera_Matrix[0][1], Camera_Matrix[0][2],
                             Camera_Matrix[1][0], Camera_Matrix[1][1], Camera_Matrix[1][2],
                             Camera_Matrix[2][0], Camera_Matrix[2][1], Camera_Matrix[2][2]);
        Mat distcoeffs = (Mat_<double>(5, 1) << Distcoeffs[0], Distcoeffs[1], Distcoeffs[2], Distcoeffs[3], Distcoeffs[4]);
        string Key_Camera_Matrix = name + (string) "_Camera_Matrix";
        fs << Key_Camera_Matrix << camera_matrix;
        string Key_DistCoeffs = name + (string) "_DistCoeffs";
        fs << Key_DistCoeffs << distcoeffs;
        fs.release();
        fs.open(FileName, FileStorage::READ);
        Mat ans1, ans2;
        fs[Key_Camera_Matrix] >> ans1;
        fs[Key_DistCoeffs] >> ans2;
        cout << "Input Over" << '\n';
        cout << "Camera_Matrix :" << '\n';
        cout << ans1 << '\n';
        cout << "DistCoeffs" << '\n';
        cout << ans2 << '\n';
        fs.release();
        cout << "Continue ?[Y/N]" << '\n';
        cin >> Flag;
        if (Flag == 'N' || Flag == 'n')
            break;
    }
    cout << " Script Over " << '\n';
    return 0;
}