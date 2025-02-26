#include "header.h"
#include "RobotArmControl.h"





////ע��
// �궨��Ҫ�޸����꣨ʹ�ú��⻹�����������б궨��  �����������
// 
// 185�У��޸ĺ������ո���
// 




// ����궨����
cv::Mat_<double> ThermalMatrix;
cv::Mat_<double> H_Thermal2Gripper;
cv::Mat_<double> CameraMatrix;
cv::Mat_<double> H_Camera2Gripper;

//��������ָ��cv::Mat
std::shared_ptr<geometry::PointCloud> pcl_ptr = std::make_shared<geometry::PointCloud>();

//�����е��λ��
cv::Mat ToolPoseCamera;
cv::Mat ToolPoseThermal;

//����任����
vector<cv::Mat>  H_Gripper2Base_cameraVec;
vector<cv::Mat>  H_Base2Gripper_cameraVec;
vector<cv::Mat> H_Base2Gripper_thermalVec;

vector<Eigen::Vector3d> thermalPositionVec;
vector<Eigen::Vector3d> cameraPositionVec;
vector<cv::Mat> thermalImageVec;
vector<cv::Mat> depthImageVec;
vector<cv::Mat> colorImageVec;

void GetColorFromImageVec(Eigen::Vector3d& colorTmp, const cv::Mat& p_base, string thermalOrCamera);
void convertDepthTocamera(const ushort& u, const ushort& v, ushort& d, double& worldX, double& worldY, double& worldZ) {   
    double tx = (u - ThermalMatrix.at<double>(0,2)) / ThermalMatrix.at<double>(0, 0);
    double ty = (v - ThermalMatrix.at<double>(1, 2)) / ThermalMatrix.at<double>(1, 1);
    worldX = d * tx;
    worldY = d * ty;
    worldZ = d;
}


struct Image {
    Image(int _imageIndex, int _u, int _v, int _distance) {
        imageIndex = _imageIndex;
        distance = _distance;
        u = _u;
        v = _v;
    }
    int imageIndex;
    int distance;
    int u;
    int v;
};

//��е�۲ɼ�����ʱ����̬
vector<string> robotPositionCamera;
vector<string> robotPositionThermal;
vector<string> vConfigInfo;
AstraCameraD2C* astraCameraD2C;
unordered_map<int, string> mCam2Name;
cv::Mat colorImage(IMAGE_HEIGHT_480, IMAGE_WIDTH_640, CV_8UC3);
cv::Mat depthImage(IMAGE_HEIGHT_480, IMAGE_WIDTH_640, CV_16UC1);//640x480
double rgbdWeight = 0.; // �������ع�Ȩ��
int reconMode = 0.; // �ع�ģʽ��0-ֻ�ع���άģ�ͣ�1-�ع���άģ���Լ�����ӳ��
int thermalF = 0;//�Ƿ���Ҫ���ƻ�е�۲ɼ����⣬1����Ҫ��0������Ҫ

void ReadTxtFile(const string& path, vector<string>& vRes) {
    ifstream fs;
    fs.open(path, std::ios::in);
    if (!fs.is_open()) {
        cout << "file not exist!" << endl;
        return;
    }
    string buf;
    while (getline(fs, buf)) {
        vRes.push_back(buf);
    }
    fs.close();
}

void SaveImg(const string& root, int imgIndex) {
    char indexBuf[10];
    sprintf_s(indexBuf, "%04d", imgIndex);
    string colorSavePath = root + "rgb/" + string(indexBuf) + ".jpg";
    string depthSavePath = root + "depth/" + string(indexBuf) + ".png";
    cout << "���ͼ·��Ϊ��" + colorSavePath << endl;
    cv::Mat colorImageDup, depthImageDup;
    cv::flip(colorImage, colorImageDup, 1);
    cv::flip(depthImage, depthImageDup, 1);
    cv::imwrite(colorSavePath, colorImageDup);
    cv::imwrite(depthSavePath, depthImageDup); // PNG16
}

vector<vector<double>> ConvertString2Point(const vector<string>& vPosStr) {
    vector<vector<double>> vvOut(vPosStr.size());
    // 516.6533169874918, 0, 313.9209304288961;
    // 0.01438109095153672, 0.9993166025230528, 0.03405161001812367, -280.9031215821541;
    for (int i = 0; i < vPosStr.size(); ++i) {
        string currStr = vPosStr[i];
        while (1) {
            int idx = currStr.find(',');
            string bitStr;
            if (idx == -1) {
                // ����ÿ�����һλ����
                bitStr = currStr.substr(0, bitStr.size() - 1);
                vvOut[i].emplace_back(atof(bitStr.c_str()));
                break;
            }
            else {
                bitStr = currStr.substr(0, idx);
                vvOut[i].emplace_back(atof(bitStr.c_str()));
                currStr = currStr.substr(idx + 1);
            }
        }
    }

    return vvOut;
}

bool AnalysisString2Mat(const vector<string>& vPosStr, cv::Mat& intrinsic, cv::Mat& handeye) {
    // ��Ч����֤
    if (vPosStr.size() != 7) return false;
    // ����ת��
    auto vvPosDouble = ConvertString2Point(vPosStr);
    // ��Ч����֤
    for (int i = 0; i < 7; ++i) {
        if (i < 3) {
            if (vvPosDouble[i].size() != 3) return false;
        }
        else {
            if (vvPosDouble[i].size() != 4) return false;
        }
    }
#if 0
    cout << "������������Ϣ��";
    for (int i = 0; i < vvPosDouble.size(); ++i) {
        for (int j = 0; j < vvPosDouble[i].size(); ++j) {
            cout << vvPosDouble[i][j] << "  ";
        }
        cout << endl;
    }
#endif
    // ����д��
    for (int i = 0; i < vvPosDouble.size(); ++i) {
        if (i < 3) {
            for (int j = 0; j < vvPosDouble[i].size(); ++j) {
                intrinsic.at<double>(i, j) = vvPosDouble[i][j];
            }
            //intrinsic.at<double>(i, 3) = 0.; // �����Ϊ��0
        }
        else {
            for (int j = 0; j < vvPosDouble[i].size(); ++j) {
                handeye.at<double>(i - 3, j) = vvPosDouble[i][j];
            }
        }
    }

    return true;
}



/// <summary>
/// �����ļ�
/// </summary>
/// <returns></returns>
/// 
void getthmeral() {
    string result_root = "./datas/";
    cout << GetSystemDate() << endl;
    vector<string> dir_name;
    vector<string> out_name;
    string input_root = "\\\\192.168.2.2\\c\\MAG\\" + GetSystemDate();

    ObtainDirInfo(input_root, dir_name);
    out_name = GetThermal_Dir(dir_name, 2);//�޸ĸ���
    for (int i = 0; i < out_name.size(); i++) {
        cout << out_name[i] << endl;
    }
    for (int i = 0; i < out_name.size(); i++) {
        string thermal_data = input_root + "\\" + out_name[i] + "\\Processed Data\\PCA\\0001.jpg";
        cout << thermal_data << endl;
        cv::Mat src = cv::imread(thermal_data);
        //cv::imshow("src", src);
        //cv::waitKey(0);
        cv::imwrite(result_root + "thermal/000" + to_string(i) + ".jpg", src);
    }
}


int main()
{
#pragma comment(lib, "comsuppw.lib") // ���char* ת wchar_t����
#pragma comment(lib, "Ws2_32.lib") // �޷�����inet_addr

    astraCameraD2C = new AstraCameraD2C();
    mCam2Name[0] = "rgb/";
    mCam2Name[1] = "thermal/";
    mCam2Name[2] = "hdrgb/";
    string root = "./datas/"; // �궨ͼƬ·��
    // ��ȡ�����ļ�
    ReadTxtFile(root + "config.txt", vConfigInfo);
    if (vConfigInfo.size() == 0) return 0;
    // ��ȡRGBD��е������
    ReadTxtFile(root + "rgbdpoint.txt", robotPositionCamera);
    if (robotPositionCamera.size() == 0) return 0;
    // ��ȡRGBD����ڲΡ����۱궨ת����ϵ
    vector<string> vPosInfo;
    ReadTxtFile(root + "rgbdcal.txt", vPosInfo);
    CameraMatrix = cv::Mat::zeros(cv::Size(4, 3), CV_64F);
    H_Camera2Gripper = cv::Mat::zeros(cv::Size(4, 4), CV_64F);
    if (!AnalysisString2Mat(vPosInfo, CameraMatrix, H_Camera2Gripper)) return 0;
    // ��ȡ����������ڲΡ����۱궨ת����ϵ
    vector<string> vPosInfo_Th;
    ReadTxtFile(root + "thermalcal.txt", vPosInfo_Th);
    ThermalMatrix = cv::Mat::zeros(cv::Size(4, 3), CV_64F);
    H_Thermal2Gripper = cv::Mat::zeros(cv::Size(4, 4), CV_64F);
    if (!AnalysisString2Mat(vPosInfo_Th, ThermalMatrix, H_Thermal2Gripper)) return 0;
    //�Ƿ��ع�����
    thermalF = atoi(vConfigInfo[2].substr(0, 1).c_str());
    // ��ȡ�ع�ģʽ
    reconMode = atoi(vConfigInfo[3].substr(0, 1).c_str());
    if (thermalF || reconMode) {
        // ��ȡRGBD��е������
        ReadTxtFile(root + "thermalpoint.txt", robotPositionThermal);
        if (robotPositionThermal.size() == 0) {
            cout << "-----------�޺�������-------------" << endl;
            return 0;
        }
    }
    // ��ȡȨ����Ϣ
    rgbdWeight = atof(vConfigInfo[4].substr(0, vConfigInfo[4].find(',')).c_str());
    //cout << "ת�����ڲ����꣺" << CameraMatrix << endl;
    //cout << "ת�������۱궨��ϵ��" << H_Camera2Gripper << endl;
    //return 0;

    // ��ȡ�����ļ����Ƿ����ƶ���е�ۣ�������������ֱ���ع�
    if (vConfigInfo[0].substr(0, 2) != "-1") {
        // RGBD��ʼ��
        if (vConfigInfo[1].substr(0, 1) == "1") {
            
            if (astraCameraD2C->CameraInit(HARDWARE_D2C) != CAMERA_STATUS_SUCCESS) {
                cout << "rgbd camera init failed!" << endl;
                system("pause");
                return 0;
            }
        }
        // ��е�۴��ڳ�ʼ��
        char portChar[6];
        sprintf_s(portChar, "COM%d", vConfigInfo[0][0] - '0');
        if (!sp.Serial_open(portChar, 115200)) {
            cout << "serial port init error!" << endl;
            system("pause");
            return 0;
        }
        system("cls");

        // ���ƻ�е�۸���point.txt�˶��ɼ�����������
        for (int i = 0; i < robotPositionCamera.size(); ++i) {
            cout << robotPositionCamera[i] << endl;
            MoveToOnePoint(robotPositionCamera[i], &sp);
            Sleep(100);
            //string a;
            //cout << "���������ַ�����ͼƬ��";
            //cin >> a;
            if (astraCameraD2C->GetStreamData(colorImage, depthImage) == CAMERA_STATUS_SUCCESS) {
                SaveImg(root, i);
            }
        }
         //���ƻ�е�۸���point.txt�˶��ɼ�robotPositionThermal����
        if (vConfigInfo[2].substr(0, 1) == "1") {
            for (int i = 0; i < robotPositionThermal.size(); ++i) {
                bool excited = false;
                string robotPositionThermal_ex;
                if ((robotPositionThermal[i].size() == 31) && (robotPositionThermal[i][30] == 'E')) {
                    excited = true;
                    robotPositionThermal_ex = robotPositionThermal[i].substr(0, 30);
                }
                else {
                    robotPositionThermal_ex = robotPositionThermal[i];
                }
              
                cout << robotPositionThermal_ex << endl;
                MoveToOnePoint(robotPositionThermal_ex, &sp);
                Sleep(100);

                if (excited) {
                    string str = "BLK\r\n";
                    sp.Serial_write_string(str, 5);
                    Sleep(100);
                    string str1 = "BLG\r\n";
                    sp.Serial_write_string(str1, 5);
                    Sleep(25000);//20s
                    excited = false;
                    //cout << "123213" << endl;
                }
               

                //string a;
                //cout << "���������ַ�����ͼƬ��";
                //cin >> a;
            }
        }
    }

    //�ع�
    getthmeral();
    cout << "������Ϣ��" << reconMode << "   " << rgbdWeight << endl;
    if (robotPositionCamera.size() != 0) {
        ToolPoseCamera = cv::Mat(robotPositionCamera.size(), 6, CV_64F, 0.0);
        str2Mat(robotPositionCamera, ToolPoseCamera);
    }

    if (robotPositionThermal.size() != 0) {
        vector<string> temp;
        for (int i = 0; i < robotPositionThermal.size(); i++) {
            if ((robotPositionThermal[i].size() == 31) && (robotPositionThermal[i][30] == 'E')) {
                temp.push_back(robotPositionThermal[i].substr(0,30));
            }
        }
        robotPositionThermal.clear();
        robotPositionThermal = vector<string>(temp);
        ToolPoseThermal = cv::Mat(robotPositionThermal.size(), 6, CV_64F, 0.0);
        str2Mat(robotPositionThermal, ToolPoseThermal);
    }

    //��е��λ��Ϊŷ����-��ת����(camera)
    for (int i = 0; i < robotPositionCamera.size(); i++) {
        string depthImagePath = root + "depth/000" + to_string(i) + ".png";
        cv::Mat srcDepth = cv::imread(depthImagePath, cv::IMREAD_ANYDEPTH);
        //cv::flip(srcDepth, srcDepth, -1);
        depthImageVec.push_back(srcDepth);

        string colorImagePath = root + "rgb/000" + to_string(i) + ".jpg";
        cv::Mat srcColor = cv::imread(colorImagePath);
        //cv::flip(srcColor, srcColor, -1);
        colorImageVec.push_back(srcColor);
        //cv::imshow("src", srcColor);
        //cv::waitKey(0);

        // ��е�۶�ȡ��������
        cv::Mat H_Gripper2Base_camera = attitudeVectorToMatrix(ToolPoseCamera.row(i), false, "xyz");
        // ������->��������ת��=ͼ������->�������->ץ��->����
        // ץ��->���磺H_Gripper2Base_camera����е�۶�ȡ��
        // �������->ץ�֣�H_Camera2Gripper���궨��ã�
        // ͼ������->������꣺�ڲΣ��궨��ã�
        cv::Mat cameraPosition = H_Gripper2Base_camera * H_Camera2Gripper * (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);

        cameraPositionVec.push_back(Eigen::Vector3d(cameraPosition.at<double>(0, 0), cameraPosition.at<double>(1, 0), cameraPosition.at<double>(2, 0)));
        H_Gripper2Base_cameraVec.push_back(H_Gripper2Base_camera);
        H_Base2Gripper_cameraVec.push_back(H_Gripper2Base_camera.inv());
    }



    //��е��λ��Ϊŷ����-��ת����(thermal)
    for (int i = 0; i < robotPositionThermal.size(); i++) {
        cv::Mat H_Gripper2Base_thermal = attitudeVectorToMatrix(ToolPoseThermal.row(i), false, "xyz");
        cv::Mat thermalPosition = H_Gripper2Base_thermal * H_Thermal2Gripper * (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
        cv::Mat H_Base2Gripper_thermal = H_Gripper2Base_thermal.inv();

        thermalPositionVec.push_back(Eigen::Vector3d(thermalPosition.at<double>(0, 0), thermalPosition.at<double>(1, 0), thermalPosition.at<double>(2, 0)));
        H_Base2Gripper_thermalVec.push_back(H_Base2Gripper_thermal);

        string thermalPath = root + "thermal/000" + to_string(i) + ".jpg";
        cv::Mat thermalImage = cv::imread(thermalPath);
        //cv::flip(thermalImage, thermalImage, -1); //WTL, PCA���������������������辵��δ�����ԭʼ������Ҫ����
        thermalImageVec.push_back(thermalImage);
    }

    vector<std::shared_ptr<open3d::geometry::PointCloud>> vPcl(robotPositionCamera.size(), std::make_shared<open3d::geometry::PointCloud>());
    // ****************************************** ��ȫ���ֵ��� ******************************************	
    for (int i = 0; i < depthImageVec.size(); i++) {
        // ��ȡ��ǰ�����ͼ��
        cv::Mat depthImage = depthImageVec[i];
        // ƴ�����ͼΪ��ά�������ݣ�ƥ���������ͼ����ά����
        for (int row = 0; row < depthImage.rows; row++) {
            for (int col = 0; col < depthImage.cols; col++) {
                if (row > 0 && depthImage.at<ushort>(row, col) == 0) {
                    if (depthImage.at<ushort>(row - 1, col) > 0) {
                        int row_ = row;
                        while (row_ < depthImage.rows && depthImage.at<ushort>(row_, col) == 0) {
                            row_++;
                        }
                        if (row_ - row < 100 && row_ < depthImage.rows) {
                            depthImage.at<ushort>(row, col) = (depthImage.at<ushort>(row_, col) - depthImage.at<ushort>(row - 1, col)) / (double)(row_ - row + 1) + depthImage.at<ushort>(row - 1, col);
                        }
                    }
                }
            }
        }
    }

    for (int i = 0; i < robotPositionCamera.size(); ++i) {
        //��ȡ��ǰ�����ͼ��
        cv::Mat depthImage = depthImageVec[i];
        //ƴ�����ͼΪ��ά�������ݣ�ƥ���������ͼ����ά����
        for (int row = 0; row < depthImage.rows; row++) {
            for (int col = 0; col < depthImage.cols; col++) {
                Eigen::Vector3d colorTemp = { 0, 0, 0 };
                ushort depthNum = depthImage.at<ushort>(row, col); // ��������ǰ���ص����ֵ
                //�����������µ�����
                double x_camera, y_camera, z_camera;
                //astraCameraD2C->convertDepthToWorld(col, row, depthNum, x_camera, y_camera, z_camera);
                convertDepthTocamera(col, row, depthNum, x_camera, y_camera, z_camera);
                cv::Mat p_camera{ (double)x_camera, (double)y_camera, (double)z_camera, 1.0 };
                //�����е���µ�������Ϣ
                cv::Mat p_gripper = H_Camera2Gripper * p_camera;
                //��������µ�������Ϣ
                cv::Mat p_base = H_Gripper2Base_cameraVec[i] * p_gripper;
                //�����������㵽��������ļ��γ���
                Eigen::Vector3d pointTemp{ p_base.at<double>(0), p_base.at<double>(1), p_base.at<double>(2) };
                //cout << "p_base:" << p_base << endl;
                //if (pointTemp[2] < 100 || pointTemp[2] > 700) continue; // �޷�
                //if (pointTemp[0] < 800 || pointTemp[0] > 1200) continue; // �޷�

                Eigen::Vector3d colorThermal(0.0, 0.0, 0.0);
                Eigen::Vector3d colorCamera(0.0, 0.0, 0.0);

                colorCamera = Eigen::Vector3d(colorImageVec[i].at<cv::Vec3b>(row, col)[0] / 255.,
                    colorImageVec[i].at<cv::Vec3b>(row, col)[1] / 255.,
                    colorImageVec[i].at<cv::Vec3b>(row, col)[2] / 255.
                );
                if(reconMode==1)
                    GetColorFromImageVec(colorThermal, p_base, "thermal");
                GetColorFromImageVec(colorCamera, p_base, "camera");

                colorTemp = rgbdWeight * colorCamera + (1 - rgbdWeight) * colorThermal;
                //cout << "colorTemp:"<< colorTemp << endl;
                pcl_ptr->points_.emplace_back(pointTemp);
                pcl_ptr->colors_.emplace_back(colorTemp);
            }
        }
    }
    open3d::io::WritePointCloudToPLY(root + "3D_MODE.ply", *pcl_ptr);
    open3d::visualization::DrawGeometries({ pcl_ptr }, "pcl");

    system("pause");
	return 0;
}





void GetColorFromImageVec(Eigen::Vector3d& colorTmp, const cv::Mat& p_base, string thermalOrCamera) {
    colorTmp << 0.0, 0.0, 0.0;
    Eigen::Vector3d pointTemp(p_base.at<double>(0), p_base.at<double>(1), p_base.at<double>(2));
    vector<Image> imageVec;
    if (thermalOrCamera == "thermal") {
        for (int j = 0; j < thermalPositionVec.size(); j++) {
            int distanceTmp = (thermalPositionVec[j] - pointTemp).norm();
            //�����������µ�������Ϣ
            cv::Mat p_thermal = H_Thermal2Gripper.inv() * H_Base2Gripper_thermalVec[j] * p_base;
            //���������Ƕ�Ӧ����������
            p_thermal = ThermalMatrix * p_thermal;

            int u = p_thermal.at<double>(0, 0) / p_thermal.at<double>(2, 0);
            int v = p_thermal.at<double>(1, 0) / p_thermal.at<double>(2, 0);
            if (u > 0 && u < 640 && v > 0 && v < 480) {
                imageVec.push_back(Image(j, u, v, distanceTmp));
            }
        }
    }
    else if (thermalOrCamera == "camera") {
        for (int j = 0; j < cameraPositionVec.size(); j++) {
            int distanceTmp = (cameraPositionVec[j] - pointTemp).norm();
            //�����������µ�������Ϣ
            cv::Mat p_camera = H_Camera2Gripper.inv() * H_Base2Gripper_cameraVec[j] * p_base;
            //���������Ƕ�Ӧ����������
            p_camera = CameraMatrix * p_camera;

            int u = p_camera.at<double>(0, 0) / p_camera.at<double>(2, 0);
            int v = p_camera.at<double>(1, 0) / p_camera.at<double>(2, 0);
            if (u > 0 && u < 640 && v > 0 && v < 480) {
                imageVec.push_back(Image(j, u, v, distanceTmp));
            }
        }
    }

    //�鿴�õ����Ƿ���ƥ�������������ͼ
    if (imageVec.empty()) {
        //���û��ƥ�������ͼ��������Ϊ��ɫ
        colorTmp << 1., 1., 1.;
    }
    else {
        //�����ƥ�������ͼ��������ͼ��Ȩ�ں�
        sort(imageVec.begin(), imageVec.end(), [=](Image img1, Image img2) {
            return img1.distance < img2.distance; });

        int differencValue = 30;
        while (imageVec.back().distance - imageVec.front().distance > differencValue) {
            imageVec.pop_back();
        }
        int distanceSum = differencValue;
        for (auto image_ : imageVec) {
            distanceSum += image_.distance;
        }
        int meanDistance = distanceSum / imageVec.size();

        for (auto image_ : imageVec) {
            double weight = (meanDistance - image_.distance) / (double)differencValue;
            cv::Mat img;
            if (thermalOrCamera == "thermal") {
                img = thermalImageVec[image_.imageIndex];
            }
            else if (thermalOrCamera == "camera") {
                img = colorImageVec[image_.imageIndex];
            }
            int u = image_.u;
            int v = image_.v;
            Eigen::Vector3d colorTemp_(img.at<cv::Vec3b>(v, u)[2] / 255.0, img.at<cv::Vec3b>(v, u)[1] / 255.0, img.at<cv::Vec3b>(v, u)[0] / 255.0);

            colorTmp += colorTemp_ * weight;
        }
    }
}