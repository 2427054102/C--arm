#include "header.h"
#include "RobotArmControl.h"





////注意
// 标定需要修改坐标（使用红外还是深度相机进行标定）  不在这个工程
// 
// 185行，修改红外拍照个数
// 




// 相机标定参数
cv::Mat_<double> ThermalMatrix;
cv::Mat_<double> H_Thermal2Gripper;
cv::Mat_<double> CameraMatrix;
cv::Mat_<double> H_Camera2Gripper;

//创建点云指针cv::Mat
std::shared_ptr<geometry::PointCloud> pcl_ptr = std::make_shared<geometry::PointCloud>();

//计算机械臂位姿
cv::Mat ToolPoseCamera;
cv::Mat ToolPoseThermal;

//定义变换矩阵
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

//机械臂采集数据时的姿态
vector<string> robotPositionCamera;
vector<string> robotPositionThermal;
vector<string> vConfigInfo;
AstraCameraD2C* astraCameraD2C;
unordered_map<int, string> mCam2Name;
cv::Mat colorImage(IMAGE_HEIGHT_480, IMAGE_WIDTH_640, CV_8UC3);
cv::Mat depthImage(IMAGE_HEIGHT_480, IMAGE_WIDTH_640, CV_16UC1);//640x480
double rgbdWeight = 0.; // 深度相机重构权重
int reconMode = 0.; // 重构模式：0-只重构三维模型，1-重构三维模型以及红外映射
int thermalF = 0;//是否需要控制机械臂采集红外，1：需要，0：不需要

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
    cout << "深度图路径为：" + colorSavePath << endl;
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
                // 保存每行最后一位数据
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
    // 有效性验证
    if (vPosStr.size() != 7) return false;
    // 坐标转换
    auto vvPosDouble = ConvertString2Point(vPosStr);
    // 有效性验证
    for (int i = 0; i < 7; ++i) {
        if (i < 3) {
            if (vvPosDouble[i].size() != 3) return false;
        }
        else {
            if (vvPosDouble[i].size() != 4) return false;
        }
    }
#if 0
    cout << "解析的坐标信息：";
    for (int i = 0; i < vvPosDouble.size(); ++i) {
        for (int j = 0; j < vvPosDouble[i].size(); ++j) {
            cout << vvPosDouble[i][j] << "  ";
        }
        cout << endl;
    }
#endif
    // 数据写入
    for (int i = 0; i < vvPosDouble.size(); ++i) {
        if (i < 3) {
            for (int j = 0; j < vvPosDouble[i].size(); ++j) {
                intrinsic.at<double>(i, j) = vvPosDouble[i][j];
            }
            //intrinsic.at<double>(i, 3) = 0.; // 最后以为是0
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
/// 共享文件
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
    out_name = GetThermal_Dir(dir_name, 2);//修改个数
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
#pragma comment(lib, "comsuppw.lib") // 解决char* 转 wchar_t报错
#pragma comment(lib, "Ws2_32.lib") // 无法解析inet_addr

    astraCameraD2C = new AstraCameraD2C();
    mCam2Name[0] = "rgb/";
    mCam2Name[1] = "thermal/";
    mCam2Name[2] = "hdrgb/";
    string root = "./datas/"; // 标定图片路径
    // 读取配置文件
    ReadTxtFile(root + "config.txt", vConfigInfo);
    if (vConfigInfo.size() == 0) return 0;
    // 读取RGBD机械臂坐标
    ReadTxtFile(root + "rgbdpoint.txt", robotPositionCamera);
    if (robotPositionCamera.size() == 0) return 0;
    // 读取RGBD相机内参、手眼标定转换关系
    vector<string> vPosInfo;
    ReadTxtFile(root + "rgbdcal.txt", vPosInfo);
    CameraMatrix = cv::Mat::zeros(cv::Size(4, 3), CV_64F);
    H_Camera2Gripper = cv::Mat::zeros(cv::Size(4, 4), CV_64F);
    if (!AnalysisString2Mat(vPosInfo, CameraMatrix, H_Camera2Gripper)) return 0;
    // 读取热像仪相机内参、手眼标定转换关系
    vector<string> vPosInfo_Th;
    ReadTxtFile(root + "thermalcal.txt", vPosInfo_Th);
    ThermalMatrix = cv::Mat::zeros(cv::Size(4, 3), CV_64F);
    H_Thermal2Gripper = cv::Mat::zeros(cv::Size(4, 4), CV_64F);
    if (!AnalysisString2Mat(vPosInfo_Th, ThermalMatrix, H_Thermal2Gripper)) return 0;
    //是否重构红外
    thermalF = atoi(vConfigInfo[2].substr(0, 1).c_str());
    // 读取重构模式
    reconMode = atoi(vConfigInfo[3].substr(0, 1).c_str());
    if (thermalF || reconMode) {
        // 读取RGBD机械臂坐标
        ReadTxtFile(root + "thermalpoint.txt", robotPositionThermal);
        if (robotPositionThermal.size() == 0) {
            cout << "-----------无红外坐标-------------" << endl;
            return 0;
        }
    }
    // 读取权重信息
    rgbdWeight = atof(vConfigInfo[4].substr(0, vConfigInfo[4].find(',')).c_str());
    //cout << "转换后内参坐标：" << CameraMatrix << endl;
    //cout << "转换后手眼标定关系：" << H_Camera2Gripper << endl;
    //return 0;

    // 读取配置文件，是否需移动机械臂，根据现有数据直接重构
    if (vConfigInfo[0].substr(0, 2) != "-1") {
        // RGBD初始化
        if (vConfigInfo[1].substr(0, 1) == "1") {
            
            if (astraCameraD2C->CameraInit(HARDWARE_D2C) != CAMERA_STATUS_SUCCESS) {
                cout << "rgbd camera init failed!" << endl;
                system("pause");
                return 0;
            }
        }
        // 机械臂串口初始化
        char portChar[6];
        sprintf_s(portChar, "COM%d", vConfigInfo[0][0] - '0');
        if (!sp.Serial_open(portChar, 115200)) {
            cout << "serial port init error!" << endl;
            system("pause");
            return 0;
        }
        system("cls");

        // 控制机械臂根据point.txt运动采集深度相机数据
        for (int i = 0; i < robotPositionCamera.size(); ++i) {
            cout << robotPositionCamera[i] << endl;
            MoveToOnePoint(robotPositionCamera[i], &sp);
            Sleep(100);
            //string a;
            //cout << "输入任意字符保存图片：";
            //cin >> a;
            if (astraCameraD2C->GetStreamData(colorImage, depthImage) == CAMERA_STATUS_SUCCESS) {
                SaveImg(root, i);
            }
        }
         //控制机械臂根据point.txt运动采集robotPositionThermal数据
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
                //cout << "输入任意字符保存图片：";
                //cin >> a;
            }
        }
    }

    //重构
    getthmeral();
    cout << "配置信息：" << reconMode << "   " << rgbdWeight << endl;
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

    //机械臂位姿为欧拉角-旋转矩阵(camera)
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

        // 机械臂读取到的坐标
        cv::Mat H_Gripper2Base_camera = attitudeVectorToMatrix(ToolPoseCamera.row(i), false, "xyz");
        // 深度相机->世界坐标转换=图像坐标->相机坐标->抓手->世界
        // 抓手->世界：H_Gripper2Base_camera（机械臂读取）
        // 相机坐标->抓手：H_Camera2Gripper（标定求得）
        // 图像坐标->相机坐标：内参（标定求得）
        cv::Mat cameraPosition = H_Gripper2Base_camera * H_Camera2Gripper * (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);

        cameraPositionVec.push_back(Eigen::Vector3d(cameraPosition.at<double>(0, 0), cameraPosition.at<double>(1, 0), cameraPosition.at<double>(2, 0)));
        H_Gripper2Base_cameraVec.push_back(H_Gripper2Base_camera);
        H_Base2Gripper_cameraVec.push_back(H_Gripper2Base_camera.inv());
    }



    //机械臂位姿为欧拉角-旋转矩阵(thermal)
    for (int i = 0; i < robotPositionThermal.size(); i++) {
        cv::Mat H_Gripper2Base_thermal = attitudeVectorToMatrix(ToolPoseThermal.row(i), false, "xyz");
        cv::Mat thermalPosition = H_Gripper2Base_thermal * H_Thermal2Gripper * (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
        cv::Mat H_Base2Gripper_thermal = H_Gripper2Base_thermal.inv();

        thermalPositionVec.push_back(Eigen::Vector3d(thermalPosition.at<double>(0, 0), thermalPosition.at<double>(1, 0), thermalPosition.at<double>(2, 0)));
        H_Base2Gripper_thermalVec.push_back(H_Base2Gripper_thermal);

        string thermalPath = root + "thermal/000" + to_string(i) + ".jpg";
        cv::Mat thermalImage = cv::imread(thermalPath);
        //cv::flip(thermalImage, thermalImage, -1); //WTL, PCA处理后的数据是正常的无需镜像，未处理的原始数据需要镜像
        thermalImageVec.push_back(thermalImage);
    }

    vector<std::shared_ptr<open3d::geometry::PointCloud>> vPcl(robotPositionCamera.size(), std::make_shared<open3d::geometry::PointCloud>());
    // ****************************************** 补全部分点云 ******************************************	
    for (int i = 0; i < depthImageVec.size(); i++) {
        // 获取当前的深度图像
        cv::Mat depthImage = depthImageVec[i];
        // 拼接深度图为三维点云数据，匹配红外特征图到三维点云
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
        //获取当前的深度图像
        cv::Mat depthImage = depthImageVec[i];
        //拼接深度图为三维点云数据，匹配红外特征图到三维点云
        for (int row = 0; row < depthImage.rows; row++) {
            for (int col = 0; col < depthImage.cols; col++) {
                Eigen::Vector3d colorTemp = { 0, 0, 0 };
                ushort depthNum = depthImage.at<ushort>(row, col); // 深度相机当前像素的深度值
                //计算深度相机下的坐标
                double x_camera, y_camera, z_camera;
                //astraCameraD2C->convertDepthToWorld(col, row, depthNum, x_camera, y_camera, z_camera);
                convertDepthTocamera(col, row, depthNum, x_camera, y_camera, z_camera);
                cv::Mat p_camera{ (double)x_camera, (double)y_camera, (double)z_camera, 1.0 };
                //计算机械手下的坐标信息
                cv::Mat p_gripper = H_Camera2Gripper * p_camera;
                //计算基地下的坐标信息
                cv::Mat p_base = H_Gripper2Base_cameraVec[i] * p_gripper;
                //计算点云坐标点到两个相机的几何长度
                Eigen::Vector3d pointTemp{ p_base.at<double>(0), p_base.at<double>(1), p_base.at<double>(2) };
                //cout << "p_base:" << p_base << endl;
                //if (pointTemp[2] < 100 || pointTemp[2] > 700) continue; // 限幅
                //if (pointTemp[0] < 800 || pointTemp[0] > 1200) continue; // 限幅

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
            //计算热像仪下的坐标信息
            cv::Mat p_thermal = H_Thermal2Gripper.inv() * H_Base2Gripper_thermalVec[j] * p_base;
            //计算热像仪对应的像素坐标
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
            //计算热像仪下的坐标信息
            cv::Mat p_camera = H_Camera2Gripper.inv() * H_Base2Gripper_cameraVec[j] * p_base;
            //计算热像仪对应的像素坐标
            p_camera = CameraMatrix * p_camera;

            int u = p_camera.at<double>(0, 0) / p_camera.at<double>(2, 0);
            int v = p_camera.at<double>(1, 0) / p_camera.at<double>(2, 0);
            if (u > 0 && u < 640 && v > 0 && v < 480) {
                imageVec.push_back(Image(j, u, v, distanceTmp));
            }
        }
    }

    //查看该点云是否有匹配的热像仪特征图
    if (imageVec.empty()) {
        //如果没有匹配的特征图，将其标记为白色
        colorTmp << 1., 1., 1.;
    }
    else {
        //如果有匹配的特征图，将特张图加权融合
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