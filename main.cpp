#include "EachStageProcess.h"
#include "YaskawaRobotArm.h"
#include "RokaeRobotArm.h"
#include "GlobalVariable.h"
#include "MindVision.h"
#include "astra/AstraCameraD2C.h"

int main() {
#pragma comment(lib, "comsuppw.lib") // 解决char* 转 wchar_t报错
#pragma comment(lib, "Ws2_32.lib") // 无法解析inet_addr


#if ROBOT_ARM
    RokaeRobotArm* robotArm = new RokaeRobotArm();
    robotArmHandle = BaseRobotArm::GetSingleInstance(robotArm);/////////
    robotArmHandle->InitRobot();

#endif // robot reset

#if ROBOT_ARM_HS
#endif // robot reset

#if RGBD_CAMERA
    // 初始化深度相机
    astraCameraD2C = new AstraCameraD2C();
    if (astraCameraD2C->CameraInit(HARDWARE_D2C) != CAMERA_STATUS_SUCCESS) {
        printf("camera init failed\n");
        return 0;
    }
#endif // RGBD_CAMERA

#if HDRGB_CAMERA
    if (!init_mindvision()) {
        printf("mindvision camera init fail! \n");
        return 0;
    }
#endif // HD RGB mindvision


#if COLLECT_STAGE == 0
#if INTERACTION
    Eigen::Vector3d goalPos;
    double goalLength, goalWidth = 0.;
    InteractObtainGoalInfo(goalPos, goalLength, goalWidth);
    //计算目标向量的范数,即目标的距离
    cout << "distance cal:" << goalPos.norm() << endl;
    if (goalPos.norm() == 0) return 0;
#endif // interaction
    Stage1Process(goalPos);
#elif COLLECT_STAGE == 1
    Stage2Process();
#elif COLLECT_STAGE == 2
    Stage3Process();
#endif
    return 0;
}