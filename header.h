#pragma once

#include <iostream>
using namespace std;

#include <string>
#include <memory>
#include <io.h>
#include <fstream>
#include <vector>

//astra
#include "astra/AstraCamera.h"
#include "astra/AstraCameraD2C.h"
#include "astra/d2cSwapper.h"
#include "astra/FrameListener.h"
//Thermal
#include "MagDevice.h"
#include "MagService.h"
//#include "./Include/CameraDefine.h"
//#include "./Include/CameraApi.h"
//opencv
#include <opencv2/opencv.hpp>

//open3d
#include <Open3D/Open3D.h>
using namespace open3d;

//多线程
#include <process.h>

//创建文件夹
#include <direct.h>

#include "DirFileNum.h"
#include "IOFile.h"
#include "Shlobj.h" //文件夹

#include <comutil.h> //char* ת wchar_t 
// 相机手眼标定
#include "CalibrateEyeInHand.h"