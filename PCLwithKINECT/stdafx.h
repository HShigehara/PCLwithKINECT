// stdafx.h : 標準のシステム インクルード ファイルのインクルード ファイル、または
// 参照回数が多く、かつあまり変更されない、プロジェクト専用のインクルード ファイル
// を記述します。
//

#pragma once

#define _CRT_SECURE_NO_WARNINGS //fopen等昔の関数の警告を非表示にする

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>



// TODO: プログラムに必要な追加ヘッダーをここで参照してください。
//Windows関係
#include <Windows.h>
#include <NuiApi.h>

//OpenCV関係
#include <opencv2\opencv.hpp>
#include <opencv2\flann\flann.hpp>

//PCL関係
#include <pcl\point_types.h>
#include <pcl\point_cloud.h>
#include <pcl\io\io.h>
#include <pcl\io\pcd_io.h> //.pcd出力用
#include <pcl\visualization\cloud_viewer.h>
#include <pcl\filters\statistical_outlier_removal.h> //外れ値フィルター用
#include <pcl\kdtree\kdtree_flann.h> //スムージング用
#include <pcl\surface\mls.h> //スムージング用
#include <pcl\filters\voxel_grid.h> //ダウンサンプリング用
#include <pcl\PCLPointField.h>

//Kinect関係
#define ERROR_CHECK(ret)                                            \
  if(ret != S_OK) {                                               \
  std::stringstream ss;                                           \
  ss << "failed " #ret " " << std::hex << ret << std::endl;       \
  throw std::runtime_error(ss.str().c_str());                     \
      }

const NUI_IMAGE_RESOLUTION CAMERA_RESOLUTION = NUI_IMAGE_RESOLUTION_640x480;