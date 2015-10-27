// stdafx.h : �W���̃V�X�e�� �C���N���[�h �t�@�C���̃C���N���[�h �t�@�C���A�܂���
// �Q�Ɖ񐔂������A�����܂�ύX����Ȃ��A�v���W�F�N�g��p�̃C���N���[�h �t�@�C��
// ���L�q���܂��B
//

#pragma once

#define _CRT_SECURE_NO_WARNINGS //fopen���̂̊֐��̌x�����\���ɂ���

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>



// TODO: �v���O�����ɕK�v�Ȓǉ��w�b�_�[�������ŎQ�Ƃ��Ă��������B
//Windows�֌W
#include <Windows.h>
#include <NuiApi.h>

//OpenCV�֌W
#include <opencv2\opencv.hpp>
#include <opencv2\flann\flann.hpp>

//PCL�֌W
#include <pcl\point_types.h>
#include <pcl\point_cloud.h>
#include <pcl\io\io.h>
#include <pcl\io\pcd_io.h> //.pcd�o�͗p
#include <pcl\visualization\cloud_viewer.h>
#include <pcl\filters\statistical_outlier_removal.h> //�O��l�t�B���^�[�p
#include <pcl\kdtree\kdtree_flann.h> //�X���[�W���O�p
#include <pcl\surface\mls.h> //�X���[�W���O�p
#include <pcl\filters\voxel_grid.h> //�_�E���T���v�����O�p
#include <pcl\PCLPointField.h>

//Kinect�֌W
#define ERROR_CHECK(ret)                                            \
  if(ret != S_OK) {                                               \
  std::stringstream ss;                                           \
  ss << "failed " #ret " " << std::hex << ret << std::endl;       \
  throw std::runtime_error(ss.str().c_str());                     \
      }

const NUI_IMAGE_RESOLUTION CAMERA_RESOLUTION = NUI_IMAGE_RESOLUTION_640x480;