#pragma once
#include "stdafx.h"

class KinectControl
{
public:
	KinectControl(void);
	~KinectControl(void);

	void initialize();
	void run();

private:
	INuiSensor *kinect;
	HANDLE imageStreamHandle;
	HANDLE depthStreamHandle;
	HANDLE streamEvent;

	DWORD width;
	DWORD height;

	void createInstance();
	void setRgbImage(cv::Mat &image);
	void setDepthImage(cv::Mat &image);

	cv::Mat rgbImage;
	cv::Mat depthImage;

	//�N���E�h�r���[���[�p
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	pcl::visualization::CloudViewer *viewer;

	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
	//pcl::visualization::CloudViewer *viewer;


	//�O��l������p
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cf;
	//pcl::visualization::CloudViewer *viewer_filtered;


	//�X���[�W���O�̌��ʂ�\��
	//pcl::PointCloud<pcl::PointNormal> mlsp; //�o�͂���_�Q�̕ۑ��ꏊ
	//pcl::PointCloud<pcl::PointXYZ>::Ptr mlsp; //�o�͂���_�Q�̕ۑ��ꏊ
	//pcl::visualization::CloudViewer *smooth;

	//pcl::PointCloud<pcl::PointXYZ>::Ptr sample;
	//pcl::visualization::CloudViewer *csample;
};