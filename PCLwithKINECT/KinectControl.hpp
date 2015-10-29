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

	//クラウドビューワー用
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	pcl::visualization::CloudViewer *viewer;

	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
	//pcl::visualization::CloudViewer *viewer;


	//外れ値処理後用
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cf;
	//pcl::visualization::CloudViewer *viewer_filtered;


	//スムージングの結果を表示
	//pcl::PointCloud<pcl::PointNormal> mlsp; //出力する点群の保存場所
	//pcl::PointCloud<pcl::PointXYZ>::Ptr mlsp; //出力する点群の保存場所
	//pcl::visualization::CloudViewer *smooth;

	//pcl::PointCloud<pcl::PointXYZ>::Ptr sample;
	//pcl::visualization::CloudViewer *csample;
};