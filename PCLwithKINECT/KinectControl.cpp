#include "stdafx.h"
#include "KinectControl.hpp"

KinectControl::KinectControl()
{
}

KinectControl::~KinectControl()
{
	if (kinect != 0) {
		kinect->NuiShutdown();
		kinect->Release();
	}
}

void KinectControl::initialize()
{
	createInstance();

	// Kinect�̐ݒ������������
	ERROR_CHECK(kinect->NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX | NUI_INITIALIZE_FLAG_USES_SKELETON));

	// RGB�J����������������
	ERROR_CHECK(kinect->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, CAMERA_RESOLUTION, 0, 2, 0, &imageStreamHandle));

	// �����J����������������
	ERROR_CHECK(kinect->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, CAMERA_RESOLUTION, 0, 2, 0, &depthStreamHandle));

	// �t���[���X�V�C�x���g�̃n���h�����쐬����
	streamEvent = ::CreateEvent(0, TRUE, FALSE, 0);
	ERROR_CHECK(kinect->NuiSetFrameEndEvent(streamEvent, 0));

	// �w�肵���𑜓x�́A��ʃT�C�Y���擾����
	::NuiImageResolutionToSize(CAMERA_RESOLUTION, width, height);

	// PointCloud�r���[����������
	viewer = new pcl::visualization::CloudViewer("Kinect Point Cloud");
	//viewer_filtered = new pcl::visualization::CloudViewer("Kinect Point Cloud(filtered)");
	//smooth = new pcl::visualization::CloudViewer("Moving Least Square"); //�X���[�W���O
	//csample = new pcl::visualization::CloudViewer("SAMPLE");
}

void KinectControl::createInstance()
{
	// �ڑ�����Ă���Kinect�̐����擾����
	int count = 0;
	ERROR_CHECK(::NuiGetSensorCount(&count));
	if (count == 0) {
		throw std::runtime_error("Kinect ��ڑ����Ă�������");
	}

	// �ŏ���Kinect�̃C���X�^���X���쐬����
	ERROR_CHECK(::NuiCreateSensorByIndex(0, &kinect));

	// Kinect�̏�Ԃ��擾����
	HRESULT status = kinect->NuiStatus();
	if (status != S_OK) {
		throw std::runtime_error("Kinect �����p�\�ł͂���܂���");
	}
}

void KinectControl::run()
{
	// ���C�����[�v
	while (1) {
		// �f�[�^�̍X�V��҂�
		DWORD ret = ::WaitForSingleObject(streamEvent, INFINITE);
		::ResetEvent(streamEvent);

		setRgbImage(rgbImage);
		setDepthImage(depthImage);

		// �摜��\������
		//cv::imshow("RGBCamera", rgbImage);
		//cv::imshow("DepthCamera", depthImage);

		// �|�C���g�N���E�h��\������
		viewer->showCloud(cloud);
		//viewer_filtered->showCloud(cf);
		//smooth->showCloud(mlsp);
		//csample->showCloud(sample);

		// �I���̂��߂̃L�[���̓`�F�b�N���A�\���̂��߂̃E�F�C�g
		int key = cv::waitKey(10);
		if (key == 'q') {
			break;
		}
	}
}

void KinectControl::setRgbImage(cv::Mat& image)
{
	try {
		// RGB�J�����̃t���[���f�[�^���擾����
		NUI_IMAGE_FRAME imageFrame = { 0 };
		ERROR_CHECK(kinect->NuiImageStreamGetNextFrame(imageStreamHandle, 0, &imageFrame));

		// �摜�f�[�^���擾����
		NUI_LOCKED_RECT colorData;
		imageFrame.pFrameTexture->LockRect(0, &colorData, 0, 0);

		// �摜�f�[�^���R�s�[����
		image = cv::Mat(height, width, CV_8UC4, colorData.pBits);

		// �t���[���f�[�^���������
		ERROR_CHECK(kinect->NuiImageStreamReleaseFrame(imageStreamHandle, &imageFrame));
	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}
}

void KinectControl::setDepthImage(cv::Mat& image)
{
	try {
		// �|�C���g�N���E�h����
		//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr points(new pcl::PointCloud<pcl::PointXYZRGBA>()); //�J���[�܂ŗ��p��������΂�������g��
		pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>()); //�����̓_�Q�������ꍇ�͂�����
		points->width = width;
		points->height = height;


		// �����摜����
		image = cv::Mat(height, width, CV_8UC1, cv::Scalar(0));

		// �����J�����̃t���[���f�[�^���擾����
		NUI_IMAGE_FRAME depthFrame = { 0 };
		ERROR_CHECK(kinect->NuiImageStreamGetNextFrame(depthStreamHandle, 0, &depthFrame));

		// �����f�[�^���擾����
		NUI_LOCKED_RECT depthData = { 0 };
		depthFrame.pFrameTexture->LockRect(0, &depthData, 0, 0);

		USHORT* depth = (USHORT*)depthData.pBits;
		for (int i = 0; i < (depthData.size / sizeof(USHORT)); ++i) {
			USHORT distance = ::NuiDepthPixelToDepth(depth[i]);

			LONG depthX = i % width;
			LONG depthY = i / width;
			LONG colorX = depthX;
			LONG colorY = depthY;

			// �����J�����̍��W���ARGB�J�����̍��W�ɕϊ�����
			kinect->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(CAMERA_RESOLUTION, CAMERA_RESOLUTION, 0, depthX, depthY, 0, &colorX, &colorY);

			// �����摜�쐬
			image.at<UCHAR>(colorY, colorX) = distance / 8192.0 * 255.0;

			// �|�C���g�N���E�h
			Vector4 real = NuiTransformDepthImageToSkeleton(depthX, depthY, distance, CAMERA_RESOLUTION);
			//pcl::PointXYZRGBA point;
			pcl::PointXYZ point;
			point.x = real.x;
			point.y = -real.y;
			point.z = real.z;

			// �e�N�X�`��
			//cv::Vec4b color = rgbImage.at<cv::Vec4b>(colorY, colorX);
			//point.r = color[2];
			//point.g = color[1];
			//point.b = color[0];
			points->push_back(point);
		}
		// �|�C���g�N���E�h���R�s�[
		cloud = points;

		//�f�[�^��.pcd�ɕۑ�
		//pcl::io::savePCDFileBinary("sample.pcd", *cloud);
		//�ۑ������f�[�^��ǂݍ���ł݂�
		//pcl::PointCloud<pcl::PointXYZ>::Ptr read(new pcl::PointCloud<pcl::PointXYZ>);
		//pcl::io::loadPCDFile("sample.pcd", *read);
		//pcl::visualization::CloudViewer read_pcd("read point cloud");
		//read_pcd.showCloud(read);


		//�t�B���^�����O�I�u�W�F�N�g�̐���
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>()); //�t�B���^�[�p�ǉ���
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> fl; //�C���X�^���X�̐���
		fl.setInputCloud(cloud); //�t�B���^�����O�Ώۂ̓_�Q�����
		fl.setMeanK(10); //�t�B���^�����O�̒��x(?)
		fl.setStddevMulThresh(1.0);
		fl.filter(*cloud_filtered);
		cloud = cloud_filtered;

		//�{�N�Z���O���b�h�t�B���^�ɂ��_�E���T���v�����O
		//pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
		//pcl::VoxelGrid<pcl::PointXYZ> sor/*(new pcl::PointCloud<pcl::PointXYZ>)*/;
		//sor.setInputCloud(cloud);
		//sor.setLeafSize(0.003f, 0.003f, 0.003f);
		//sor.filter(*filtered);
		//cloud = filtered;


		//�X���[�W���O����
		//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>()); //KdTree�̍쐬
		//pcl::PointCloud<pcl::PointNormal> mls_points; //�o�͂���_�Q�̕ۑ��ꏊ
		////pcl::PointCloud<pcl::PointXYZ>::Ptr mls_points; //�o�͂���_�Q�̕ۑ��ꏊ
		////pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
		////pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointCloud<pcl::PointXYZ>::Ptr> mls;
		//pcl::MovingLeastSquares < pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointNormal>> mls;
		//mls.setComputeNormals(true); //�@���̌v�Z
		//�e�p�����[�^�̐ݒ�
		//mls.setInputCloud(cloud);
		//mls.setPolynomialFit(true);
		//mls.setSearchMethod(tree);
		//mls.setSearchRadius(0.03);
		//mls.process(mls_points); //�o��
		//mlsp = mls_points;

		//cloud = mls_points;

		//boost::shared_ptr < pcl::visualization::PCLVisualizer> viewer_mls(new pcl::visualization::PCLVisualizer("3D Viewer"));
		//viewer_mls->setBackgroundColor(0, 0, 0);
		//viewer_mls->addPointCloud<pcl::PointXYZ> (mls_points, "sample");
		//viewer_mls->getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample");
		
		//cloud = mls_points;

		// �t���[���f�[�^���������
		ERROR_CHECK(kinect->NuiImageStreamReleaseFrame(depthStreamHandle, &depthFrame));
	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}
}