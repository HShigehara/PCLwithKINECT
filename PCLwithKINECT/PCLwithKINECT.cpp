// PCLwithKINECT.cpp : �R���\�[�� �A�v���P�[�V�����̃G���g�� �|�C���g���`���܂��B
//

#include "stdafx.h"
#include "KinectControl.hpp"

int _tmain(int argc, _TCHAR* argv[])
{
	try {
		KinectControl kinect;
		kinect.initialize();
		kinect.run();
	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}
	return 0;
}