#include "../include/recongnizer.h"

void ansToChar(double * in, char * out)
{
	char *inC = (char *)in;
	for (int i = 0; i < 20; i++)
	{
		out[i] = inC[i];
	}
}

void Preprocessing::setPrintMode(int modeI)
{
	printMode = modeI;
	if (modeI < PRTTP_NOTHING)
	{
		cvNamedWindow("Process Result", CV_WINDOW_AUTOSIZE);
	}
}

void Preprocessing::set_estimateMode(int estimate_mode)
{
	estimateMode = estimate_mode;
}

int Preprocessing::Otsu(cv::Mat &image)
{
	// To do
	return 0;

}

void Preprocessing::drawDetConts(cv::Mat & img, cv::vector<cv::vector<cv::Point>> contours, cv::vector<cv::Point2d> Centers, int contoursSize)
{
	int min = numLights;
	if (numLights > contoursSize)
		min = contoursSize;
	for (int i = 0; i < min; i++)
	{
		//填充光斑
		cv::drawContours(img, contours, (contoursSize - i - 1), 255, CV_FILLED);
		//绘制光斑重心点
		cv::circle(img, Centers[contoursSize - i - 1], 2, cv::Scalar(0, 0, 255), -1, 8);
	}
}

void Preprocessing::get_dockingCenter(cv::vector<cv::Point2d> Centers, int contoursSize)
{
	int centerTemp_x = 0;
	int centerTemp_y = 0;
	double sum_x = 0;
	double sum_y = 0;
	int min = numLights;

	if (numLights > contoursSize)
		min = contoursSize;
	for (int i = 0; i < min; i++)
	{
		sum_x += Centers[contoursSize - i - 1].x;
		sum_y += Centers[contoursSize - i - 1].y;
	}
	centerTemp_x = int(sum_x / min);
	centerTemp_y = int(sum_y / min);
	dockingCenter = cv::Point(centerTemp_x, centerTemp_y);
}

void Preprocessing::get_Angles(cv::Size imageSize, double pos[])
{
	int hw = imageSize.width / 2;
	int hh = imageSize.height / 2;
	//规定：目标在视线右边时 angle < 0;在左边时 angle > 0 (逆时针为正)
	float xAngle = ((hw - dockingCenter.x) / hw) * (horizontalFOV / 2);
	//规定：目标在视线上方时 angle > 0;在下方时 angle < 0
	float yAngle = ((hh - dockingCenter.y) / hh) * (verticalFOV / 2);
	pos[1] = xAngle;
	pos[2] = yAngle;
}

//针对特定的对接装置而言的匹配方法
void Preprocessing::get_targetPoints(cv::vector<cv::Point2d> lightCenters, cv::vector<cv::Point2d> & matchPoints)
{
	//确定0位置
	matchPoints[0] = lightCenters[0];
	//确定 3 4 位置
	if (lightCenters[4].x < lightCenters[5].x)
	{
		matchPoints[3] = lightCenters[4];
		matchPoints[4] = lightCenters[5];
	}
	else
	{
		matchPoints[3] = lightCenters[5];
		matchPoints[4] = lightCenters[4];
	}

	//比较分析得出1 2 5 的位置
	if (lightCenters[1].x < lightCenters[2].x)
	{
		if (lightCenters[1].x < lightCenters[3].x)
		{
			matchPoints[1] = lightCenters[1];
			if (lightCenters[2].x < lightCenters[3].x)
			{
				matchPoints[5] = lightCenters[2];
				matchPoints[2] = lightCenters[3];
			}
			else
			{
				matchPoints[5] = lightCenters[3];
				matchPoints[2] = lightCenters[2];
			}
		}
		else
		{
			matchPoints[1] = lightCenters[3];
			matchPoints[5] = lightCenters[1];
			matchPoints[2] = lightCenters[2];
		}
	}
	else
	{
		if (lightCenters[1].x > lightCenters[3].x)
		{
			matchPoints[2] = lightCenters[1];
			if (lightCenters[2].x < lightCenters[3].x)
			{
				matchPoints[1] = lightCenters[2];
				matchPoints[5] = lightCenters[3];
			}
			else
			{
				matchPoints[1] = lightCenters[3];
				matchPoints[5] = lightCenters[2];
			}
		}
		else
		{
			matchPoints[1] = lightCenters[2];
			matchPoints[5] = lightCenters[1];
			matchPoints[2] = lightCenters[3];
		}
	}
}

void Preprocessing::process(cv::Mat image, double pos[], int & ansType)
{
	clock_t start_time = clock();
	for (int i = 0; i < 5; i++)
		pos[i] = -20000;

	cv::Mat pSrcImg = image;
	cv::Mat pSrcImg_ = pSrcImg.clone();
	cv::Size imgSize = cv::Size(pSrcImg.cols, pSrcImg.rows);	//(宽， 高)

	//去畸变
	cv::Mat mapx = cv::Mat(imgSize, CV_32FC1);
	cv::Mat mapy = cv::Mat(imgSize, CV_32FC1);
	cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
	cv::Mat newCamMat = cv::getOptimalNewCameraMatrix(cameraMat, distCoeffs, imgSize, 1);
	initUndistortRectifyMap(cameraMat, distCoeffs, R, newCamMat, imgSize, CV_32FC1, mapx, mapy);
	cv::remap(pSrcImg_, pSrcImg, mapx, mapy, cv::INTER_LINEAR);

	//cv::undistort(pSrcImg_, pSrcImg, cameraMat, distCoeffs, getOptimalNewCameraMatrix(cameraMat, distCoeffs, imgSize, 1));
	if (printMode <= PRTTP_TESTMOD)
	{
		cv::namedWindow("Gray", CV_WINDOW_AUTOSIZE);
		cv::imshow("undist", pSrcImg);
	}
	int contours_size = 0;
	
	//缩小图像，降低时耗
	cv::resize(pSrcImg, pSrcImg, cv::Size(800, 450));
	imgSize = cv::Size(pSrcImg.cols, pSrcImg.rows);	//(宽， 高)

	cv::Mat imgGray(imgSize, CV_8UC1);
	cv::Mat imgBlur;
	cv::Mat imgBin(imgSize, CV_8UC1);
	cv::Mat imgBinDet = cv::Mat::zeros(imgSize, CV_8UC1);

	cv::cvtColor(pSrcImg, imgGray, CV_BGR2GRAY);
	if (printMode <= PRTTP_TESTMOD)
	{
		cv::namedWindow("Gray", CV_WINDOW_AUTOSIZE);
		cv::imshow("Gray", imgGray);
	}
	cv::GaussianBlur(imgGray, imgBlur, cv::Size(7, 7), 3);
	
	//binThrehold = Otsu(out) + 60;
	cout << "binThrehold = " << binThrehold << endl;
	cv::threshold(imgBlur, imgBin, binThrehold, 255, cv::THRESH_BINARY);
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5), cv::Point(-1, -1));
	cv::morphologyEx(imgBin, imgBin, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 1);
	if (printMode <= PRTTP_TESTMOD)
	{
		cv::namedWindow("bin", CV_WINDOW_AUTOSIZE);
		cv::imshow("bin", imgBin);
	}

	vector<vector<cv::Point>> contours;
	cv::findContours(imgBin, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
	cv::drawContours(pSrcImg, contours, -1, 0, 2);
	contours_size = contours.size();
	if (contours_size == 0)
	{
		std::cout << "No lights detected!" << endl;
		pos[0] = 1;
		ansType = ANSTYPE_NON;
		return;
	}
	if (printMode <= PRTTP_ERRONLY)
	{
		cout << "contour_size = " << contours_size << endl;
	}
	//-----------------------计算光斑重心----------------------------
	cv::Moments moment;
	cv::vector<cv::Point2d> Center;
	int x = 0;
	int y = 0;
	for (int i = 0; i < contours_size; i++)
	{
		cv::Mat temp(contours.at(i));
		cv::Scalar color(0, 0, 255);
		moment = moments(temp, false);
		if (moment.m00 != 0)
		{
			x = cvRound(moment.m10 / moment.m00);
			y = cvRound(moment.m01 / moment.m00);
		}
		cv::Point gravity_center = cv::Point(x, y);
		//画重心点
		cv::circle(pSrcImg, gravity_center, 2, color, -1, 8);
		Center.push_back(gravity_center);
	}
	//以重心y值的大小进行顺序排序(从小到大)
	cv::Point temp_P;
	cv::vector<cv::Point> temp_Contour;
	for (int i = 0; i < contours_size; i++)
	{
		for (int j = contours_size - 1; j > i; j--)
		{
			if (Center[j].y < Center[j - 1].y)
			{
				//重心排序
				temp_P = Center[j];
				Center[j] = Center[j - 1];
				Center[j - 1] = temp_P;
				//轮廓排序
				temp_Contour = contours[j];
				contours[j] = contours[j - 1];
				contours[j - 1] = temp_Contour;
			}
		}
	}

	//根据光斑检测情况确定解算逻辑
	if (numLights <= contours_size)
	{
		cout << "All lights detected!" << endl;
		if (printMode <= PRTTP_TESTMOD)
		{
			drawDetConts(imgBinDet, contours, Center, contours_size);
			cv::namedWindow("binDet", CV_WINDOW_AUTOSIZE);
			cv::imshow("binDet", imgBinDet);
		}
		get_dockingCenter(Center, contours_size);
		cv::circle(pSrcImg, dockingCenter, 2, cv::Scalar(0,255,0), -1, 8);

		/*if (contours_size < numLights)
			estimateMode = ESTIMATE_MODE_SIMPLE;*/

		if (estimateMode == ESTIMATE_MODE_SIMPLE)
		{
			get_Angles(imgSize, pos);
			pos[0] = 2;
			ansType = ANSTYPE_POS;
		}
		else
		{
			// PointsCam 为图像坐标系中与世界坐标对应的点的向量（顺序同世界坐标系，一一对应）
			cv::vector<cv::Point2d> PointsCam(6);
			cv::vector<cv::Point2d> lightCenters(6);

			//把 center 中的后六个点取出来
			for (int i = 0; i < numLights; i++)
			{
				lightCenters.push_back(Center[contours_size - numLights + i]);
			}
			// 通过比较得出各个光源的正确的位置，做到一一对应
			get_targetPoints(lightCenters, PointsCam);

			//调用PoseEstimation类计算相对位姿
			PoseEstimation pnpsolver;
			//设置相机内部参
			pnpsolver.SetCameraMatrix(752.316145729373490, 771.167006926629140, 634.524795343358850, 328.191773442282910);
			//设置畸变系数
			pnpsolver.SetDistortionCoefficients(-0.341704164412008, 0.125056005119510, 0.000581877489323, -0.000029980361742, 0.000000000000000);
			//给出3D点
			pnpsolver.set_3DPoints(WorldPoints);
			//给出对应的2D点
			pnpsolver.set_2DPoints(PointsCam);

			if (pnpsolver.poseEsti(PoseEstimation::METHOD::CV_EPNP) == 0)
			{
				pos[0] = 3;
				pos[1] = pnpsolver.Position_target.x;
				pos[2] = pnpsolver.Position_target.y;
				pos[3] = pnpsolver.Position_target.z;
				pos[4] = sqrt(pos[1] * pos[1] + pos[2] * pos[2] + pos[3] * pos[3]);	//距离
				ansType = ANSTYPE_ALL;
			}
			else
			{
				cout << "Can't calculate the precise positon!" << endl;
				pos[0] = 1;
				ansType = ANSTYPE_NON;
			}
		}
	}
	else if (contours_size > 0)
	{
		cout << "Not enough lights detected!" << endl;
		if (printMode <= PRTTP_TESTMOD)
		{
			drawDetConts(imgBinDet, contours, Center, contours_size);
			cv::namedWindow("binDet", CV_WINDOW_AUTOSIZE);
			cv::imshow("binDet", imgBin);
		}
		get_dockingCenter(Center, contours_size);
		cv::circle(pSrcImg, dockingCenter, 2, cv::Scalar(0, 255, 0), -1, 8);
		get_Angles(imgSize, pos);
		pos[0] = 2;
		ansType = ANSTYPE_POS;
	}
	else
	{
		cout << "No lights detected!" << endl;
	}

	clock_t end_time = clock();
	if (printMode <= PRTTP_ERRONLY)
		std::cout << "time cost:" << (end_time - start_time) << endl;

	if (printMode < PRTTP_NOTHING)
	{
		cv::imshow("Process Result", pSrcImg);
	}

	return;
}

Preprocessing::Preprocessing()	//默认构造函数
{
	double camera[9] = { 752.316145729373490, 0, 634.524795343358850,
						 0, 771.167006926629140, 328.191773442282910,
					 	 0, 0, 1.0 };
	double distCo[5] = { -0.341704164412008, 0.125056005119510, 0.000581877489323, -0.000029980361742, 0.000000000000000 };
	
	cameraMat = cv::Mat(3, 3, CV_64FC1, camera);
	distCoeffs = cv::Mat(5, 1, CV_64FC1, distCo);
	numLights = 6;
	binThrehold = 245;
	printMode = PRTTP_TESTMOD;
	dockingCenter = cv::Point(0.0, 0.0);
	horizontalFOV = 2.094395;				//rad （120°）
	verticalFOV = 1.22173;					//rad （70°）
	estimateMode = ESTIMATE_MODE_COMPLEX;

	//to do 是否删除
	if (printMode < PRTTP_NOTHING)
	{
		cvNamedWindow("Process Result", CV_WINDOW_AUTOSIZE);
	}
}

Preprocessing::~Preprocessing()
{
}



//----------------------------------------------------------------------------------------------------------------------

void PoseEstimation::codeRotateByZ(double x, double y, double thetaz, double& outx, double& outy)
{
	double x1 = x;//将变量拷贝一次，保证&x == &outx这种情况下也能计算正确
	double y1 = y;
	double rz = thetaz * CV_PI / 180;
	outx = cos(rz) * x1 - sin(rz) * y1;
	outy = sin(rz) * x1 + cos(rz) * y1;
}

void PoseEstimation::codeRotateByY(double x, double z, double thetay, double& outx, double& outz)
{
	double x1 = x;
	double z1 = z;
	double ry = thetay * CV_PI / 180;
	outx = cos(ry) * x1 + sin(ry) * z1;
	outz = cos(ry) * z1 - sin(ry) * x1;
}

void PoseEstimation::codeRotateByX(double y, double z, double thetax, double& outy, double& outz)
{
	double y1 = y;//将变量拷贝一次，保证&y == &y这种情况下也能计算正确
	double z1 = z;
	double rx = thetax * CV_PI / 180;
	outy = cos(rx) * y1 - sin(rx) * z1;
	outz = cos(rx) * z1 + sin(rx) * y1;
}

cv::Point3f PoseEstimation::RotateByVector(double old_x, double old_y, double old_z, double vx, double vy, double vz, double theta)
{
	double r = theta * CV_PI / 180;
	double c = cos(r);
	double s = sin(r);
	double new_x = (vx*vx*(1 - c) + c) * old_x + (vx*vy*(1 - c) - vz * s) * old_y + (vx*vz*(1 - c) + vy * s) * old_z;
	double new_y = (vy*vx*(1 - c) + vz * s) * old_x + (vy*vy*(1 - c) + c) * old_y + (vy*vz*(1 - c) - vx * s) * old_z;
	double new_z = (vx*vz*(1 - c) - vy * s) * old_x + (vy*vz*(1 - c) + vx * s) * old_y + (vz*vz*(1 - c) + c) * old_z;
	return cv::Point3f(new_x, new_y, new_z);
}

void PoseEstimation::set_3DPoints(cv::vector<cv::Point3d> worldPoints)
{
	for (int i = 0; i < 6; i++)
		Points3D.push_back(worldPoints[i]);
}

void PoseEstimation::set_2DPoints(cv::vector<cv::Point2d> PointCam)
{
	for (int i = 0; i < 6; i++)
		Points2D.push_back(PointCam[i]);
}

void PoseEstimation::SetCameraMatrix(double fx, double fy, double u0, double v0)
{
	camera_matrix = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));
	camera_matrix.ptr<double>(0)[0] = fx;
	camera_matrix.ptr<double>(0)[2] = u0;
	camera_matrix.ptr<double>(1)[1] = fy;
	camera_matrix.ptr<double>(1)[2] = v0;
	camera_matrix.ptr<double>(2)[2] = 1.0f;
}

void PoseEstimation::SetDistortionCoefficients(double k_1, double  k_2, double  p_1, double  p_2, double k_3)
{
	distortion_coefficients = cv::Mat(5, 1, CV_64FC1, cv::Scalar::all(0));
	distortion_coefficients.ptr<double>(0)[0] = k_1;
	distortion_coefficients.ptr<double>(1)[0] = k_2;
	distortion_coefficients.ptr<double>(2)[0] = p_1;
	distortion_coefficients.ptr<double>(3)[0] = p_2;
	distortion_coefficients.ptr<double>(4)[0] = k_3;
}

int PoseEstimation::poseEsti(METHOD method = METHOD::CV_EPNP)
{
	//数据校验
	if (camera_matrix.cols == 0 || distortion_coefficients.cols == 0)
	{
		printf("ErrCode:-1,相机内参数或畸变参数未设置！\r\n");
		return -1;
	}

	if (Points3D.size() != Points2D.size())
	{
		printf("ErrCode:-2，3D点数量与2D点数量不一致！\r\n");
		return -2;
	}
	if (method == METHOD::CV_P3P || method == METHOD::CV_ITERATIVE)
	{
		if (Points3D.size() != 4)
		{
			printf("ErrCode:-2,使用CV_ITERATIVE或CV_P3P方法时输入的特征点数量应为4！\r\n");
			return -3;
		}
	}
	else
	{
		if (Points3D.size() < 4)
		{
			printf("ErrCode:-2,输入的特征点数量应大于4！\r\n");
			return -3;
		}
	}
	/*******************解决PNP问题*********************/
	//有三种方法求解
	solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, method);    

	//实测迭代法似乎只能用共面特征点求位置
	//solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, CV_ITERATIVE);    
	//Gao的方法可以使用任意四个特征点
	//solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, CV_P3P);
	//四个点及以上
	//solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, CV_EPNP);


	/*******************提取旋转矩阵*********************/
	double rm[9];
	RoteM = cv::Mat(3, 3, CV_64FC1, rm);
	Rodrigues(rvec, RoteM);
	double r11 = RoteM.ptr<double>(0)[0];
	double r12 = RoteM.ptr<double>(0)[1];
	double r13 = RoteM.ptr<double>(0)[2];
	double r21 = RoteM.ptr<double>(1)[0];
	double r22 = RoteM.ptr<double>(1)[1];
	double r23 = RoteM.ptr<double>(1)[2];
	double r31 = RoteM.ptr<double>(2)[0];
	double r32 = RoteM.ptr<double>(2)[1];
	double r33 = RoteM.ptr<double>(2)[2];
	TransM = tvec;

	//计算出相机坐标系的三轴旋转欧拉角，旋转后可以转出世界坐标系。
	//旋转顺序为z、y、x
	double thetaz = atan2(r21, r11) / CV_PI * 180;
	double thetay = atan2(-1 * r31, sqrt(r32*r32 + r33*r33)) / CV_PI * 180;
	double thetax = atan2(r32, r33) / CV_PI * 180;

	//相机系到世界系的三轴旋转欧拉角，相机坐标系照此旋转后可以与世界坐标系完全平行。
	//旋转顺序为z、y、x
	Theta_C2W.z = thetaz;
	Theta_C2W.y = thetay;
	Theta_C2W.x = thetax;

	//计算出世界系到相机系的三轴旋转欧拉角，世界系照此旋转后可以转出相机坐标系。
	//旋转顺序为x、y、z
	Theta_W2C.x = -1 * thetax;
	Theta_W2C.y = -1 * thetay;
	Theta_W2C.z = -1 * thetaz;


	/**********************此处计算出相机坐标系原点Oc在世界坐标系中的位置***********************/

	/***********************************************************************************/
	/* 当原始坐标系经过旋转z、y、x三次旋转后，与世界坐标系平行，向量OcOw会跟着旋转 */
	/* 而我们想知道的是两个坐标系完全平行时，OcOw的值 */
	/* 因此，原始坐标系每次旋转完成后，对向量OcOw进行一次反相旋转，最终可以得到两个坐标系完全平行时的OcOw */
	/* 该向量乘以-1就是世界坐标系下相机的坐标 */
	/***********************************************************************************/

	//提出平移矩阵，表示从相机坐标系原点，跟着向量(x,y,z)走，就到了世界坐标系原点
	double tx = tvec.ptr<double>(0)[0];
	double ty = tvec.ptr<double>(0)[1];
	double tz = tvec.ptr<double>(0)[2];

	//x y z 为唯一向量在相机原始坐标系下的向量值
	//也就是向量OcOw在相机坐标系下的值
	double x = tx, y = ty, z = tz;
	Position_OwInC.x = x;
	Position_OwInC.y = y;
	Position_OwInC.z = z;
	//进行三次反向旋转（z y x）
	codeRotateByZ(x, y, -1 * thetaz, x, y);
	codeRotateByY(x, z, -1 * thetay, x, z);
	codeRotateByX(y, z, -1 * thetax, y, z);


	//获得相机在世界坐标系下的位置坐标
	//即向量OcOw在世界坐标系下的值
	Position_OcInW.x = x*-1;
	Position_OcInW.y = y*-1;
	Position_OcInW.z = z*-1;

	//我们需要的值这里只需要 z 乘系数-1
	Position_target.x = x;
	Position_target.y = y;
	Position_target.z = -1 * z;

	return 0;
}

PoseEstimation::PoseEstimation()
{
	//初始化输出矩阵
	vector<double> rv(3), tv(3);
	cv::Mat rvec(rv), tvec(tv);
}

PoseEstimation::PoseEstimation(double fx, double fy, double u0, double v0,
							   double k_1, double  k_2, double  p_1, double  p_2, double k_3)
{
	//初始化输出矩阵
	vector<double> rv(3), tv(3);
	cv::Mat rvec(rv), tvec(tv);
	SetCameraMatrix(fx, fy, u0, v0);
	SetDistortionCoefficients(k_1, k_2, p_1, p_2, k_3);
}

//PoseEstimation::PoseEstimation()
//{
//	double camera[9] = { 0, 0, 0,
//						 0, 0, 0,
//						 0, 0, 0 };
//	double distCo[5] = { 0, 0, 0, 0, 0 };
//	cv::vector<cv::Point3d> realPos = { cv::Point3d(0, 0, 0), cv::Point3d(0, 0, 0),
//										cv::Point3d(0, 0, 0), cv::Point3d(0, 0, 0),
//										cv::Point3d(0, 0, 0) };
//	Points3D = realPos;
//	camera_matrix = cv::Mat(3, 3, CV_64FC1, camera);
//	distortion_coefficients = cv::Mat(5, 1, CV_64FC1, distCo);
//
//}

PoseEstimation::~PoseEstimation()
{
}
