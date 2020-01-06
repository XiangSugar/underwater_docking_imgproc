#pragma once

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <cmath>
#include <ctime>
#include <vector>

#define ANSTYPE_ALL 0
#define ANSTYPE_POS 1
#define ANSTYPE_NON 2

#define ESTIMATE_MODE_SIMPLE  1
#define ESTIMATE_MODE_COMPLEX 0

#define PRTTP_TESTMOD 0
#define PRTTP_ANSONLY 1
#define PRTTP_ERRONLY 2
#define PRTTP_NOTHING 3


//3D coordinates of the light spots in the world
//TO DO
static const cv::vector<cv::Point3d> WorldPoints = 
	{ cv::Point3d(0, 0, 0), cv::Point3d(0, 0, 0),
	  cv::Point3d(0, 0, 0), cv::Point3d(0, 0, 0),
	  cv::Point3d(0, 0, 0), cv::Point3d(0, 0, 0) 
	};

using namespace std;

class Preprocessing
{
private:
	int binThrehold;
	int numLights;
	int printMode;
	float horizontalFOV;
	float verticalFOV;
	int estimateMode;
	int remap_xy_flag;
	cv::Mat cameraMat;
	cv::Mat distCoeffs;
	cv::Mat mapx;
	cv::Mat mapy;
	cv::Point2d dockingCenter;
	cv::vector<cv::Point> targetPoints;

private:
	/* @ breif 计算最佳阈值
	*/
	int Otsu(cv::Mat &image);

	/* @breif 在图像坐标系中计算 docking 的中心位置坐标
	*/
	void get_dockingCenter(cv::vector<cv::Point2d> Centers, int contoursSize);

	/* @breif 在黑色背景上画出经过选择的目标光斑及其中心点
	*/
	void drawDetConts(cv::Mat & img, cv::vector<cv::vector<cv::Point>> contours, cv::vector<cv::Point2d> Centers, int contourSize);

	/* @brief 给出 docking 中心相对于摄像头的两个方位信息
	*/
	void get_Angles(cv::Size imageSize, double ans[]);
public:
	
	/* @breif 得到匹配后的点对
	*  @param lightCenters  待匹配的光斑坐标
	*  @param matchPoints   根据几何关系,按照规定的标号顺序匹配排列好之后的光斑坐标
	*/
	void get_targetPoints(cv::vector<cv::Point2d> lightCenters, cv::vector<cv::Point2d> & matchPoints);
	
	/* @breif 设置位姿解算的模式
	*  @param estimate_mode: ESTIMATE_MODE_SIMPLE  代表使用简单的解算方式，只能给出两个方位角信息
	*                        ESTIMATE_MODE_COMPLEX 代表使用复杂的解算方式，可以给出六个位姿信息
	*/
	void set_estimateMode(int estimate_mode);

	//***************************************图像分析函数*****************************************
	//**  调用这个函数来分析图像中的识别目标的位姿。                                               **
	//**                                                                                       **
	//**  image为图像输入，要求不能有过大的噪点。                                                 **
	//**                                                                                       **
	//**  pos为位姿输出的数组指针，长度至少为6。                                                  **
	//**  在 ESTIMATE_MODE_SIMPLE 模式下：													   **
	//**  pos[0]为信息校验位：
	//**			如果为0，无法解算结果(如:丢失光斑视野、解算出错等)，在控制程序中直接跳过控制更新，**
	///**			进入下一个控制循环														   **
	//**			如果为1，只有两个方位角信息，（如有部分光斑被检测到，或者人为设置简单解算模式）   **
	//**			在控制程序中正常处理即可													   **
	//**  pos[1]为输出的水平角位置，代表以摄像头方向为正方向时，目标的水平位置偏角。向右偏为正。      **
	//**  pos[2]为输出的竖直角位置，代表以摄像头方向为正方向时，目标的数值位置偏角。向上偏为正。      **
	//**  pos[3]为输出的目标距离，代表摄像头与识别目标的距离。                                     **
	//**
	//**  在 ESTIMATE_MODE_COMPLEX 模式下：													   **
	//**  pos[0]为信息校验位：															       **
	//**			0， 1 同上																   **
	//**			如果为2，则为精确解算模式，给出相对坐标信息，在控制程序中可用LOS制导进行控制	   **
	//**  pos[1]为自定义坐标系下的 x 坐标														   **
	//**  pos[2]为自定义坐标系下的 y 坐标														   **
	//**  pos[3]为自定义坐标系下的 z 坐标														   **
	//**  pos[4]为输出的目标距离，代表摄像头与识别目标的距离									   **
	//**																					   **
	//**  一些解释：                                                                            **
	//**    水平角位置和竖直角位置，直接代表目标中心在摄像头中的位置（x,y），是根据目标的像素值       **
	//**  直接换算为角度值而来的。															       **
	//**                                                                                       **
	//**  ansType为输出类别的输出。如果ansType的值为ANSTYPE_ALL，则识别到了全六自由度信息，pos[1    **
	//**  -4]全部都有效。如果ansType的值为ANSTYPE_POS，则只识别到了位置角信息，只有pos[1]和pos[2    **
	//**  ]是有效的。如果ansType的值为ANSTYPE_NON，则没有识别到目标，pos[]中无有效信息。            **
	//********************************************************************************************
	void process(cv::Mat image, double pos[], int & ansType);

	//***********************************运行模式设置函数*****************************************
	//**  这个函数用于设定运行模式。                                                            **
	//**  输入PRTTP_TESTMOD代表测试模式，会显示所有测试信息以及处理过程信息。                   **
	//**  输入PRTTP_ANSONLY代表结果测试模式，会显示中间过程的结果。                             **
	//**  输入PRTTP_ERRONLY代表只显示故障模式，会显示必要的故障信息以及耗时。建议在此模式运行。 **
	//**  输入PRTTP_NOTHING代表不显示信息模式，高速运行。                                       **
	//********************************************************************************************
	void setPrintMode(int modeI);

	//设置相机内参数矩阵
	void SetCameraMatrix(double fx, double fy, double u0, double v0);

	//设置畸变系数矩阵
	void SetDistortionCoefficients(double k_1, double  k_2, double  p_1, double  p_2, double k_3);

	//矫正畸变时获取 x, y 的映射关系
	void get_new_mapxy(cv::Size imageSize);
	
	//默认构造函数
	Preprocessing();

	//带参数初始化
	Preprocessing(double fx, double fy, double u0, double v0, double k_1, double  k_2, double  p_1, double  p_2, double k_3);

	~Preprocessing();
};

class PoseEstimation
{
private:
	cv::Mat camera_matrix;				//内参矩阵
	cv::Mat distortion_coefficients;	//畸变系数
	cv::Mat rvec;						//解得的旋转向量
	cv::Mat tvec;						//解得的平移向量
public:
	enum METHOD
	{
		CV_ITERATIVE = CV_ITERATIVE,
		CV_P3P = CV_P3P,
		CV_EPNP = CV_EPNP
	};
	/***********************位姿估计所用特征点**************************/
	vector<cv::Point3f> Points3D;//存储世界坐标
	vector<cv::Point2f> Points2D;//存储图像坐标

	/***********************位姿估计结果**************************/
	//最后求出的旋转矩阵与平移矩阵
	cv::Mat RoteM, TransM;
	//世界系到相机系的三轴旋转欧拉角，世界系照此旋转后可以与相机坐标系完全平行。
	//旋转顺序为x、y、z
	cv::Point3f Theta_W2C;
	//相机系到世界系的三轴旋转欧拉角，相机坐标系照此旋转后可以与世界坐标系完全平行。
	//旋转顺序为z、y、x
	cv::Point3f Theta_C2W;
	//相机坐标系中，世界坐标系原点Ow的坐标
	cv::Point3f Position_OwInC;
	//世界坐标系中，相机坐标系原点Oc的坐标（的相反数）
	cv::Point3f Position_OcInW;
	//特定于对接项目所需要的坐标值
	cv::Point3f Position_target;

public:
	/********************公有静态方法*********************/

	//将空间点绕Z轴旋转
	//输入参数 x y为空间点原始x y坐标
	//thetaz为空间点绕Z轴旋转多少度，角度制范围在-180到180
	//outx outy为旋转后的结果坐标
	static void codeRotateByZ(double x, double y, double thetaz, double& outx, double& outy);
	//将空间点绕Y轴旋转
	//输入参数 x z为空间点原始x z坐标
	//thetay为空间点绕Y轴旋转多少度，角度制范围在-180到180
	//outx outz为旋转后的结果坐标
	static void codeRotateByY(double x, double z, double thetay, double& outx, double& outz);
	//将空间点绕X轴旋转
	//输入参数 y z为空间点原始y z坐标
	//thetax为空间点绕X轴旋转多少度，角度制，范围在-180到180
	//outy outz为旋转后的结果坐标
	static void codeRotateByX(double y, double z, double thetax, double& outy, double& outz);
	//点绕任意向量旋转，右手系
	//输入参数old_x，old_y，old_z为旋转前空间点的坐标
	//vx，vy，vz为旋转轴向量
	//theta为旋转角度角度制，范围在-180到180
	//返回值为旋转后坐标点
	static cv::Point3f RotateByVector(double old_x, double old_y, double old_z, double vx, double vy, double vz, double theta);


	/***********************************公有方法***************************************/

	//*********************************************************************************
	//	解PNP问题，获得位姿信息
	//	调用后在RoteM, TransM, W2CTheta等属性中提取计算结果，属性说明参见注释
	//	opencv调用方法：
	//		CV_ITERATIVE
	//		CV_P3P（默认）
	//		CV_EPNP，		具体请参见Opencv documentation.
	//	实测:
	//		CV_ITERATIVE迭代法似乎只能用4个共面特征点求解，5个点或非共面4点解不出正确的解
	//		CV_P3P的Gao的方法可以使用任意四个特征点，特征点数量不能少于4也不能多于4
	//		CV_EPNP方法可以实现特征点数>=4的问题求解，不需要4点共面
	//	返回值：
	//		0   正确
	//		-1  相机内参数或畸变参数未设置
	//		-2  未提供足够的特征点，或特征点数目不匹配
	//		-3  输入的点数据有误，详见printf信息
	//*********************************************************************************
	int poseEsti(METHOD method);
	//void poseEsti(cv::vector<cv::Point> targetPoints, double ans[]);

	//设置相机内参数矩阵
	void SetCameraMatrix(double fx, double fy, double u0, double v0);

	//设置畸变系数矩阵
	void SetDistortionCoefficients(double k_1, double  k_2, double  p_1, double  p_2, double k_3);

	void set_3DPoints(cv::vector<cv::Point3d> worldPoints);
	void set_2DPoints(cv::vector<cv::Point2d> PointCam);

	PoseEstimation();
	//带参数初始化
	PoseEstimation(double fx, double fy, double u0, double v0, double k_1, double  k_2, double  p_1, double  p_2, double k_3);
	~PoseEstimation();
};

void ansToChar(double * in, char * out);
