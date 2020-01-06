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
	/* @ breif ���������ֵ
	*/
	int Otsu(cv::Mat &image);

	/* @breif ��ͼ������ϵ�м��� docking ������λ������
	*/
	void get_dockingCenter(cv::vector<cv::Point2d> Centers, int contoursSize);

	/* @breif �ں�ɫ�����ϻ�������ѡ���Ŀ���߼������ĵ�
	*/
	void drawDetConts(cv::Mat & img, cv::vector<cv::vector<cv::Point>> contours, cv::vector<cv::Point2d> Centers, int contourSize);

	/* @brief ���� docking �������������ͷ��������λ��Ϣ
	*/
	void get_Angles(cv::Size imageSize, double ans[]);
public:
	
	/* @breif �õ�ƥ���ĵ��
	*  @param lightCenters  ��ƥ��Ĺ������
	*  @param matchPoints   ���ݼ��ι�ϵ,���չ涨�ı��˳��ƥ�����к�֮��Ĺ������
	*/
	void get_targetPoints(cv::vector<cv::Point2d> lightCenters, cv::vector<cv::Point2d> & matchPoints);
	
	/* @breif ����λ�˽����ģʽ
	*  @param estimate_mode: ESTIMATE_MODE_SIMPLE  ����ʹ�ü򵥵Ľ��㷽ʽ��ֻ�ܸ���������λ����Ϣ
	*                        ESTIMATE_MODE_COMPLEX ����ʹ�ø��ӵĽ��㷽ʽ�����Ը�������λ����Ϣ
	*/
	void set_estimateMode(int estimate_mode);

	//***************************************ͼ���������*****************************************
	//**  �����������������ͼ���е�ʶ��Ŀ���λ�ˡ�                                               **
	//**                                                                                       **
	//**  imageΪͼ�����룬Ҫ�����й������㡣                                                 **
	//**                                                                                       **
	//**  posΪλ�����������ָ�룬��������Ϊ6��                                                  **
	//**  �� ESTIMATE_MODE_SIMPLE ģʽ�£�													   **
	//**  pos[0]Ϊ��ϢУ��λ��
	//**			���Ϊ0���޷�������(��:��ʧ�����Ұ����������)���ڿ��Ƴ�����ֱ���������Ƹ��£�**
	///**			������һ������ѭ��														   **
	//**			���Ϊ1��ֻ��������λ����Ϣ�������в��ֹ�߱���⵽��������Ϊ���ü򵥽���ģʽ��   **
	//**			�ڿ��Ƴ���������������													   **
	//**  pos[1]Ϊ�����ˮƽ��λ�ã�����������ͷ����Ϊ������ʱ��Ŀ���ˮƽλ��ƫ�ǡ�����ƫΪ����      **
	//**  pos[2]Ϊ�������ֱ��λ�ã�����������ͷ����Ϊ������ʱ��Ŀ�����ֵλ��ƫ�ǡ�����ƫΪ����      **
	//**  pos[3]Ϊ�����Ŀ����룬��������ͷ��ʶ��Ŀ��ľ��롣                                     **
	//**
	//**  �� ESTIMATE_MODE_COMPLEX ģʽ�£�													   **
	//**  pos[0]Ϊ��ϢУ��λ��															       **
	//**			0�� 1 ͬ��																   **
	//**			���Ϊ2����Ϊ��ȷ����ģʽ���������������Ϣ���ڿ��Ƴ����п���LOS�Ƶ����п���	   **
	//**  pos[1]Ϊ�Զ�������ϵ�µ� x ����														   **
	//**  pos[2]Ϊ�Զ�������ϵ�µ� y ����														   **
	//**  pos[3]Ϊ�Զ�������ϵ�µ� z ����														   **
	//**  pos[4]Ϊ�����Ŀ����룬��������ͷ��ʶ��Ŀ��ľ���									   **
	//**																					   **
	//**  һЩ���ͣ�                                                                            **
	//**    ˮƽ��λ�ú���ֱ��λ�ã�ֱ�Ӵ���Ŀ������������ͷ�е�λ�ã�x,y�����Ǹ���Ŀ�������ֵ       **
	//**  ֱ�ӻ���Ϊ�Ƕ�ֵ�����ġ�															       **
	//**                                                                                       **
	//**  ansTypeΪ���������������ansType��ֵΪANSTYPE_ALL����ʶ����ȫ�����ɶ���Ϣ��pos[1    **
	//**  -4]ȫ������Ч�����ansType��ֵΪANSTYPE_POS����ֻʶ����λ�ý���Ϣ��ֻ��pos[1]��pos[2    **
	//**  ]����Ч�ġ����ansType��ֵΪANSTYPE_NON����û��ʶ��Ŀ�꣬pos[]������Ч��Ϣ��            **
	//********************************************************************************************
	void process(cv::Mat image, double pos[], int & ansType);

	//***********************************����ģʽ���ú���*****************************************
	//**  ������������趨����ģʽ��                                                            **
	//**  ����PRTTP_TESTMOD�������ģʽ������ʾ���в�����Ϣ�Լ����������Ϣ��                   **
	//**  ����PRTTP_ANSONLY����������ģʽ������ʾ�м���̵Ľ����                             **
	//**  ����PRTTP_ERRONLY����ֻ��ʾ����ģʽ������ʾ��Ҫ�Ĺ�����Ϣ�Լ���ʱ�������ڴ�ģʽ���С� **
	//**  ����PRTTP_NOTHING������ʾ��Ϣģʽ���������С�                                       **
	//********************************************************************************************
	void setPrintMode(int modeI);

	//��������ڲ�������
	void SetCameraMatrix(double fx, double fy, double u0, double v0);

	//���û���ϵ������
	void SetDistortionCoefficients(double k_1, double  k_2, double  p_1, double  p_2, double k_3);

	//��������ʱ��ȡ x, y ��ӳ���ϵ
	void get_new_mapxy(cv::Size imageSize);
	
	//Ĭ�Ϲ��캯��
	Preprocessing();

	//��������ʼ��
	Preprocessing(double fx, double fy, double u0, double v0, double k_1, double  k_2, double  p_1, double  p_2, double k_3);

	~Preprocessing();
};

class PoseEstimation
{
private:
	cv::Mat camera_matrix;				//�ڲξ���
	cv::Mat distortion_coefficients;	//����ϵ��
	cv::Mat rvec;						//��õ���ת����
	cv::Mat tvec;						//��õ�ƽ������
public:
	enum METHOD
	{
		CV_ITERATIVE = CV_ITERATIVE,
		CV_P3P = CV_P3P,
		CV_EPNP = CV_EPNP
	};
	/***********************λ�˹�������������**************************/
	vector<cv::Point3f> Points3D;//�洢��������
	vector<cv::Point2f> Points2D;//�洢ͼ������

	/***********************λ�˹��ƽ��**************************/
	//����������ת������ƽ�ƾ���
	cv::Mat RoteM, TransM;
	//����ϵ�����ϵ��������תŷ���ǣ�����ϵ�մ���ת��������������ϵ��ȫƽ�С�
	//��ת˳��Ϊx��y��z
	cv::Point3f Theta_W2C;
	//���ϵ������ϵ��������תŷ���ǣ��������ϵ�մ���ת���������������ϵ��ȫƽ�С�
	//��ת˳��Ϊz��y��x
	cv::Point3f Theta_C2W;
	//�������ϵ�У���������ϵԭ��Ow������
	cv::Point3f Position_OwInC;
	//��������ϵ�У��������ϵԭ��Oc�����꣨���෴����
	cv::Point3f Position_OcInW;
	//�ض��ڶԽ���Ŀ����Ҫ������ֵ
	cv::Point3f Position_target;

public:
	/********************���о�̬����*********************/

	//���ռ����Z����ת
	//������� x yΪ�ռ��ԭʼx y����
	//thetazΪ�ռ����Z����ת���ٶȣ��Ƕ��Ʒ�Χ��-180��180
	//outx outyΪ��ת��Ľ������
	static void codeRotateByZ(double x, double y, double thetaz, double& outx, double& outy);
	//���ռ����Y����ת
	//������� x zΪ�ռ��ԭʼx z����
	//thetayΪ�ռ����Y����ת���ٶȣ��Ƕ��Ʒ�Χ��-180��180
	//outx outzΪ��ת��Ľ������
	static void codeRotateByY(double x, double z, double thetay, double& outx, double& outz);
	//���ռ����X����ת
	//������� y zΪ�ռ��ԭʼy z����
	//thetaxΪ�ռ����X����ת���ٶȣ��Ƕ��ƣ���Χ��-180��180
	//outy outzΪ��ת��Ľ������
	static void codeRotateByX(double y, double z, double thetax, double& outy, double& outz);
	//��������������ת������ϵ
	//�������old_x��old_y��old_zΪ��תǰ�ռ�������
	//vx��vy��vzΪ��ת������
	//thetaΪ��ת�ǶȽǶ��ƣ���Χ��-180��180
	//����ֵΪ��ת�������
	static cv::Point3f RotateByVector(double old_x, double old_y, double old_z, double vx, double vy, double vz, double theta);


	/***********************************���з���***************************************/

	//*********************************************************************************
	//	��PNP���⣬���λ����Ϣ
	//	���ú���RoteM, TransM, W2CTheta����������ȡ������������˵���μ�ע��
	//	opencv���÷�����
	//		CV_ITERATIVE
	//		CV_P3P��Ĭ�ϣ�
	//		CV_EPNP��		������μ�Opencv documentation.
	//	ʵ��:
	//		CV_ITERATIVE�������ƺ�ֻ����4��������������⣬5�����ǹ���4��ⲻ����ȷ�Ľ�
	//		CV_P3P��Gao�ķ�������ʹ�������ĸ������㣬������������������4Ҳ���ܶ���4
	//		CV_EPNP��������ʵ����������>=4��������⣬����Ҫ4�㹲��
	//	����ֵ��
	//		0   ��ȷ
	//		-1  ����ڲ�����������δ����
	//		-2  δ�ṩ�㹻�������㣬����������Ŀ��ƥ��
	//		-3  ����ĵ������������printf��Ϣ
	//*********************************************************************************
	int poseEsti(METHOD method);
	//void poseEsti(cv::vector<cv::Point> targetPoints, double ans[]);

	//��������ڲ�������
	void SetCameraMatrix(double fx, double fy, double u0, double v0);

	//���û���ϵ������
	void SetDistortionCoefficients(double k_1, double  k_2, double  p_1, double  p_2, double k_3);

	void set_3DPoints(cv::vector<cv::Point3d> worldPoints);
	void set_2DPoints(cv::vector<cv::Point2d> PointCam);

	PoseEstimation();
	//��������ʼ��
	PoseEstimation(double fx, double fy, double u0, double v0, double k_1, double  k_2, double  p_1, double  p_2, double k_3);
	~PoseEstimation();
};

void ansToChar(double * in, char * out);
