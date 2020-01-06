
#define _WINSOCK_DEPRECATED_NO_WARNINGS

//#include <opencv2/core/types_c.h>
#include <iostream>
#include "../include/recongnizer.h"
#include <WINSOCK2.H>   
#include <stdio.h> 
#include <fstream>


//���������ʹ�õĳ���
//��������IP��ַ(ÿ����ʵ���ʱ����Ҫcheck���IP��ַ��ȷ������ȷ�ģ���Ϊÿ��·���������IP���ܲ�һ��)      
#define SERVER_ADDRESS "192.168.0.100" 
#define PORT           6000         //�������Ķ˿ں�      
#define MSGSIZE        1024         //�շ��������Ĵ�С      
#pragma comment(lib, "ws2_32.lib")      

int main(int argc, char** argv)
{
	//using namespace std;
	std::ofstream fout1("../result/calculation_result.txt");
	fout1 << "x " << "y " << "z " << "d" << std::endl;
	std::ofstream fout2("../resuslt/simple_culculation.txt");
	fout2 << "hor_angle " << "ver_angle " << std::endl;
	WSADATA wsaData;
	SOCKET sClient;
	SOCKADDR_IN server;
	char szMessage[MSGSIZE];
	//int ret;
	WSAStartup(0x0202, &wsaData);
	sClient = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	memset(&server, 0, sizeof(SOCKADDR_IN));
	server.sin_family = PF_INET;
	server.sin_port = htons(PORT);
	server.sin_addr.s_addr = inet_addr(SERVER_ADDRESS);
	connect(sClient, (struct sockaddr *) &server, sizeof(SOCKADDR_IN));

	int count = 0;
	int ansType;
	//IplImage��OpenCV��C���Ե�ͼ������; cv::Mat��OpenCV��C++���Ե�ͼ������;
	IplImage* pSrcImg = NULL;
	//��ȡ��Ƶ��
	CvCapture* capture = cvCaptureFromFile("http://192.168.0.122:8090/?action=stream?dummy=param.mjpg");
	if (capture == 0)
	{
		cvReleaseCapture(&capture);
		std::cout << "��ȡ��Ƶ������" << std::endl;
		return 0;
	}
	Preprocessing * preproc = new Preprocessing();
	preproc->SetCameraMatrix(752.316145729373490, 771.167006926629140, 634.524795343358850, 328.191773442282910);
	preproc->SetDistortionCoefficients(-0.341704164412008, 0.125056005119510, 0.000581877489323, -0.000029980361742, 0.00000000000000);
	//������ʾ��Ϣ��ģʽ
	preproc->setPrintMode(PRTTP_ANSONLY);

	while (1)
	{
		/*
		for (int i = 0; i < 3; i++)//һ��ѭ��ץ3֡�����һ֡��������
		{
			//if (pSrcImg = cvLoadImage("point0.jpg"))
			if (pSrcImg = cvQueryFrame(capture))
			{
				//cv::Mat Img(pSrcImg);	//ͼ���ʽת��	ǳ����
				//cvShowImage("Video", pSrcImg);
			}
			else
			{
				cvReleaseCapture(&capture);
				std::cout << "no image stream!\n";
				std::system("pause");//������ͣ�������н��
				return 0;
			}
		}
		*/
		pSrcImg = cvQueryFrame(capture);
		if (!pSrcImg)
		{
			cvReleaseCapture(&capture);
			std::cout << "no image stream!\n";
			std::system("pause");//������ͣ�������н��
			return 0;
		}

		double ans[5];
		cv::Mat Img(pSrcImg);
		cv::imshow("Video", Img);
		
		std::cout << "---------------------------------" << std::endl;
		preproc->process(Img, ans, ansType);
		
		if (ansType == ANSTYPE_NON)
			std::cout << "NO SOLUTION!" << std::endl;
		else if (ansType == ANSTYPE_POS)
		{
			std::cout << "POSITION ONLY!" << std::endl;
			std::cout << "horiziontal angle:" << ans[1] << std::endl;
			std::cout << "vertical    angle:" << ans[2] << std::endl;
			fout2 << ans[1] << " " << ans[2] << std::endl;
		}
		else
		{
			std::cout << "ALL SOLUTION GET!" << std::endl;
			std::cout << "x		 :" << ans[1] << std::endl;
			std::cout << "y		 :" << ans[2] << std::endl;
			std::cout << "z		 :" << ans[3] << std::endl;
			std::cout << "distance:" << ans[4] << std::endl;
			fout1 << ans[1] << " " << ans[2] << " " 
				  << ans[3] << " " << ans[4] << std::endl;
		}

		// ��������      
		ansToChar(ans, szMessage);
		send(sClient, szMessage, 20, 0);
		std::cout << "OK!" << std::endl;
		cvWaitKey(5);
	}
	cvReleaseCapture(&capture);
	std::cout << "no image stream!\n";
	std::system("pause");
	return 0;
}

