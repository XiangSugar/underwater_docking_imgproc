
#define _WINSOCK_DEPRECATED_NO_WARNINGS

#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <math.h>
#include <time.h>
#include <vector>
#include <algorithm>
#include "recongnizer.h"
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
	using namespace std;
	ofstream fout1("calculation_result.txt");
	fout1 << "x " << "y " << "z " << "d" << endl;
	ofstream fout2("simple_culculation.txt");
	fout2 << "hor_angle " << "ver_angle " << endl;
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
		std::cout << "��ȡ��Ƶ������" << endl;
		return 0;
	}
	Preprocessing * preproc = new Preprocessing();
	//������ʾ��Ϣ��ģʽ
	preproc->setPrintMode(PRTTP_ERRONLY);

	while (1)
	{
		for (int i = 0; i < 3; i++)//һ��ѭ��ץ3֡�����һ֡��������
		{
			//if (pSrcImg = cvLoadImage("point0.jpg"))
			if (pSrcImg = cvQueryFrame(capture))
			{
				//cv::Mat Img(pSrcImg);	//ͼ���ʽת��	ǳ����
				//�˴���ȡԭʼ��Ƶ
				cvNamedWindow("Video", CV_WINDOW_AUTOSIZE);
				cvShowImage("Video", pSrcImg);
			}
			else
			{
				cvReleaseCapture(&capture);
				std::cout << "no image stream!\n";
				std::system("pause");//������ͣ�������н��
				return 0;
			}
		}
		double ans[5];
		cv::Mat Img(pSrcImg);
		cout << "---------------------------------" << endl;
		preproc->process(Img, ans, ansType);
		
		if (ansType == ANSTYPE_NON)
			cout << "NO SOLUTION!" << endl;
		else if (ansType == ANSTYPE_POS)
		{
			cout << "POSITION ONLY!" << endl;
			cout << "horiziontal angle:" << ans[1] << endl;
			cout << "vertical    angle:" << ans[2] << endl;
			fout2 << ans[1] << " " << ans[2] << endl;
		}
		else
		{
			cout << "ALL SOLUTION GET!" << endl;
			cout << "x		 :" << ans[1] << endl;
			cout << "y		 :" << ans[2] << endl;
			cout << "z		 :" << ans[3] << endl;
			cout << "distance:" << ans[4] << endl;
			fout1 << ans[1] << " " << ans[2] << " " 
				  << ans[3] << " " << ans[4] << endl;
		}

		// ��������      
		ansToChar(ans, szMessage);
		send(sClient, szMessage, 20, 0);
		cout << "OK!" << endl;
		cvWaitKey(5);
	}
	cvReleaseCapture(&capture);
	cout << "no image stream!\n";
	system("pause");
	return 0;
}

