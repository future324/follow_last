// Opencvͷ�ļ�
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
// ����ͷ�ļ�
#pragma region 
#include <iostream>
#include <ctype.h>
#include <stdio.h>
#include <conio.h>
#include <float.h>
#include <math.h>
// windowsͷ�ļ�
#include <windows.h>
// ģ��ͷ�ļ�
#include "KV2_Grabber.h"
#include "USART.h"
// �����ռ�
using namespace cv;
using namespace std;
// ����ͨ������
Comm USART;
// �����ƶ�����
unsigned short Speed = 0;
short Yaw = 0;
short AngleRate = 0;
// ����״̬����
bool TraceEnabled = false;
bool bodyIsReady = false;
// Z��ֵ
float Zdiff = 0, Zlast = 1;
// Kinect����	// knct
KinectV2_SG k2;
// ��������;
CameraSpacePoint BodyXYZ;
// �ָ������
Point GravityCenter(Mat src);

// ������
Mat frame;
bool automated = 0;
bool NOJoints = false;
// ��������
BYTE move_Mecanum(UINT16 Gspeed, INT Angle, CHAR TState);
void TraceStop();
void Trace();
void SetSpeed(unsigned short speed, short yaw, short anglerate);
Comm m_Comm1;
BYTE Send_Buff[10] = { "1234" };
int main(int argc, char** argv)
{
	
	// ͨ�ų�ʼ��
start:  // init
	m_Comm1.openport(L"COM3", 115200);


	// �������ģ��
	for (;;)
	{
		// ���ݸ���
		k2.Update_DepthFrame();
		k2.Update_IRFrame();
		k2.Map_CameraSpaceIR();
		frame = k2.FrameIR;
		k2.Update_Body();
		// �Զ�����
		if (!TraceEnabled)										// ���û�м���
		{ 
			if (k2.ClosestBodyID >= 0)
			{

				cout << ".";
			}
			// ��⶯��:
			if (k2.JointData[k2.ClosestBodyID][JointType_HandRight].Position.Y > k2.JointData[k2.ClosestBodyID][JointType_ElbowRight].Position.Y - 0.1&&
				k2.JointData[k2.ClosestBodyID][JointType_HandRight].Position.X > k2.JointData[k2.ClosestBodyID][JointType_ElbowLeft].Position.X&&
				k2.JointData[k2.ClosestBodyID][JointType_HandRight].Position.X < k2.JointData[k2.ClosestBodyID][JointType_ElbowRight].Position.X)
			{													// �ַ�����ǰ�߶ȣ��Ի����ſ�������ȭ���ɼ���
				if (k2.HandData[k2.ClosestBodyID].HandRight == HandState_Open)
				{
					bodyIsReady = true;							// ׼��
					cout << "ready...";
				}
				else
				{
					if (bodyIsReady&&k2.HandData[k2.ClosestBodyID].HandRight == HandState_Closed)
					{
						TraceEnabled = true;					// ����
						cout << "OK";
					}
					else
					{
						bodyIsReady = false;
					}
				}
			}
			TraceStop();										// ֹͣ�ƶ�
		}
		else
		{
			// ����Ƿ���ֹͣ����	
			if (k2.JointData[k2.ClosestBodyID][JointType_HandRight].Position.X < k2.JointData[k2.ClosestBodyID][JointType_HandLeft].Position.X&&
				k2.JointData[k2.ClosestBodyID][JointType_HandRight].Position.Y > k2.JointData[k2.ClosestBodyID][JointType_ElbowRight].Position.Y)
			{
				cout << "done";
				TraceEnabled = false;
				TraceStop();
				continue;										// ����ͣ�²��ȴ�����
			}
			else												// û���������������
			{
				Zlast = BodyXYZ.Z;
				if (k2.ClosestBodyID < 0 || NOJoints)			// ���û�Ǽ�
				{
					Trace();									// ������ȷָ�ó�λ��
				}
				else											// ����йǼ�
				{
					DepthSpacePoint p;							// ӳ��ؽ����굽���֡
					k2.pCoordinateMapper->MapCameraPointToDepthSpace(k2.JointData[k2.ClosestBodyID][JointType_SpineShoulder].Position, &p);
					int pixelX = (p.X + 0.5f);
					int pixelY = (p.Y + 0.5f);
					if (pixelX>0 && pixelY>0)					// ͳһΪ����ռ�����
					{
						BodyXYZ.X = k2.Coordinate.at(0).at<float>(pixelY, pixelX);
						BodyXYZ.Y = k2.Coordinate.at(1).at<float>(pixelY, pixelX);
						BodyXYZ.Z = k2.Coordinate.at(2).at<float>(pixelY, pixelX);
					}
				}
				Zdiff = BodyXYZ.Z - Zlast;
			}
			// �����ƶ�
			Zdiff = BodyXYZ.Z - Zlast;
			if (Zdiff >= 0.19)	// ��ֹͻȻ����
			{
				//move_Mecanum(0, 0, 0);
				//cout << abs(k2.JointData[k2.ClosestBodyID][JointType_HandRight].Position.X - k2.JointData[k2.ClosestBodyID][JointType_HandLeft].Position.Z) << endl;
				TraceStop();
			}
			float x = BodyXYZ.X;
			float y = BodyXYZ.Y;
			float z = BodyXYZ.Z;
			DepthSpacePoint dp;
			k2.pCoordinateMapper->MapCameraPointToDepthSpace(BodyXYZ, &dp);
			Point cir((int)dp.X, (int)dp.Y);
			circle(frame, cir, 3, Scalar(128, 255, 0), -1, 8);
			imshow("", frame);
			cv::moveWindow("", 700, 250);
			//**************************************************************************     
			x -= 0.05;
			float distance = sqrt(pow(z, 2) + pow(x, 2)) - 1.0f;
			//float distance =//z - 0.8;

			if (distance > 0)
			{
				Yaw = 0;	// ����
				cout << (Speed = abs(distance * 420)) << endl;
			}
			else
			{
				Yaw = 180;	// ������
				cout << (Speed = abs(distance * 1000)) << endl;
			}

			cout << (Speed = abs(distance * 420)) << endl;
			AngleRate = -(short)(x * 100);//k=500

			SetSpeed(Speed, Yaw, AngleRate);//speed ƽ���ٶ�||yaw �����||anglerate �����ٶ�

			waitKey(1);
		}
	}

	return 0;
}
void TraceStop()
{
	SetSpeed(0, 0, 0);
	Sleep(10);
}
/************************************************************************************
����ͨ�ź�����(������)
������----------------------------------------------------------------------------
Tspeed	translation speed	ƽ���ٶȣ�����ת�ٶ��޹أ��˶�ʱ���߿�ֱ�ӵ���
Dangle	drift angle			ƽ��ƫ�ǣ�����ǰ��Ϊ0�㣬˳ʱ��Ϊ������Ϊ��ֵ
Rspeed	rotating speed		��ת�ٶȣ���ֵΪ˳ʱ�뷽�򣬸�ֵ����ʱ�뷽��
���ݰ���--------------------------------------------------------------------------
"[" + �����ַ� + Tspeed + Dangle + Rspeed + Checksum + "]"
��ע�� 5�ֽ�	 �ȵ�λ���λ����  ���Ǹ�ֵ		|
---------------------------------У�������Լ��������������
************************************************************************************/


/**************************************************************************************************************************
���̣�
1���ȸ��ݹǼ�λ�ø��٣�����ʶ���־��뼹���������2�����������ſ���ͣ�£��Ǹ��Ǽ�id�����Ч��
ֱ���Ǽܶ�ʧ���߸ղ��Ǹ�������ȭ�ٱ����Ч��
���ʱ��һֱ�Ǹ�������ĹǼ�
2���Ǽ�������ƶ�����ʧ��ʱ���Ǹ�����ԭ�������һ�µ�ʱ��ʧ�ĵ㸽����ԼΪ1ƽ�����׵ľ��Σ������ֵ�Ƿ�͹Ǽܶ�ʧǰ��Zֵ���࣬�Ǽܿ���һЩ����Զ�ڹǼ�0.15���ڹǼ�0.25
�˴�Ҫ�����Ƿ�����ĹǼܵ��жϣ���Y���߹����ǳ�����ſ���ϻ��߷������˲Ż�����
3ֱ�ӶԵ�ǰ��������ı߽�����жϣ��ҳ����ģ�X��Z���
4���������Ǽܶ������ж��Ƿ�ֹͣ����
5���м����õ����ݲ����в��������е��ò������ݵĿ���
��ע��
ok 1������϶�Z��������3.5m�ľͲ����ˣ����������䵽һ���������ٶ��ߣ���z��Զ��ʱ��ͣ�������Ƚ�����ʵ�����飩
ok 2���֡30FPS���ǹ��翨�˻����㷨���ǲ�����֡���صģ������ϼ��ٶȽϴ�Ҳ�ܸ���û����Ӱ���Թ���û���ƣ������·����·�
todo 3�������ò�Զ���ڷ��������Ǽܻ����в�Զ�Ŀ�ȴ���0.1m�Ĺ����棨�������Ż����ˣ�����ͷת�����ڵĴ���λ��"��һ��"
������ֹǼܣ�����ȡ���������Ƿ����Ѿ���ʶ���ˣ���ʶ�ʹ��к������к�Ҫ������1�����ڲ��ٴ�
������ֹ����棬̧ͷ��һ��,���ͷ��ʶ�������������к���
û���ֻ����ŶȲ����ƽ���ٿ��ظ��ٵ��˵Ĳ���λ��
todo 4תͷ����:X��Z�����Ƕȣ�YΪ���Ժõľ�ֵ̬��Yaw���ٶȴ�Pitch���ٶ�С
2015-08-16 01:29:23
**************************************************************************************************************************/

/*
�����õĹǼܵ�
k2.pCoordinateMapper->MapCameraPointToDepthSpace(BodyXYZ, &p);
��	vector<Mat> Coordinate;
split(k2.CameraSpaceIR, Coordinate);// �����XYZ�� at ��0 1 2��
Coordinate.at(2).at<float>(pixelX, pixelY));
*/

/*************************************************************************************************************************
/*	���ͣ�	void
/*	������	Tracing()
/*	��;��	���Ǽܶ�ʧ�л������������������������
/*	ע�⣺	һ����һ����Ч�����꣬
/*************************************************************************************************************************/
#define Z_DiffThreshold 0.2
void Trace()
{
	// ��ʵ����ռ�����ӳ�䵽������������λ��
	DepthSpacePoint p;
	Mat m;
	k2.pCoordinateMapper->MapCameraPointToDepthSpace(BodyXYZ, &p);
	int pixelX = (p.X + 0.5f);
	int pixelY = (p.Y + 0.5f);
	k2.Coordinate.at(2).copyTo(m);//��Zͨ�������������д���
								  //Zcur�ǵ�ǰ�˵�Z�minΪ������������Z����ֵ��maxΪԶ����
	float Zcur = BodyXYZ.Z, Zmin = Zcur - 0.3, Zmax = Zcur + 0.4;
	float Ymin = -0.1, Ymax = 1;
	for (int i = 0; i<424; i++)
		for (int j = 0; j<512; j++)
			m.ptr<float>(i)[j] = (k2.Coordinate.at(2).ptr<float>(i)[j]>Zmin && k2.Coordinate.at(2).ptr<float>(i)[j]<Zmax &&
				k2.Coordinate.at(1).ptr<float>(i)[j]>Ymin && k2.Coordinate.at(1).ptr<float>(i)[j]<Ymax);
	m.convertTo(m, CV_8U, 2);
	threshold(m, m, 1, 255, CV_THRESH_BINARY);
	//imshow("m", m);
	//;
	//2������̬ѧ����  
	//�����  
	Mat element;
	// �������͸�ʴ
	element = getStructuringElement(MORPH_ELLIPSE, Size(10, 10));
	morphologyEx(m, m, MORPH_ERODE, element);
	morphologyEx(m, m, MORPH_DILATE, element);
	element = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
	morphologyEx(m, m, MORPH_ERODE, element);
	element = getStructuringElement(MORPH_ELLIPSE, Size(20, 25));
	morphologyEx(m, m, MORPH_DILATE, element);
	// ��䲢����
	cvtColor(m, m, CV_GRAY2BGR);
	if (pixelX>0 && pixelY>0)
		floodFill(m, Point(pixelX, pixelY), Scalar(192, 255, 100));
	//imshow("m,", m);
	vector<Mat>channels;
	split(m, channels);
	m = channels.at(1) - channels.at(0);
	// ��ֵ�����������
	threshold(m, m, 1, 255, CV_THRESH_BINARY);
	Point gp = GravityCenter(m);
	cvtColor(m, m, CV_GRAY2BGR);
	circle(m, gp, 5, Scalar(0, 255, 0), 5, 8);

	if (k2.Coordinate.at(2).at<float>(gp.y, gp.x) > 0)
	{
		BodyXYZ.X = k2.Coordinate.at(0).at<float>(gp.y, gp.x);
		BodyXYZ.Y = k2.Coordinate.at(1).at<float>(gp.y, gp.x);
		BodyXYZ.Z = k2.Coordinate.at(2).at<float>(gp.y, gp.x);
	}
}
Point GravityCenter(Mat src)
{
	Point p;
	IplImage s = src;
	double m00, m10, m01;
	CvMoments moment;
	cvMoments(&s, &moment, 1);
	m00 = cvGetSpatialMoment(&moment, 0, 0);
	if (m00 == 0)
	{
		p.x = -1;
		p.y = -1;
		return p;
	}
	m10 = cvGetSpatialMoment(&moment, 1, 0);
	m01 = cvGetSpatialMoment(&moment, 0, 1);
	p.x = (int)(m10 / m00);
	p.y = (int)(m01 / m00);
	return p;
}

void SetSpeed(unsigned short m_TanslationSpeed, short m_Yaw, short m_RotationAngleSpeed)
{
	UCHAR sum = 0;
	Send_Buff[0] = 0x5A;
	Send_Buff[1] = 0x5A;
	Send_Buff[2] = 0x03;
	Send_Buff[3] = 0x08;
	Send_Buff[4] = 0x01;
	Send_Buff[5] = *(UCHAR*)(&m_TanslationSpeed);
	Send_Buff[6] = *((UCHAR*)(&m_TanslationSpeed) + 1);
	Send_Buff[7] = *(UCHAR*)(&m_Yaw);
	Send_Buff[8] = *((UCHAR*)(&m_Yaw) + 1);
	Send_Buff[9] = *(UCHAR*)(&m_RotationAngleSpeed);
	Send_Buff[10] = *((UCHAR*)(&m_RotationAngleSpeed) + 1);

	sum = 0x03 + 0x08 + 0x01 + *(UCHAR*)(&m_TanslationSpeed) + *((UCHAR*)(&m_TanslationSpeed) + 1)
		+ *(UCHAR*)(&m_Yaw) + *((UCHAR*)(&m_Yaw) + 1)
		+ *(UCHAR*)(&m_RotationAngleSpeed) + *((UCHAR*)(&m_RotationAngleSpeed) + 1);
	Send_Buff[11] = sum;
	//������ڵ����в���							   
	PurgeComm(m_Comm1.hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);//����PurgeComm����������ֹ���ڽ��еĶ�д�������ú�������������������������е����ݡ�
	if (m_Comm1.WriteChar(Send_Buff, 12)) {}
	else { MessageBox(NULL, L"����ʧ��", L"����", 1); }
}