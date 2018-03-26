// Opencv头文件
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
// 基础头文件
#pragma region 
#include <iostream>
#include <ctype.h>
#include <stdio.h>
#include <conio.h>
#include <float.h>
#include <math.h>
// windows头文件
#include <windows.h>
// 模块头文件
#include "KV2_Grabber.h"
#include "USART.h"
// 命名空间
using namespace cv;
using namespace std;
// 串行通信声明
Comm USART;
// 底盘移动参数
unsigned short Speed = 0;
short Yaw = 0;
short AngleRate = 0;
// 激活状态变量
bool TraceEnabled = false;
bool bodyIsReady = false;
// Z差值
float Zdiff = 0, Zlast = 1;
// Kinect声明	// knct
KinectV2_SG k2;
// 人物坐标;
CameraSpacePoint BodyXYZ;
// 分割后重心
Point GravityCenter(Mat src);

// 调试用
Mat frame;
bool automated = 0;
bool NOJoints = false;
// 函数声明
BYTE move_Mecanum(UINT16 Gspeed, INT Angle, CHAR TState);
void TraceStop();
void Trace();
void SetSpeed(unsigned short speed, short yaw, short anglerate);
Comm m_Comm1;
BYTE Send_Buff[10] = { "1234" };
int main(int argc, char** argv)
{
	
	// 通信初始化
start:  // init
	m_Comm1.openport(L"COM3", 115200);


	// 人物跟随模块
	for (;;)
	{
		// 数据更新
		k2.Update_DepthFrame();
		k2.Update_IRFrame();
		k2.Map_CameraSpaceIR();
		frame = k2.FrameIR;
		k2.Update_Body();
		// 自动跟随
		if (!TraceEnabled)										// 如果没有激活
		{ 
			if (k2.ClosestBodyID >= 0)
			{

				cout << ".";
			}
			// 检测动作:
			if (k2.JointData[k2.ClosestBodyID][JointType_HandRight].Position.Y > k2.JointData[k2.ClosestBodyID][JointType_ElbowRight].Position.Y - 0.1&&
				k2.JointData[k2.ClosestBodyID][JointType_HandRight].Position.X > k2.JointData[k2.ClosestBodyID][JointType_ElbowLeft].Position.X&&
				k2.JointData[k2.ClosestBodyID][JointType_HandRight].Position.X < k2.JointData[k2.ClosestBodyID][JointType_ElbowRight].Position.X)
			{													// 手放在胸前高度，对机子张开手再握拳即可激活
				if (k2.HandData[k2.ClosestBodyID].HandRight == HandState_Open)
				{
					bodyIsReady = true;							// 准备
					cout << "ready...";
				}
				else
				{
					if (bodyIsReady&&k2.HandData[k2.ClosestBodyID].HandRight == HandState_Closed)
					{
						TraceEnabled = true;					// 激活
						cout << "OK";
					}
					else
					{
						bodyIsReady = false;
					}
				}
			}
			TraceStop();										// 停止移动
		}
		else
		{
			// 检测是否有停止手势	
			if (k2.JointData[k2.ClosestBodyID][JointType_HandRight].Position.X < k2.JointData[k2.ClosestBodyID][JointType_HandLeft].Position.X&&
				k2.JointData[k2.ClosestBodyID][JointType_HandRight].Position.Y > k2.JointData[k2.ClosestBodyID][JointType_ElbowRight].Position.Y)
			{
				cout << "done";
				TraceEnabled = false;
				TraceStop();
				continue;										// 有则停下并等待激活
			}
			else												// 没有则更新人物坐标
			{
				Zlast = BodyXYZ.Z;
				if (k2.ClosestBodyID < 0 || NOJoints)			// 如果没骨架
				{
					Trace();									// 根据深度分割得出位置
				}
				else											// 如果有骨架
				{
					DepthSpacePoint p;							// 映射关节坐标到深度帧
					k2.pCoordinateMapper->MapCameraPointToDepthSpace(k2.JointData[k2.ClosestBodyID][JointType_SpineShoulder].Position, &p);
					int pixelX = (p.X + 0.5f);
					int pixelY = (p.Y + 0.5f);
					if (pixelX>0 && pixelY>0)					// 统一为相机空间坐标
					{
						BodyXYZ.X = k2.Coordinate.at(0).at<float>(pixelY, pixelX);
						BodyXYZ.Y = k2.Coordinate.at(1).at<float>(pixelY, pixelX);
						BodyXYZ.Z = k2.Coordinate.at(2).at<float>(pixelY, pixelX);
					}
				}
				Zdiff = BodyXYZ.Z - Zlast;
			}
			// 底盘移动
			Zdiff = BodyXYZ.Z - Zlast;
			if (Zdiff >= 0.19)	// 防止突然跟丢
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
				Yaw = 0;	// 跟人
				cout << (Speed = abs(distance * 420)) << endl;
			}
			else
			{
				Yaw = 180;	// 往回走
				cout << (Speed = abs(distance * 1000)) << endl;
			}

			cout << (Speed = abs(distance * 420)) << endl;
			AngleRate = -(short)(x * 100);//k=500

			SetSpeed(Speed, Yaw, AngleRate);//speed 平移速度||yaw 航向角||anglerate 自旋速度

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
底盘通信函数：(单发送)
参数：----------------------------------------------------------------------------
Tspeed	translation speed	平移速度，与旋转速度无关，运动时两者可直接叠加
Dangle	drift angle			平移偏角，以正前方为0°，顺时针为正，可为负值
Rspeed	rotating speed		自转速度，正值为顺时针方向，负值则逆时针方向
数据包：--------------------------------------------------------------------------
"[" + 控制字符 + Tspeed + Dangle + Rspeed + Checksum + "]"
备注： 5字节	 先低位后高位发送  不是负值		|
---------------------------------校验和与除自己外所有数据相加
************************************************************************************/


/**************************************************************************************************************************
流程：
1首先根据骨架位置跟踪，对于识别到手距离脊柱距离大于2分米且手掌张开的停下，那个骨架id变成无效的
直到骨架丢失或者刚才那个姿势握拳再变成有效的
这段时间一直是跟随最近的骨架
2当骨架因底盘移动而丢失的时候，那个还在原处，检测一下当时丢失的点附近（约为1平方分米的矩形）的深度值是否和骨架丢失前的Z值相差不多，骨架靠后一些，相差不远于骨架0.15进于骨架0.25
此处要加上是否是真的骨架的判断，当Y过高过低那除非人趴地上或者飞起来了才会这样
3直接对当前点所在面的边界进行判断，找出中心，X和Z解决
4根据语音骨架动作来判断是否停止跟踪
5所有计算用的数据不能有残留或者有调用残留数据的可能
备注：
ok 1在深度上对Z轴距离大于3.5m的就不管了（或者慢慢变到一个较慢的速度走，当z更远的时候停下来，比较贴合实际体验）
ok 2深度帧30FPS除非供电卡了或者算法慢是不会跳帧严重的，理论上加速度较大也能跟，没有拖影，对光照没限制，但是怕反光衣服
todo 3如果人离得不远，在发现其他骨架或者有不远的宽度大于0.1m的孤立面（可能是门或者人），让头转向所在的大致位置"看一眼"
如果发现骨架，把脸取出来看看是否是已经认识的人，认识就打招呼（打招呼要设间隔，1分钟内不再打）
如果发现孤立面，抬头看一眼,如果头部识别有人脸，打招呼，
没发现或置信度不足就平视再看回跟踪的人的脖子位置
todo 4转头看脸:X和Z决定角度，Y为测试好的静态值，Yaw加速度大，Pitch加速度小
2015-08-16 01:29:23
**************************************************************************************************************************/

/*
跟随用的骨架点
k2.pCoordinateMapper->MapCameraPointToDepthSpace(BodyXYZ, &p);
读	vector<Mat> Coordinate;
split(k2.CameraSpaceIR, Coordinate);// 分离出XYZ轴 at （0 1 2）
Coordinate.at(2).at<float>(pixelX, pixelY));
*/

/*************************************************************************************************************************
/*	类型：	void
/*	函数：	Tracing()
/*	用途：	当骨架丢失切换到这个函数来继续更新坐标
/*	注意：	一定是一个有效的坐标，
/*************************************************************************************************************************/
#define Z_DiffThreshold 0.2
void Trace()
{
	// 真实世界空间坐标映射到坐标矩阵的像素位置
	DepthSpacePoint p;
	Mat m;
	k2.pCoordinateMapper->MapCameraPointToDepthSpace(BodyXYZ, &p);
	int pixelX = (p.X + 0.5f);
	int pixelY = (p.Y + 0.5f);
	k2.Coordinate.at(2).copyTo(m);//把Z通道拷贝下来进行处理
								  //Zcur是当前人的Z深，min为朝近处浮动的Z深阈值，max为远处。
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
	//2进行形态学操作  
	//定义核  
	Mat element;
	// 进行膨胀腐蚀
	element = getStructuringElement(MORPH_ELLIPSE, Size(10, 10));
	morphologyEx(m, m, MORPH_ERODE, element);
	morphologyEx(m, m, MORPH_DILATE, element);
	element = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
	morphologyEx(m, m, MORPH_ERODE, element);
	element = getStructuringElement(MORPH_ELLIPSE, Size(20, 25));
	morphologyEx(m, m, MORPH_DILATE, element);
	// 填充并分离
	cvtColor(m, m, CV_GRAY2BGR);
	if (pixelX>0 && pixelY>0)
		floodFill(m, Point(pixelX, pixelY), Scalar(192, 255, 100));
	//imshow("m,", m);
	vector<Mat>channels;
	split(m, channels);
	m = channels.at(1) - channels.at(0);
	// 二值化并求出重心
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
	//清除串口的所有操作							   
	PurgeComm(m_Comm1.hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);//调用PurgeComm函数可以终止正在进行的读写操作，该函数还会清除输入或输出缓冲区中的内容。
	if (m_Comm1.WriteChar(Send_Buff, 12)) {}
	else { MessageBox(NULL, L"发送失败", L"错误", 1); }
}