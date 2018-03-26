#include "kinect.h"  
#include "kinect.face.h" 
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"  
#include "KV2_Grabber.h"
using namespace std;
using namespace cv;
//using namespace pcl;

//HRESULT WINAPI CreateFaceFrameSource(_In_ IKinectSensor* sensor, _In_ const UINT64 initialTrackingId, _In_ const DWORD initialFaceFrameFeatures, _COM_Outptr_ IFaceFrameSource** ppSource);

KinectV2_SG::KinectV2_SG()
{
	//矩阵初始化区域
	FrameBGR.create(1080, 1920, CV_8UC3);//初始化BGR帧
	FrameDepth.create(424, 512, CV_16UC1);//初始化IR帧
	FrameIR.create(424, 512, CV_16UC1);//初始化IR帧
	CameraSpaceBGR.create(1080, 1920, CV_32FC3);//初始化BGR帧到真实世界坐标映射
	CameraSpaceIR.create(424, 512, CV_32FC3);//初始化BGR帧到真实世界坐标映射
											 //	pointcloud(new PointCloud<PointXYZRGB>();
	for (int i = 0; i < BODY_COUNT; i++)
	{
		pBodies[i] = { 0 };
	}
	//设备初始化区域
	cout << "Kinect V2 Initialization Start... ";
	if (FAILED(GetDefaultKinectSensor(&KinectSensor)))
	{
		cout << "Failed" << endl;
		cout << "Check Power supply and USB connection" << endl;
		cout << "Press any key to terminate..." << endl;
		cin.get();
		return;
	}
	KinectSensor->get_IsOpen(&bIsOpen);
	if (!bIsOpen)
	{
		if (FAILED(KinectSensor->Open()))
		{
			cout << "Failed" << endl;
			cout << "Check Kinect Service and Working conditions" << endl;
			cout << "Press any key to terminate..." << endl;
			cin.get();
			return;
		}
	}
	//流接口初始化区域
	/*彩色源*/ hr = KinectSensor->get_ColorFrameSource(&pColorSource);
	if (FAILED(hr))
	{
		cerr << "Error : IKinectSensor::get_ColorFrameSource()" << endl;
	}
	hr = pColorSource->OpenReader(&pColorReader);
	if (FAILED(hr))
	{
		cerr << "Error : IColorFrameSource::OpenReader()" << endl;
	}
	/*红外源*/ hr = KinectSensor->get_LongExposureInfraredFrameSource(&pInfraredSource);
	if (FAILED(hr))
	{
		cerr << "Error : IKinectSensor::get_InfraredFrameSource()" << endl;
	}
	hr = pInfraredSource->OpenReader(&pInfraredReader);
	/*深度源*/ hr = KinectSensor->get_DepthFrameSource(&pDepthSource);
	if (FAILED(hr))
	{
		cerr << "Error : IKinectSensor::get_DepthFrameSource()" << endl;
	}
	hr = pDepthSource->OpenReader(&pDepthReader);
	if (FAILED(hr))
	{
		cerr << "Error : IDepthFrameSource::OpenReader()" << endl;
	}
	/*索引源*/

	//...

	/*骨骼帧源*/hr = KinectSensor->get_BodyFrameSource(&pBodySource);
	if (FAILED(hr))
	{
		cerr << "Error : IKinectSensor::get_BodyFrameSource()" << endl;
	}
	hr = pBodySource->OpenReader(&pBodyReader);
	if (FAILED(hr))
	{
		cerr << "Error : IBodyFrameSource::OpenReader()" << endl;
	}
	/*坐标映射器*/hr = KinectSensor->get_CoordinateMapper(&pCoordinateMapper);
	if (FAILED(hr))
	{
		cerr << "Error : IKinectSensor::get_CoordinateMapper()" << endl;
	}
	//附加区域
	/*Gamma纠正*/
	for (int i = 0; i < 256; i += 1)
	{
		lut[i] = saturate_cast<uchar>(pow((float)(i / 255.0), 0.2) * 255.0f);
	}
	//结束
	cout << "SUCCESS" << endl;
}

KinectV2_SG::~KinectV2_SG()
{

}

void KinectV2_SG::Update_IRFrame(int flag)
{
	/*更新红外帧*/hr = pInfraredReader->AcquireLatestFrame(&pInfraredFrame);
	if (SUCCEEDED(hr))
	{
		Mat buffer_IRMat(424, 512, CV_16UC1);//IR16
		hr = pInfraredFrame->CopyFrameDataToArray(217088, reinterpret_cast<UINT16*>(buffer_IRMat.data));
		if (SUCCEEDED(hr))
		{
			if (flag == 0)
			{
				buffer_IRMat.convertTo(buffer_IRMat, CV_64F, 1);
				normalize(buffer_IRMat, buffer_IRMat, 0, 65535, CV_MINMAX);
				buffer_IRMat += 2;
				log(buffer_IRMat, buffer_IRMat);
				normalize(buffer_IRMat, buffer_IRMat, -0, 255, CV_MINMAX);
				buffer_IRMat.convertTo(FrameIR, CV_8U, 1); //FrameIR -= 32;
														   //equalizeHist(FrameIR, FrameIR);
			}
			else
			{
				buffer_IRMat.convertTo(buffer_IRMat, CV_64F, 1);
				normalize(buffer_IRMat, buffer_IRMat, 0, 65535, CV_MINMAX);
				buffer_IRMat += 2;
				log(buffer_IRMat, buffer_IRMat);
				normalize(buffer_IRMat, buffer_IRMat, -96, 255, CV_MINMAX);
				buffer_IRMat.convertTo(FrameIR, CV_8U, 1);
				equalizeHist(FrameIR, FrameIR);
			}


		}
	}
	SafeRelease(pInfraredFrame);
}
void KinectV2_SG::Update_BGRFrame(bool EnhanceDarkVision)
{
	/*更新彩色帧*/hr = pColorReader->AcquireLatestFrame(&pColorFrame);
	if (SUCCEEDED(hr))
	{
		Mat buffer_ColorMat(1080, 1920, CV_8UC4);//BGRA
		hr = pColorFrame->CopyConvertedFrameDataToArray(
			8294400,//缓存大小1920 * 1080 * 4 * sizeof(unsigned char)
			reinterpret_cast<BYTE*>(buffer_ColorMat.data),
			ColorImageFormat::ColorImageFormat_Bgra);
		if (SUCCEEDED(hr))
		{
			cvtColor(buffer_ColorMat, FrameBGR, CV_BGRA2BGR);
			if (EnhanceDarkVision)
			{
				Mat orig = FrameBGR.clone();
				MatIterator_<Vec3b> it, end;
				for (it = FrameBGR.begin<Vec3b>(), end = FrameBGR.end<Vec3b>(); it != end; it += 1)
				{
					(*it)[0] = lut[((*it)[0])];
					(*it)[1] = lut[((*it)[1])];
					(*it)[2] = lut[((*it)[2])];
				}
				addWeighted(orig, 0.6, FrameBGR, 0.4, 0, FrameBGR);
			}
		}
	}
	SafeRelease(pColorFrame);
}
void KinectV2_SG::Update_DepthFrame(int flag)
{
	/*更新深度帧*/hr = pDepthReader->AcquireLatestFrame(&pDepthFrame);
	if (SUCCEEDED(hr))
	{
		Mat buffer_DepthMat(424, 512, CV_16UC1);//Depth16
		hr = pDepthFrame->CopyFrameDataToArray(217088, reinterpret_cast<UINT16*>(buffer_DepthMat.data));
		if (SUCCEEDED(hr))
		{
			FrameDepth = buffer_DepthMat.clone();
		}
	}
	SafeRelease(pDepthFrame);
}
void KinectV2_SG::Update_Body(bool FaceTracing)
{
	// 获取骨骼帧
	hr = pBodyReader->AcquireLatestFrame(&pBodyFrame);
	if (SUCCEEDED(hr))
	{
		// 更新骨骼数据
		hr = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, pBodies);
		if (SUCCEEDED(hr))
		{
			float ReferZ = 4500;
			bool Tracking = 0;
			for (int count = 0; count < BODY_COUNT; count += 1)
			{
				BOOLEAN Tracked = 1;
				hr = pBodies[count]->get_IsTracked(&Tracked);
				if (SUCCEEDED(hr) && Tracked)
				{
					Joint joint[JointType_Count];//获取关节信息
					hr = pBodies[count]->GetJoints(JointType_Count, joint);
					if (hr == S_OK)
					{
						for (int i = 0; i < 25; i++)
						{
							JointData[count][i] = joint[i];
						}
						float NowZ = joint[JointType_SpineMid].Position.Z;
						if (NowZ < ReferZ)
						{
							ClosestBodyID = count;
							Tracking = 1;
						}

						ReferZ = NowZ;
					}
					pBodies[count]->get_HandRightState(&HandData[count].HandRight);
					pBodies[count]->get_HandLeftState(&HandData[count].HandLeft);
				}
			}
			if (!Tracking)
			{
				ClosestBodyID = -1;
			}
			SafeRelease(pBodyFrame);
			//			// 释放掉
		}
	}
	for (int count = 0; count < BODY_COUNT; count += 1)
	{
		SafeRelease(pBodies[count]);
	}
}
void  KinectV2_SG::Map_CameraSpaceBGR()
{
	Point point;
	pCoordinateMapper->MapColorFrameToCameraSpace(217088, reinterpret_cast<UINT16*>(FrameDepth.data), 2073600, reinterpret_cast<CameraSpacePoint*>(CameraSpaceBGR.data));
	if (true)
	{
		CameraSpaceBGR.at<CameraSpacePoint>(point.y, point.x);
	}
}
void  KinectV2_SG::Map_CameraSpaceIR()
{
	Point point;
	pCoordinateMapper->MapDepthFrameToCameraSpace(217088, reinterpret_cast<UINT16*>(FrameDepth.data), 217088, reinterpret_cast<CameraSpacePoint*>(CameraSpaceIR.data));
	split(CameraSpaceIR, Coordinate);// 分离出XYZ轴 at （0 1 2）
}
