#ifndef KV2_GRABBER
#define KV2_GRABBER

#include "kinect.h" 
#include "kinect.face.h" 
#include <opencv2/opencv.hpp>
//#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace cv;
//using namespace pcl;

template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL) {
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

//struct FaceData {
//	// 是否被跟踪
//	BOOL            tracked;
//	// 面部外框
//	RectI           faceBox;
//	// 面部特征点
//	PointF          facePoints[FacePointType::FacePointType_Count];
//	// 面部旋转四元数
//	Vector4         faceRotation;
//	// 面部相关属性
//	DetectionResult faceProperties[FaceProperty::FaceProperty_Count];
//};
struct HandData {
	// 左手状态
	HandState HandLeft;
	// 右手状态
	HandState HandRight;
};
class KinectV2_SG
{
public:
	KinectV2_SG();											// 进入作用域时初始化
	~KinectV2_SG();											// 离开作用域时清理	
#pragma region 函数
	void								Update_BGRFrame(bool EnhanceGamma = false);					// 更新BGR图像帧
	void 								Update_DepthFrame(int flag = 0);;							// 更新深度帧
	void 								Update_IRFrame(int flag = 0);								// 更新IR图像帧
	void 								Update_BodyIndexFrame();									// 更新索引帧
	void 								Update_Body(bool FaceTracing = false);						// 更新骨架与2D面部
	void								Map_CameraSpaceBGR();										// 更新BGR镜头到真实世界坐标映射
	void								Map_CameraSpaceIR();										// 更新IR镜头到真实世界坐标映射
#pragma endregion 		
	//坐标映射
	ICoordinateMapper*					pCoordinateMapper;											// 坐标映射器
	void test(bool wtf);
	// 图像与上面函数对应
	Mat									FrameBGR;												// BGR图像矩阵
	Mat									FrameDepth;												// 深度图像矩阵
	Mat									FrameIR;												// IR图像矩阵
	Mat									CameraSpaceBGR;											// BGR帧到真实世界坐标映射矩阵
	Mat									CameraSpaceIR;											// IR帧到真实世界坐标映射矩阵
	int									ClosestBodyID;											// 被选中的ID
	Joint								JointData[BODY_COUNT][25];								// 关节信息
	HandData							HandData[BODY_COUNT];									// 手部信息
	/*TODO:		整合上面三个为一个		BodyData[BODY_COUNT];										// 骨骼信息	*/
	vector<Mat>							Coordinate;		// 分离的相机空间坐标系XYZ

																								//UINT16* buffer = new UINT16[217088];
																								//CameraSpacePoint* bufferc = new CameraSpacePoint[8294400];
private:
	// 基本变量：
	IKinectSensor*						KinectSensor;											// Kinect设备接口
	BOOLEAN								bIsOpen = 0;											// 是否打开
	HRESULT								hr;														// 句柄结果
																								// 彩色图：
	IColorFrameSource*					pColorSource;											// 彩色帧源
	IColorFrameReader*					pColorReader;											// 彩色读取器
	IColorFrame*						pColorFrame;											// 彩色帧
																								// 红外图：
	ILongExposureInfraredFrameSource*	pInfraredSource;										// 红外帧源
	ILongExposureInfraredFrameReader*	pInfraredReader;										// 红外读取器
	ILongExposureInfraredFrame*			pInfraredFrame;											// 红外帧
																								// 深度图：
	IDepthFrameSource*					pDepthSource;											// 红外帧源
	IDepthFrameReader*					pDepthReader;											// 红外读取器
	IDepthFrame*						pDepthFrame;											// 红外帧
																								// 身体索引：
	IBodyIndexFrameSource*				pBodyIndexSource;										// 索引帧源
	IBodyIndexFrameReader*				pBodyIndexReader;										// 索引读取器
	IBodyIndexFrame*					pBodyIndexFrame;										// 索引帧
																								// 骨架：
	IBodyFrameSource*					pBodySource;
	IBodyFrameReader*					pBodyReader;											// 索引读取器
	IBodyFrame*							pBodyFrame;												// 骨骼帧
	IBody*								pBodies[BODY_COUNT];									// 骨骼数据

	unsigned char						lut[256];												// 彩色帧Gamma矫正

};

//0		停止更新
//1		BGR
//2		IR
//4		Deph
//8		Body Index
//16	Body Joint
//32	CS_IR
//64	CS_BGR
//128	别的映射器？


#endif