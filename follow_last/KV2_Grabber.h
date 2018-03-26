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
//	// �Ƿ񱻸���
//	BOOL            tracked;
//	// �沿���
//	RectI           faceBox;
//	// �沿������
//	PointF          facePoints[FacePointType::FacePointType_Count];
//	// �沿��ת��Ԫ��
//	Vector4         faceRotation;
//	// �沿�������
//	DetectionResult faceProperties[FaceProperty::FaceProperty_Count];
//};
struct HandData {
	// ����״̬
	HandState HandLeft;
	// ����״̬
	HandState HandRight;
};
class KinectV2_SG
{
public:
	KinectV2_SG();											// ����������ʱ��ʼ��
	~KinectV2_SG();											// �뿪������ʱ����	
#pragma region ����
	void								Update_BGRFrame(bool EnhanceGamma = false);					// ����BGRͼ��֡
	void 								Update_DepthFrame(int flag = 0);;							// �������֡
	void 								Update_IRFrame(int flag = 0);								// ����IRͼ��֡
	void 								Update_BodyIndexFrame();									// ��������֡
	void 								Update_Body(bool FaceTracing = false);						// ���¹Ǽ���2D�沿
	void								Map_CameraSpaceBGR();										// ����BGR��ͷ����ʵ��������ӳ��
	void								Map_CameraSpaceIR();										// ����IR��ͷ����ʵ��������ӳ��
#pragma endregion 		
	//����ӳ��
	ICoordinateMapper*					pCoordinateMapper;											// ����ӳ����
	void test(bool wtf);
	// ͼ�������溯����Ӧ
	Mat									FrameBGR;												// BGRͼ�����
	Mat									FrameDepth;												// ���ͼ�����
	Mat									FrameIR;												// IRͼ�����
	Mat									CameraSpaceBGR;											// BGR֡����ʵ��������ӳ�����
	Mat									CameraSpaceIR;											// IR֡����ʵ��������ӳ�����
	int									ClosestBodyID;											// ��ѡ�е�ID
	Joint								JointData[BODY_COUNT][25];								// �ؽ���Ϣ
	HandData							HandData[BODY_COUNT];									// �ֲ���Ϣ
	/*TODO:		������������Ϊһ��		BodyData[BODY_COUNT];										// ������Ϣ	*/
	vector<Mat>							Coordinate;		// ���������ռ�����ϵXYZ

																								//UINT16* buffer = new UINT16[217088];
																								//CameraSpacePoint* bufferc = new CameraSpacePoint[8294400];
private:
	// ����������
	IKinectSensor*						KinectSensor;											// Kinect�豸�ӿ�
	BOOLEAN								bIsOpen = 0;											// �Ƿ��
	HRESULT								hr;														// ������
																								// ��ɫͼ��
	IColorFrameSource*					pColorSource;											// ��ɫ֡Դ
	IColorFrameReader*					pColorReader;											// ��ɫ��ȡ��
	IColorFrame*						pColorFrame;											// ��ɫ֡
																								// ����ͼ��
	ILongExposureInfraredFrameSource*	pInfraredSource;										// ����֡Դ
	ILongExposureInfraredFrameReader*	pInfraredReader;										// �����ȡ��
	ILongExposureInfraredFrame*			pInfraredFrame;											// ����֡
																								// ���ͼ��
	IDepthFrameSource*					pDepthSource;											// ����֡Դ
	IDepthFrameReader*					pDepthReader;											// �����ȡ��
	IDepthFrame*						pDepthFrame;											// ����֡
																								// ����������
	IBodyIndexFrameSource*				pBodyIndexSource;										// ����֡Դ
	IBodyIndexFrameReader*				pBodyIndexReader;										// ������ȡ��
	IBodyIndexFrame*					pBodyIndexFrame;										// ����֡
																								// �Ǽܣ�
	IBodyFrameSource*					pBodySource;
	IBodyFrameReader*					pBodyReader;											// ������ȡ��
	IBodyFrame*							pBodyFrame;												// ����֡
	IBody*								pBodies[BODY_COUNT];									// ��������

	unsigned char						lut[256];												// ��ɫ֡Gamma����

};

//0		ֹͣ����
//1		BGR
//2		IR
//4		Deph
//8		Body Index
//16	Body Joint
//32	CS_IR
//64	CS_BGR
//128	���ӳ������


#endif