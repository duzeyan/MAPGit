
#ifndef _NJUST_MAP_APP_H_
#define _NJUST_MAP_APP_H_


#include<vector>
#include<string.h>
#include<string>
#include<algorithm>
#include<errno.h>
#include<cmath>
#include<limits>
#include<stdio.h>

#include "NJUST_ALV_BYD_H/ALV_BYD.h"
#include"MAP_BASIC_data.h" //�������ֽڶ���Ķ���
#include"MapCommunion.h"
#include"MapFileStream.h"
#include"MapTools.h"

using std::vector;
using std::string;

///��֪BUG
///1.��ʼ����������յ㣬���³�����ʱ��λ���յ㣬�ڵ�һ�ν���pathplaning�����Խ�����
///2.�����ڵ�һ��·�Ͼ�Ҫ���ع滮���ᵼ�¶δ���(��wholePath����)

//ģ������ģʽ
enum NJUST_MAP_WORK_MODEL
{
	MAP_WORK_NORMAL = 0x00,                  //��ʻģʽ
	MAP_WORK_BACKDRAW ,           			 //����
	MAP_WORK_REPLAN  ,                       //�ع滮
	MAP_WORK_SHEARRIGHT,                     //�����滮·���еĺ���λ��
	MAP_WORK_TASK							 //�����ģʽ���ҵ�
};

#ifdef _NJUST_OS_USE_LINUX_
//�߳���,ͬ����������
extern pthread_mutex_t     gMutex ;
extern pthread_cond_t      cond ;
#endif



// Mapģ�����ࣨҵ����㷨��
class MapApp{
private:
	MapFileStream* _mapFile;			   //�ļ���
	bool _exitFlag;						   //MAPģ�������־
	bool _checkExpOut;					   //����Ƿ��쳣�˳�
	NJUST_MAP_BUILD_MAP _map;			   //�Խ���ͼ��Ϣ
	vector<MAP_TASK_NODE_ZZ> _mapTaskNode;	   //�����ļ��и�����·����ת�����Խ���ͼ�еġ�·�ڱ�����С�
	NJUST_PLAN_PATH _planPath;			   //�滮��·�� ���
	vector<int> _historyPath;			   //�����߹��Ĺ滮·����¼ ���
	vector<MAP_DOUBLE_POINT> _GPSList;     //��ǰ��·����·�ڵ�GPS���� ��λ�Ƿ�
	int _frameNum;					       //MAP��ͳ��֡��(pl,mo���ܣ�
	FILE *_pRecord;					       //�ļ�ָ��(״̬�ļ���)
	int _nCurToMapCmdID;                   //����PL�������


public :
	static NJUST_MAP_GPS_INFO s_GPSInfo;		 //MC���͹�����GPS��Ϣ
	static MAP_PACKAGE s_mapPackage;			//���MO״̬�� ��Ҫ�Ƕ�����
	static NJUST_MAP_WORK_MODEL s_WorkModel;       //MAP����ģʽ

public:  
	
	//************************************
	// ���������캯��
	// ����ֵ: ��
	// ����:   const char * loadpath ��ͼ��ϢĿ¼
	//************************************
	MapApp();
	~MapApp();

	
	//************************************
	// ����������MAPģ�� �滮��������ָ��Ŀ¼��
	// ����ֵ: void
	//************************************
	void run();

	//************************************
	// ������ģ������MAPģ��,���ɽ�������ض��ļ���
	// ����ֵ: void
	//************************************
	void simulate();

	//************************************
	// ����������������ض��ļ���
	// ����ֵ: void
	//************************************
	void simOutResult(const vector<MAP_DOUBLE_POINT> &list);

	//************************************
	// ������ģ������MAPģ��,���ɽ�������ض��ļ���
	// ����ֵ: void
	//************************************
	void simMakeGPS(double lng,double lat,vector<MAP_DOUBLE_POINT> &GPSList);
	
	//************************************
	// ��������ʼ������ 
	// ����ֵ: void
	// ����:   const char * loadpath  ��ͼ��ϢĿ¼
	//************************************
	void initalize(const char* loadpath);

	
	//************************************
	// ����������PL�Ļص���������Ҫ����PL����
	// ����ֵ: int
	// ����:   void * pl_to_map ���ݰ�
	// ����:   size_t size      ��С
	// ����:   void * args      �������
	//************************************
#ifdef _USING_NEW_IP_COMM
	i32 static PLCallBack(void* pl_to_map, u32 size, void* args);
#endif
#ifdef _USING_OLD_IP_COMM
	int static PLCallBack(void* pl_to_map, size_t size, void* args);
#endif
	



	//************************************
	// ����������MC�Ļص���������Ҫ�洢MC������GPS��Ϣ
	// ����ֵ: int
	// ����:   void * pl_to_map ���ݰ�
	// ����:   size_t size      ��С
	// ����:   void * args      �������
	//************************************
#ifdef _USING_NEW_IP_COMM
	i32 static MCCallBack(void* mc_to_map, u32 size, void* args);
#endif
#ifdef _USING_OLD_IP_COMM
	int static MCCallBack(void* mc_to_map, size_t size, void* args);
#endif

	//************************************
	// ����������MO�Ļص���������Ҫ������ӦMO�Ŀ�����Ϣ
	// ����ֵ: int
	// ����:   void * pl_to_map ���ݰ�
	// ����:   size_t size      ��С
	// ����:   void * args      �������
	//************************************
#ifdef _USING_NEW_IP_COMM
	i32 static MOCallBack(void* mo_to_map, u32 size, void* args);
#endif
#ifdef _USING_OLD_IP_COMM
	int static MOCallBack(void* mo_to_map, size_t size, void* args);
#endif
	
	//************************************
	// �������滮·��(ѡȡ�Խ���ͼ�еĽڵ�)������Ϊ1,4�Ľڵ� 
	//		 ��滮�ṹ��:[1],1,[2],2,[3],3,[4] 
	//       ����[]��Ϊ·�ڱ��,ʣ�µ�Ϊ��·���
	//       �������_planPath��
	// ����ֵ: void
	// ����:   int s �������ڵ�ı��
	// ����:   int e �յ㴦�ڵ���
	//************************************
	void pathPlaning(int s,int e);

	
	//************************************
	// ������  ������ʱ,��λ�������ļ��е�·�ڴ�
	// ����ֵ: void
	// ����:   int & srart [out]���ض�λ���
	//************************************
	void startPlan(int& srart);

	
	//************************************
	// ������  ��Dijkstra�㷨�ҳ��Խ���ͼ��s·�ڵ�e·�ڵ����·��
	// ����ֵ: void
	// ����:   int s ��ʼ·�ڵı��
	// ����:   int e �յ�·�ڵı��
	// ����:   vector<int> & [out]�滮���Ľڵ�����,����Ϊ���
	//************************************
	void dijkstra(int s,int e,vector<int>&);

	
	//************************************
	// ������ �ڳ�����ʱ,ͨ����ǰ��γ�ȼ���������Խ���ͼ�е�λ��
	// ����ֵ: int  ID   ��λ����IDֵ ���ȷ��ؽڵ�ID,���Ϊ��·ID
	// ����:   double lng ���ȣ���λ��
	// ����:   double lat ά�ȣ���λ��
	//************************************
	int location(double lng,double lat);

	
	//************************************
	// ������ �жϾ�γ���Ƿ����ĳ����·������(�ϱ�����)��
	// ����ֵ: bool   true��ʾ����
	// ����:   double lng ����,��λ��
	// ����:   double lat ά��,��λ��
	// ����:   int lineID ��·ID
	//************************************
	bool isInLine(double lng,double lat,int lineID);

	//
	//************************************
	// ���������ݵ�ǰGPSֵ,����GPS�����е�����
	// ����ֵ: int index   _GPSList GPS��������
	// ����:   double lng ����,��λ��
	// ����:   double lat ά��,��λ��
	//************************************
	int locationGPS(double lng,double lat);

	
	//************************************
	// ������  �㲥��·��Ϣ
	// ����ֵ: void 
	// ����:   double lng ����,��λ��
	// ����:   double lat ά��,��λ��
	// ����:   int curID  ��ǰ��·ID ��·x
	// ����:   int nextIndex ��·x�յ��·������
	//************************************
	void sendRoad(double lng,double lat,int curID,int nextIndex);

	
	//************************************
	// ������  �㲥·����Ϣ   
	// ����ֵ: void
	// ����:   double lng ����,��λ��
	// ����:   double lat ά��,��λ��
	// ����:   int curID   ��·��ID
	// ����:   int lastIndex  ��һ��·������
	// ����:   int nextIndex  ��һ��·������
	//************************************
	void sendNode(double lng,double lat,int curID,int lastIndex,int nextIndex);

	
	//************************************
	// ������  ��MO����״̬����
	// ����ֵ: void
	// ����:   char buff[] ����������
	// ����:   int n       ���ݳ���
	//************************************
	void send2Mo();

	void sendPLCmd(NJUST_PL_MAP_COMMAND_TYPE cmdType);

	
	//************************************
	// ������ ����·�ڵ�ת�䷽��
	// ����ֵ: void
	// ����:   NJUST_MAP_INFO_NODE & node
	// ����:   int curIndex  ��ǰ�ڵ�ID
	// ����:   int lastIndex ����ڵ�ID
	// ����:   int nextIndex ��һ��Ҫ·���ڵ�ID
	//************************************
	void getDirection(NJUST_MAP_INFO_NODE &node,int curIndex,int lastIndex,int nextIndex);

	
	//************************************
	// ������  ���location�Ƿ����,�������������־
	// ����ֵ: bool		�Ƿ����
	// ����:   int ID   location�������ID
	//************************************
	bool checkLoaction(int ID);

	
	//************************************
	// ������  д��ǰ״̬
	// ����ֵ: void
	// ����:   int curID   ��ǰID
	// ����:   int lastID  �վ���·�ڵ�ID
	// ����:   int sIndex  ·������
	//************************************
	void recordWrite(int curID,int lastID,int sIndex);

	//************************************
	// ������  ���ϴ������˳���״̬
	// ����ֵ: void
	// ����:   int & curID ��ȡ�洢�ĵ�ǰID
	// ����:   int & lastID ��ȡ�洢��ǰID
	// ����:   int & sIndex ·������
	//************************************
	void recordRead(int &curID,int &lastID,int &sIndex);


	//************************************
	// ������  ��������ģ��,Ĭ���ڹ���ģʽ�ڸ�ģ���� 
	// ����ֵ: bool ������ģʽ���е��յ�,�򷵻�true 
	// ����:   double curLng ��ǰ���� ��
	// ����:   double curLat ��ǰά�� ��
	// ����:   int curID     ��ǰ·��(�ڵ�)ID
	// ����:   int & lastID  ��һ�ξ���·��(�ڵ�)ID
	// ����:   int & errTime ��λ�������ͳ��
	//************************************
	bool processRunMoudle(double curLng,double curLat,int &curID,int &lastID,int &lastNode,int &sIndex,int &errTime);
	
	//************************************
	// ������  ����ģʽ
	// ����ֵ: bool �Ƿ��˳���ģʽ
	// ����:   double curLng ��ǰά�� ��
	// ����:   double curLat ��ǰ���� ��
	//************************************
	bool processDriveBack(double curLng,double curLat);

	//************************************
	// ������  ����ֹͣ,�ع滮·��
	// ����ֵ: void
	// ����:   int sIndex ����ڵ�����
	//************************************
	void processRePlan(int sIndex);

	
	//************************************
	// ������  ��鵱ǰλ���Ƿ�������㴦
	// ����ֵ: bool �Ƿ���ʹ�������
	// ����:   double lng ���� ��
	// ����:   double lat ά�� ��
	// ����:   double dis ������ֵ ��λM
	//************************************
	bool locationTaskPoint(double lng,double lat,double dis);


	//************************************
	// ������  �ͷų����ڴ�
	// ����ֵ: void
	//************************************
	void release(); 
};

#endif



