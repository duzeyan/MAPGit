#ifndef _MAPBASICDATA_H_
#define _MAPBASICDATA_H_
//----------------------------------------------------------------------------
#include<iostream>
#include "NJUST_ALV_BYD_H/ALV_BYD.h"
////////////////////////////////////////////////////////////////////////////////////////////////
// 
//  ���ö��뷽ʽ
//
////////////////////////////////////////////////////////////////////////////////////////////////
#pragma pack(push) //�������״̬
#pragma pack(1)    //��1�ֽڶ���,������ڵ�һ��

//#define _USING_NEW_IP_COMM  //ʹ���°汾ͨ��
#define _USING_OLD_IP_COMM  //ʹ�þɰ汾ͨ��

#include<vector>

#include "stdio.h"
using std::vector;
using std::string;

//�����ģ��
#define LINUX_SIM 
//�������GPS����
#define CACHE_GPS_LEN     5
#define INITL_GPS_VALUE   -1
#define INF              1000000000

//�ڵ�ID��ʼֵ
#define START_NODE_ID   10000
//��·DI��ʼֵ
#define START_LINE_ID   20000

//��־ ���
extern FILE *gLOG_OUT;
#define _MAP_LOG 1
#if _MAP_LOG
	#define MAP_PRINT(fmt,v) fprintf(gLOG_OUT,fmt,v);fflush(gLOG_OUT)
#else
    #define MAP_PRINT(fmt,v) 
#endif

extern FILE *gOFFData;
#define _MAP_OFF_DATA_


//debug ���
extern FILE *gDEBUG_OUT;
#define _MAP_DEBUG 1
#if _MAP_DEBUG
#define DEBUG_PRINT(fmt,v)  fprintf(gDEBUG_OUT,fmt,v);fflush(gDEBUG_OUT)
#else
#define DEBUG_PRINT(fmt,v) 
#endif




//�ɼ���ͼ��Ϣʱ ·�ڽṹ
typedef struct MAP_BUTTON_NOTE{
	//�ڵ��Ӧ��ʵ��·�ڵ���Ϣ
	int     neigh;//��֧��
	int     HLD;//���̵�λ��
	int     HLDkind;//���̵�����
	int     lukou;//·��
	int     zebra;//������
	double  gpsx;//gpsx��Ӧ��longtitude
	double  gpsy;//gpsy��Ӧ��latitude
	int     earthx;
	int     earthy;//�������

	//�ڵ�����Ļ�ϵ�λ����Ϣ
	 //����ʱ��֪������ʱ����ֵ

	int    idself;
	 //����ʱ��֪������ʱ����ֵ
	int     NeighNoteID[4];
	int     NeighLineID[4];


}MAP_BUTTON_NOTE;
//----------------------------------------------------------------------------

//�ɼ���ͼ��Ϣʱ ��·�ṹ
typedef struct  MAP_BUTTON_LINE{
		//����Ļ����Ϣ
        //����ʱ����֪������ʱ����ֵ

	int       idself;
    //����ʱ��֪������ʱ����ֵ
	int       idstart;
	int       idend;
	//ֱ��б�ʷ���,�Ի�������Ϊ����ϵ
	//kx+by+c=0
	double        k;
	double        b;
	double        c;

     //ʵ�ʵ�·��Ϣ   
	int     roadkind;//��·����
	float   wedth;//��·���,��λ����
	float   length;//��·����

	int     maluyazi;//��·����
	float   hyazi;//��·���Ӹ߶ȣ���λ����
	int     hulan;//����
	float   hhulan;//�����߶�					
    int     xingdaoxiannum; //�е�����Ŀ
    int     leftxingdaoxian;//���е���
	int     middlexingdaoxian;//�м��е���
	int     rightxingdaoxian;//���е���

	int     chedaonum;//������Ŀ
    int     leftdaolubianjie;//���·�߽�
	int     rightdaolubianjie;//�ҵ�·�߽�
	int     idealspeed;   //�����ٶ�
}MAP_BUTTON_LINE;

//----------------------------------------------------------------------------
//��������Ϊdouble �ĵ��
typedef struct MAP_DOUBLE_POINT{
	double x;
	double y;
}MAP_DOUBLE_POINT;


//---------------------------------------------------����MC��gps��Ϣ�ṹ��
 
typedef struct  MAP_MC_GPS{
	int FrameID;
    double longtitude_degree;
    double latitude_degree;

} MAP_MC_GPS;
//----------------------------------------------------------------------------



//�߳�ͬ���ṹ��
//typedef struct CondForThread
//{
//    pthread_cond_t cond ;
//    pthread_mutex_t mutex ;
//} MAP_THTEAD_MUTEX;

//�Խ���ͼ�ļ�ͷ
 
typedef struct data
{
	int     m_adjustx;       //�Զ�������ϵ���ĵ�
	int     m_adjusty;       //�Զ�������ϵ���ĵ�
	int     notecounter;     //�ڵ���Ŀ
	int     linecounter;     //��·��Ŀ
	int		obstaclecounter; //�ϰ�����Ŀ
}MAP_BUILD_FILE_HEAD;




////�ϰ�������
//typedef struct obstacle
//{
//	NJUST_MAP_GPSPoint    ObstacleCenterGPS; //�ϰ�������λ�õľ�γ��
//	int             RadialCM;            //�뾶����Բ������,��λ:cm
//}NJUST_MAP_OBSTACLE;
//
//enum NJUST_MAP_ROAD_STRUCTURE    //��·�ṹ
//{
//	NJUST_MAP_STRUCTURED_ROAD=0X00, //�ṹ����·
//	NJUST_MAP_HALF_STRUCTURED_ROAD,  //��ṹ����·
//	NJUST_MAP_NON_STRUCTURED_ROAD,   //�ǽṹ����·
//	NJUST_MAP_ROAD_STRUCTURE_TOTAL_NUM           //��β
//};

//enum NJUST_MAP_GPS_SOURCE      //GPS��Ϣ��Դ
//{ 
//	NJUST_MAP_GPS_FROM_CAR=0X00,    //GPS�����ǳ���ʻ�ɼ���
//	NJUST_MAP_GPS_FROM_HAND_DEVICE, //GPS�������ֳ�GPS�豸�ɼ���
//	NJUST_MAP_GPS_FROM_HAND_DRAW,   //GPS�������˹����Ʊ༭��
//	NJUST_MAP_GPS_SOURCE_TOTAL_NUM           //��β
//};

//���㶪����
 
typedef struct MAP_PACKAGE{
	int startID; //��һ�����յ�֡ID
	int endID;	 //���һ�����յ�֡ID
	int count;   //ͳ�ƽ����˶���֡
	NJUST_IP_TIME startTime;//��֡��ʼ����ʱ��
}MAP_PACKAGE;

 
typedef struct node
{
	int     idself;
    int     neigh;//��֧��
	int     NeighNoteID[4]; //Ĭ��һ��·�������ĸ���·����  ����·�����ӵ�·��ID
	int     NeighLineID[4]; //����·�����ӵĵ�·ID  ������Ϊneigh
	int     HLD;//���̵�λ��
	int     HLDkind;//���̵�����
	int     lukou;//·��
	int     zebra;//������
	double  gpsx;//gpsx��Ӧ��longtitude
	double  gpsy;//gpsy��Ӧ��latitude
	int     earthx;
	int     earthy;
	//NJUST_MAP_ROAD_STRUCTURE structure; //�ṹ������
	//NJUST_MAP_GPS_SOURCE    GPSDataFrom;//GPS��Ϣ��Դ
}MAP_NODE;

	

 
typedef struct{
	int id;        //����ID ������ɾ�Ĳ�
	NJUST_MAP_OBSTACLE ob;
	int x;        //��������
	int y;
	double rPix; // ���ؿ��
	bool operator==(const int& id) // �������������
    {
        return this->id==id;
    }
}CREATE_MAP_OBSTACLE;

 
typedef struct line
{
	int       idself;
	int       idstart;
	int       idend;
	//kx+by+c=0
	double        k;
	double        b;
	double        c; 
	int     roadkind;//��·����
	float   wedth;//��·���,��λ����
	float   length;//��·����
	int     maluyazi;//��·����
	float   hyazi;//��·���Ӹ߶ȣ���λ����
	int     hulan;//����
	float   hhulan;//�����߶�					
    int     xingdaoxiannum; //�е�����Ŀ
    int     leftxingdaoxian;//���е���
	int     middlexingdaoxian;//�м��е���
	int     rightxingdaoxian;//���е���
	int     chedaonum;//������Ŀ
    int     leftdaolubianjie;//���·�߽�
	int     rightdaolubianjie;//�ҵ�·�߽�
	int     idealspeed;   //�����ٶ�
	NJUST_MAP_ROAD_STRUCTURE structure; //�ṹ������
	NJUST_MAP_GPS_SOURCE    GPSDataFrom;//GPS��Ϣ��Դ
}MAP_ROAD;

 
typedef struct record
{
	int nodenum;
	int cur;
	int next;
	int a;
	int b;
	int flag;
	int ludian;
}MAP_RECORD_NODE;

 
typedef struct roadfile
{
	int     num;
	double  longtitude;//��Ϊ��λ
	double  latitude;
	double  hight;
	int     shuxing1;
	int     shuxing2;
}RoadFileNode;


enum PROPRITY   //·������
{
	START = 0x00, //���
	STRAIGHT,     //ֱ��
	LEFT,         //���
	RIGHT,        //�ҹ�
	Uturn,        //Uturn
	TRAFIC_SIGN,   //��ͨ��־
	SPECIAL_TASK,  //��������
	END           //��β
};






//�������ļ��и����Ľڵ�
  
typedef struct ROADNODE
{
    int       num;
	double    longtitude;
	double    latitude;               //�Զ�Ϊ��λ
	int       noderesult;            //��·�����������λ���ĵ�,����·���������-1
	int       shuxing1;
	int       shuxing2;
	int       duiyingludianbianhao;  
}MAP_TASK_NODE;

//��װ�����ļ�
typedef struct tagZZNode{
	int num; //ԭ�ļ����
	double    longtitude; //���� ��
	double    latitude;  //ά�� ��
	double    heightM;   //�߶� ��λ��
	int type;//0��� 1�յ� 2·�ϵ� 3��� 4��������
	int resultCode;//����������·��
}MAP_TASK_NODE_ZZ;


//�洢��ǰ�滮��·����Ϣ
typedef struct NJUST_PLAN_PATH{
	vector<int> planPathQueue;			 //���¹滮��·�� �� �ڵ��ţ���·��ţ��ڵ��ţ����������ڵ���
	int cur;						 //��ǰʱ�� ���������λ��(��·or·��)
}NJUST_PLAN_PATH;

//�Խ���ͼ��Ϣ
typedef struct NJUST_MAP_BUILD_MAP{
	MAP_DOUBLE_POINT adjustPoint;		//ת����������ԭ��,��ֹ������� 
	vector<MAP_BUTTON_NOTE>  mapNode;   //�Խ���ͼ�е�����·��
	vector<MAP_BUTTON_LINE>  mapLine;   //�Խ���ͼ�е����е�·
	vector<NJUST_MAP_OBSTACLE> mapObs;  //�Խ���ͼ�е��ϰ���
	vector<int> adjMat;			       //��ͼ�������� ����תһά������ȡ
}NJUST_MAP_BUILD_MAP;

//GPS��Ϣ
 
typedef struct NJUST_MAP_GPS_INFO{
	double curLongtitude;			 //���µľ��� ��λ��
	double curLatitude;				 //���µ�ά��

	double lastLongtitude;			 //GPS���ƽ��ֵ ����
	double lastlatitudel;			 //GPS���ƽ��ֵ ά��
	//double cacheLng[CACHE_GPS_LEN];  //����GPS�� ����
	//double cacheLat[CACHE_GPS_LEN];  //����GPS�� ά��
	//int curOld;                      //�����α� ָ���������
}NJUST_MAP_GPS_INFO;

//���أ�GPS��ת�洢
 
typedef struct  COMPUTE_GPS{
	double lng;//����
	double lat;//ά��
	int x;     //ͼ��x
	int y;	   //ͼ��y
	COMPUTE_GPS():lng(0.0),lat(0.0),x(0),y(0){}
	COMPUTE_GPS(int x,int y,double lng,double lat){
		this->x=x;
		this->y=y;
		this->lng=lng;
		this->lat=lat;
	}
}COMPUTE_GPS;

//�Զ���ͼƬ����ṹ
 
typedef struct  MAP_CPOINT{
	int x;     //ͼ��x
	int y;	   //ͼ��y
	MAP_CPOINT(int x,int y){
		this->x=x;
		this->y=y;
	}
}MAP_CPOINT;

//-------------------------��·����------------------------------
typedef struct {
	
	MAP_ROAD road;
}MAP_ROAD_PACKAGE;

 
typedef struct tagTurn
{
	int turn;//0ֱ�У�1��գ�2�ҹգ�3��Uturn��4��Uturn,5δ֪
	double turndegree;
}MAP_TURN;


#endif


