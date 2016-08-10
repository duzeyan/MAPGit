#ifndef _NJUST_MAP_TOOLS_H_
#define _NJUST_MAP_TOOLS_H_

#include<cmath>
#include<errno.h>
#include<string.h>
#include "NJUST_ALV_BYD_H/ALV_BYD.h"
#include"MAP_BASIC_data.h"





//Map 通用工具类
class MapTools{
public:
	
	//************************************
	// 描述： MAP_NODE转MAP_BUTTON_NOTE类型
	// 返回值: void
	// 参数:   MAP_NODE & node
	// 参数:   MAP_BUTTON_NOTE & buttonNode
	//************************************
	void static Node2ButtonNode(MAP_NODE &node,MAP_BUTTON_NOTE &buttonNode);

	//************************************
	// 描述： MAP_ROAD转MAP_BUTTON_LINE类型
	// 返回值: void
	// 参数:   MAP_ROAD & line
	// 参数:   MAP_BUTTON_LINE & MAP_ROAD
	//************************************
	void static Line2ButtonLine(MAP_ROAD &line,MAP_BUTTON_LINE &buttonLine);

	
	//************************************
	// 描述：  本线程sleep msecs 毫秒
	// 返回值: void
	// 参数:   unsigned int msecs sleep时间 单位毫秒
	//************************************
	void static ms_sleep( unsigned int msecs );

	
	//************************************
	// 描述：  检测GPS值是否在合理区间内(大致中国版图)
	//         并判断与上一帧GPS差是否合理
	// 返回值: bool true 合理
	// 参数:   double lng 当前经度 度
	// 参数:   double lat 当前维度 度
	// 参数:   double lastlng 上一帧经度 度
	// 参数:   double lastlat 上一帧维度 度
	//************************************
	bool static CheckGPS(double lng,double lat,double lastlng,double lastlat);

	
	//************************************
	// 描述：  GPS转化为大地坐标
	// 返回值: void
	// 参数:   double lat 维度
	// 参数:   double lng 经度
	// 参数:   int & earthx  x cm
	// 参数:   int & earthy  y cm
	//************************************
	void static GPS2Earthy(double lat, double lng, int &earthx, int &earthy);

	
	//************************************
	// 描述： 计算两对经纬度获取之间的欧氏距离(m)
	// 返回值: double      距离 米
	// 参数:   double lng1 经度 度
	// 参数:   double lat1 维度 度
	// 参数:   double lng2 经度 度
	// 参数:   double lat2 维度 度
	//************************************
	double static GetDistanceByGPS(double lng1,double lat1,double lng2,double lat2);

	
	//************************************
	// 描述：  通过路口ID获得路口索引
	// 返回值: int  返回nodes中结果索引
	// 参数:   const vector<MAP_BUTTON_NOTE> & nodes 节点
	// 参数:   int id                          查询ID
	//************************************
	int static GetNodeIndexByID(const vector<MAP_BUTTON_NOTE> &nodes,int id);

	//************************************
	// 描述： 根据距离且在给出GPS序列点上的障碍物对象列表
	// 返回值: void
	// 参数:   double lng  经度 度
	// 参数:   double lat  维度 度
	// 参数:   double distanceM 
	// 参数:   const vector<NJUST_MAP_OBSTACLE> & obs 障碍物对象列表
	// 参数:   vector<NJUST_MAP_OBSTACLE> & outObs    [out]符合条件的障碍物列表
	//************************************
	static void GetObsByDistance(double lng,double lat,
								double distanceM,const vector<NJUST_MAP_OBSTACLE> &obs
								,vector<NJUST_MAP_OBSTACLE> &outObs);

	
	//************************************
	// 描述：  通过ID获取路口属性
	// 返回值: MAP_BUTTON_NOTE
	// 参数:   const vector<MAP_BUTTON_NOTE> & nodes
	// 参数:   int id
	//************************************
	MAP_BUTTON_NOTE static GetNodeByID(const vector<MAP_BUTTON_NOTE> &nodes,int id);

	
	//************************************
	// 描述：  通过道路ID获得道路索引
	// 返回值: int
	// 参数:   const vector<MAP_BUTTON_LINE> & lines
	// 参数:   int id
	//************************************
	int static GetLineIndexByID(const vector<MAP_BUTTON_LINE> &lines,int id);

	
	//************************************
	// 描述：  通过道路ID获得道路属性
	// 返回值: MAP_BUTTON_LINE
	// 参数:   const vector<MAP_BUTTON_LINE> & lines 道路属性源
	// 参数:   int id						   待查ID
	//************************************
	MAP_BUTTON_LINE static GetLineByID(const vector<MAP_BUTTON_LINE> &lines,int id);


	
	//************************************
	// 描述： MAP_BUTTON_LINE转化NJUST_MAP_INFO_ROAD
	// 返回值: void
	// 参数:   MAP_BUTTON_LINE * line
	// 参数:   NJUST_MAP_INFO_ROAD * * road
	//************************************
	void static StructTransformLine(MAP_BUTTON_LINE* line, NJUST_MAP_INFO_ROAD **road);

	
	//************************************
	// 描述：  MAP_BUTTON_NOTE转化为NJUST_MAP_INFO_NODE
	// 返回值: void
	// 参数:   MAP_BUTTON_NOTE * note	     
	// 参数:   NJUST_MAP_INFO_NODE * * node  
	//************************************
	void static StructTransformNote(MAP_BUTTON_NOTE* note, NJUST_MAP_INFO_NODE **node);

	
	 //************************************
	 // 描述：  打包MAP数据包
	 // 返回值: int
	 // 参数:   const void * pUnknow 原始数据
	 // 参数:   int date       数据大小
	 // 参数:   char globle[]  结果
	 //************************************
	 int static NJUST_MAP_Encode_IP_Data(const void* pUnknow, int date, char globle[]);

	
	 //************************************
	 // 描述：   计算向量v1,v2的夹角
	 // 返回值: double 角度 单位度
	 // 参数:   double x1  v1(x1,y1)
	 // 参数:   double y1
	 // 参数:   double x2  v2(x2,y2)
	 // 参数:   double y2
	 //************************************
	 double static GetRotateAngle(double x1, double y1, double x2, double y2);//以度为单位

	 //************************************
	 // 描述：  ID转化为编号
	 // 返回值: int       编号
	 // 参数:   int id  ID
	 //************************************
	 int static ID2Code(int id);


	 
	 //************************************
	 // 描述：  编号转化为ID
	 // 返回值: int       ID
	 // 参数:   int code  编号
	 // 参数:   int type  类型，1为节点 2为道路
	 //************************************
	 int static Code2ID(int code,int type);
};

#endif