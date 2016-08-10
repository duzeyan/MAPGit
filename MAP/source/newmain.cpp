#include "NJUST_ALV_BYD_H/ALV_BYD.h"
#include"MapModule.h"
#include<iostream>
#include <fstream>
#include<sstream>
#include <time.h>
#include <stdarg.h>
using namespace std;



#ifdef _NJUST_OS_USE_WINDOWS_
//关键段变量声明  
CRITICAL_SECTION  g_csThreadGPS;//GPS读取互斥

//读取离线数据
DWORD  WINAPI readOffLineData(LPVOID p){
	double lat,lng;//维度 精度 度
	ifstream is;
	string oneline;//一行数据
	is.open("data.txt");
	if(!is.is_open()){
		//MAP_PRINT("[error]打开离线文件错误%s","\n");
		return 0;
	}
	//MAP_PRINT("[info]打开离线文件成功%s","\n");
	while (getline(is,oneline)){
		istringstream stream(oneline);
		stream>>lng>>lat;
		EnterCriticalSection(&g_csThreadGPS);//进入各子线程互斥区域 
		//更新curGPS
		if(MapApp::s_GPSInfo.curLongtitude!=INITL_GPS_VALUE){  //不是第一次取值
			MapApp::s_GPSInfo.lastLongtitude=MapApp::s_GPSInfo.curLongtitude;
			MapApp::s_GPSInfo.lastlatitudel=MapApp::s_GPSInfo.curLatitude;
		}else                                               //第一次取值
		{
			MapApp::s_GPSInfo.lastLongtitude=lng;
			MapApp::s_GPSInfo.lastlatitudel=lat;
		}
		MapApp::s_GPSInfo.curLongtitude = lng;
		MapApp::s_GPSInfo.curLatitude = lat;
		//计算丢包率相关
		MapApp::s_mapPackage.count++;
		LeaveCriticalSection(&g_csThreadGPS);
		Sleep(10);
	}
	return 0;
}
#endif

/////////////////////////////////测试函数/////////////////////////////////////////
//测试 MapFileStream的LoadMapNode
void testLoadMapNode(string s){
	NJUST_MAP_BUILD_MAP buildMap;
	MapFileStream *mapFile=new MapFileStream(s.c_str());
	mapFile->LoadMapNode(buildMap);
	
	vector<MAP_BUTTON_LINE>::iterator it=buildMap.mapLine.begin();
	for(;it!=buildMap.mapLine.end();it++){
		printf("%d\n",(*it).idself);
		//MAP_PRINT("%lf\n",(*it).b);
	}

	for(unsigned int i=0;i<buildMap.mapNode.size();i++){
		printf("NodeID:%d\n",buildMap.mapNode[i].idself);
		printf("gpsx:%lf\n",buildMap.mapNode[i].gpsx);
		printf("gpsy:%lf\n",buildMap.mapNode[i].gpsy);
		//for(unsigned int j=0;j<buildMap.mapNode[i].neigh;j++){
			//MAP_PRINT("%d\n",buildMap.mapNode[i].NeighLineID[j]);
		//}
	}

	//for(unsigned int i=0;i<buildMap.mapObs.size();i++){
		//MAP_PRINT("障碍物中心:%lf\n",buildMap.mapObs[i].ObstacleCenterGPS.longtitude);
	//}
}

//测试 MapFileStream的LoadMapTask
void testLoadMapTask(string s){
	vector<MAP_TASK_NODE_ZZ> taskMap;
	MapFileStream *mapFile=new MapFileStream(s.c_str());
	mapFile->LoadMapTask(taskMap);
	vector<MAP_TASK_NODE_ZZ>::iterator it=taskMap.begin();
	for(;it!=taskMap.end();it++){
		printf("%d:\n",(*it).resultCode);
		printf("%lf\n",(*it).longtitude);
		printf("%lf\n",(*it).latitude);
	}
	printf("结构体长度 %lu",sizeof(MAP_TASK_NODE_ZZ));
}

//邻接矩阵
void testReadAdjst(string s){
	NJUST_MAP_BUILD_MAP buildMap;
	MapFileStream *mapFile=new MapFileStream(s.c_str());
	mapFile->LoadMapNode(buildMap);
	mapFile->LoadAdjMat(buildMap);
	vector<int>::iterator it=buildMap.adjMat.begin();
	int count=buildMap.mapNode.size();
	for(int i=0;i<count;i++){
		for(int j=0;j < count ;j++)
			MAP_PRINT("%10d ",*(it+i*count+j));
		MAP_PRINT("%s\n","");
	}
}

//测试 apFileStream的读取序列点
void testReadMapNode(string s){
	vector<MAP_DOUBLE_POINT> GPSlist;
	MapFileStream *mapFile=new MapFileStream(s.c_str());
	mapFile->ReadMapGPS(1,2,GPSlist,true);

	int len=GPSlist.size();
	printf("count:%d\n",len);
	printf("fiset one: %lf  %lf\n",GPSlist[0].x,GPSlist[0].y);
	printf("fiset one: %lf  %lf\n",GPSlist[len-1].x,GPSlist[len-1].y);

	mapFile->ReadMapGPS(1,2,GPSlist,false);

	len=GPSlist.size();
	printf("count:%d\n",len);
	printf("fiset one: %lf  %lf\n",GPSlist[0].x,GPSlist[0].y);
	printf("fiset one: %lf  %lf\n",GPSlist[len-1].x,GPSlist[len-1].y);
}

//测试 读取记录文件
void testReadRecord(const char * filename){
	int len,code; 
	int curID,lastID;
	MAP_DOUBLE_POINT point;
	NJUST_PLAN_PATH _planPath;
	vector<MAP_DOUBLE_POINT> _GPSList;

	FILE *pFile=fopen(filename,"rb");

	fread(&curID,sizeof(int),1,pFile);    //curID
	fread(&lastID,sizeof(int),1,pFile);   //lastID

	//规划路线
	fread(&len,sizeof(int),1,pFile); 
	fread(&_planPath.cur,sizeof(int),1,pFile); 
	for(int i=0;i<len;i++){
		fread(&code,sizeof(int),1,pFile);
		_planPath.planPathQueue.push_back(code);
	}

	//GPS点
	fread(&len,sizeof(int),1,pFile);
	printf("len：%d\n",len);
	for(int i=0;i<len;i++){
		fread(&point,sizeof(MAP_DOUBLE_POINT),1,pFile);
		_GPSList.push_back(point);
	}
	fclose(pFile);

	printf("curID：%d\n",curID);
	printf("lastID：%d\n",lastID);
	printf("_planPath.cur：%d\n",_planPath.cur);
	for(size_t i=0;i<_planPath.planPathQueue.size();i++){
		printf("%lu:%d\n",i+1,_planPath.planPathQueue[i]);
	}
	for(size_t i=0;i<_GPSList.size();i++){
		printf("%lu:%lf\n",i+1,_GPSList[i].y);
	}

}
///////////////////////////////////初始化DEBUG输出///////////////////////////////////////



int main(int argc,char *argv[]){
	//Step 1 -----------检验输入参数--------------
#ifdef _NJUST_OS_USE_WINDOWS_
	string s="njustmap\\";
	InitializeCriticalSection(&g_csThreadGPS);
#endif

#ifdef _NJUST_OS_USE_LINUX_
	string s="njustmap/";
#endif

	//Step 2 -----------运行--------------
	MapApp mapapp;
	mapapp.initalize(s.c_str());
	mapapp.run();

	//Step 3 -----------关闭DEBUG--------------
	fclose(gDEBUG_OUT);
	fclose(gLOG_OUT);

	return 0;
}
