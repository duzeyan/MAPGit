#include<vector>
#include"MAP_BASIC_data.h"
#include"MapModule.h"
#include <time.h>
#include <stdarg.h>
#include <fstream>

#ifdef _NJUST_OS_USE_LINUX_
	#include <unistd.h>
	pthread_mutex_t     gMutex= PTHREAD_MUTEX_INITIALIZER;
	pthread_cond_t      cond =PTHREAD_COND_INITIALIZER;
	#define ACCESS access
#endif

#ifdef _NJUST_OS_USE_WINDOWS_
#include <io.h>
#define F_OK 0 //检查文件存在性
#define ACCESS _access
extern CRITICAL_SECTION  g_csThreadGPS;//GPS读取互斥
#endif
///
/// MapApp
///

#define RECORD_FILE_NAME "record.db" //中间状态文件
#define LOCATION_ERROR_TIME_MAX  50*5    //定位错误最大次数,超过后搜索定位 1次大约20毫秒
#define START_DIS_THRESHOLD_M 100  //启动点距路口距离(单位M)
#define NODE_DIS_THRESHOLD_M 20  //定位路口阈值(单位M)
const double TASK_DIS_THRESHOLD_M=50.0;  

//静态变量 定义
NJUST_MAP_GPS_INFO MapApp::s_GPSInfo;
MAP_PACKAGE MapApp::s_mapPackage;
NJUST_MAP_WORK_MODEL MapApp::s_WorkModel=MAP_WORK_NORMAL;



FILE* gDEBUG_OUT=NULL;
FILE* gLOG_OUT=NULL;
FILE* gOFFData=NULL;
void initDeBug(){
	char logName[1024] = {0};  
	time_t now;  
	time(&now);  
	struct tm *local;  
	local = localtime(&now);  
	sprintf(logName,"DEBUGMAP-%04d-%02d-%02d_%02d-%02d-%02d", local->tm_year+1900, local->tm_mon+1,  
		local->tm_mday, local->tm_hour, local->tm_min, local->tm_sec);  
	gDEBUG_OUT = fopen(logName,"a+");
	return ;  
}

void initLog(){
	char logName[1024] = {0};  
	time_t now;  
	time(&now);  
	struct tm *local;  
	local = localtime(&now);  
	sprintf(logName,"LOGMAP-%04d-%02d-%02d_%02d-%02d-%02d", local->tm_year+1900, local->tm_mon+1,  
		local->tm_mday, local->tm_hour, local->tm_min, local->tm_sec);  
	gLOG_OUT = fopen(logName,"a+");  
	return ;  
}

void intOFFlineData(){
#ifdef _MAP_OFF_DATA_
	char logName[1024] = {0};  
	time_t now;  
	time(&now);  
	struct tm *local;  
	local = localtime(&now);  
	sprintf(logName,"OFFDATA-%04d-%02d-%02d_%02d-%02d-%02d", local->tm_year+1900, local->tm_mon+1,  
		local->tm_mday, local->tm_hour, local->tm_min, local->tm_sec);  
	gOFFData = fopen(logName,"a+");  
	return ;  
#endif
}


  //当前位置到任务点距离(单位M)

///
/// 编号 索引 ID的定义
/// 节点的ID为：10000~19999
/// 道路的ID为：20000~
/// 索引只在邻接矩阵处出现，且只针对节点而言
/// 编号，在作图软件中，路口道路已1开头 如道路1 道路2 节点1 
/// 另外GPS是按照编号方式进行命名的
/// 编号转ID为 编号-1+10000|20000


MapApp::MapApp(){
}

MapApp::~MapApp(){
	release();
}


void MapApp::run(){
	double curLng,curLat,lastLng,lastLat;//最新获取的经纬度,上一次获取的经纬度
	int sIndex=-1;                           //在任务节点中的位置索引
	int lastID;								 //最近离开道路或路口的 ID
	int curID;								 //当前所在的道路或节点 ID
	int lastNode=-1;						 //上一次经过的节点编号，用来判断当前方向
	bool isDriveBack=false;					 //是否进入倒车状态
	int locationErrTime=0;					 //错误定位计数器
	int GPSErrorTime=0;//GPS偏离合理值计数器

	//Step 1 -----------判断是否异常退出--------------
	if(_checkExpOut){ //异常退出
		recordRead(curID,lastID,sIndex); //读取记录文件
		MAP_PRINT("意外重启,最近启动点: %d\n",sIndex);
	}else{           //正常启动
		startPlan(sIndex);
		if(sIndex==-1){
			MAP_PRINT("附近没有路口,无法启动 %d\n",sIndex);
			return;
		}
		pathPlaning(_mapTaskNode[sIndex].resultCode,_mapTaskNode[sIndex+1].resultCode);//规划路径，结果存在planPath
		_historyPath.push_back(_planPath.planPathQueue[0]);//记录当前路径

		//启动时的初始化
		lastID=MapTools::Code2ID(_planPath.planPathQueue[0],1);
		curID=lastID;
		//启动的特殊处理 认为启动点是路段 节点1-》节点2之间的读点
		_mapFile->ReadMapGPS(_planPath.planPathQueue[0],_planPath.planPathQueue[2],_GPSList,false);
		recordWrite(curID,lastID,sIndex);//记录状态
	}
	MAP_PRINT("初始curID:%d\n",curID);
	//Step 2 -----------进入运行状态--------------
	
	while(1){
		//Step 2.1 -----------获取最新的GPS--------------
#ifdef _NJUST_OS_USE_LINUX_
		pthread_mutex_lock(&gMutex);
#endif
#ifdef _NJUST_OS_USE_WINDOWS_
		EnterCriticalSection(&g_csThreadGPS);//进入各子线程互斥区域 
#endif
		curLng=MapApp::s_GPSInfo.curLongtitude; //最新经度 (度)
		curLat=MapApp::s_GPSInfo.curLatitude;   //最新维度 (度)
		lastLng=MapApp::s_GPSInfo.lastLongtitude;
		lastLat=MapApp::s_GPSInfo.lastlatitudel;
#ifdef _NJUST_OS_USE_LINUX_
		pthread_cond_signal(&cond);
		pthread_mutex_unlock(&gMutex);
#endif
#ifdef _NJUST_OS_USE_WINDOWS_
		LeaveCriticalSection(&g_csThreadGPS);//进入各子线程互斥区域 
#endif
		MapTools::ms_sleep(10);		//控制接受频率
		//Step 2.2 -----------丢弃不合理GPS数据--------------
		if (MapTools::CheckGPS(curLng,curLat,lastLng,lastLat))
		{	
			//检查当前点是否是任务点
			if(locationTaskPoint(curLng,curLat,TASK_DIS_THRESHOLD_M)){
				MapApp::s_WorkModel=MAP_WORK_TASK;
			}
			//检查是否已经偏离规划路径过久
			if(locationErrTime>LOCATION_ERROR_TIME_MAX){
				MapApp::s_WorkModel=MAP_WORK_SHEARRIGHT;
			}
			switch(MapApp::s_WorkModel){
				//Step 3.1 -----------导航模式---------------
				case  MAP_WORK_NORMAL:{	
					if(processRunMoudle(curLng,curLat,curID,lastID,lastNode,sIndex,locationErrTime))
						goto END;                     //运行结束
					break;
				}
				//Step 3.2 -----------倒车模式------------------
				case  MAP_WORK_BACKDRAW:{	
						if(!isDriveBack){   
							DEBUG_PRINT("进入倒车模式%s","\n");
							DEBUG_PRINT("_historyPath.size(): %lu \n",_historyPath.size());
							for(size_t i=0;i<_historyPath.size();i++)
								DEBUG_PRINT("%d ",_historyPath[i]);
							DEBUG_PRINT("%s","\n");
							isDriveBack=true;//标记为倒车模式
						}
						if(processDriveBack(curLng,curLat))
							MapApp::s_WorkModel=MAP_WORK_REPLAN;//进入到重规划状态
						send2Mo();//持续发状态包
						break;
				}
				//Step 3.3 -----------重规划模式--------------
				case  MAP_WORK_REPLAN:{
					processRePlan(sIndex);
					MapApp::s_WorkModel=MAP_WORK_NORMAL;//进入到正常行驶状态
					DEBUG_PRINT("重规划完成%s","\n");
					for(size_t i=0;i<_planPath.planPathQueue.size();i++){
						if(i%2==0){//节点
							DEBUG_PRINT("[%d] ",_planPath.planPathQueue[i]);
						}else{//道路
							DEBUG_PRINT("%d ",_planPath.planPathQueue[i]);
						}
					}
					DEBUG_PRINT("curIndex:%d\n",_planPath.cur);
					lastID=MapTools::Code2ID(_planPath.planPathQueue[1],2);//更新当前道路ID
					curID=lastID;
					recordWrite(curID,lastID,sIndex);//记录状态
					sendPLCmd(NJUST_PL_MAP_COMMAND_TYPE_CAR_REVERSE_STOP);//通知PL停止倒车
					send2Mo();//持续发状态包
					break;
				}
				//Step 3.4 -----------道路规划重定位模式--------------
				case MAP_WORK_SHEARRIGHT:{
					int theID=location(curLat,curLng);//当前定位ID
					int tID; 
					for(unsigned int i=0;i<_planPath.planPathQueue.size();i++){
						if(i%2==0){ //节点
							tID=MapTools::Code2ID(_planPath.planPathQueue[i],1);
						}else{     //道路
							tID=MapTools::Code2ID(_planPath.planPathQueue[i],2);
						}
						if(tID==theID){
							_planPath.cur=i; //重置
							curID=lastID=theID;
							break;
						}
					}
					locationErrTime=0;
					MapApp::s_WorkModel=MAP_WORK_NORMAL;//进入到正常行驶状态
					DEBUG_PRINT("从搜索模式切换成正常模式%s","\n");
					break;
			   }
				//Step 3.5 -----------巡航目标检测模式--------------
				case MAP_WORK_TASK:{
					DEBUG_PRINT("到达目标检测任务点%s","\n");
					MapTools::ms_sleep(100);
					MapApp::s_WorkModel=MAP_WORK_NORMAL;
					break;
				 }
			}//switch
		}//checkGPS
		else{
			GPSErrorTime++;
			GPSErrorTime=GPSErrorTime%10000000;
			DEBUG_PRINT("GPSErrorTime:%d\n",GPSErrorTime);
		}
	}//while
END:
	sendPLCmd(NJUST_PL_MAP_COMMAND_TYPE_REACH_ENDPOINT);
	return;
}//fun

bool MapApp::processRunMoudle(double curLng,double curLat,int &curID,int &lastID,int &lastNode,int &sIndex,int &errTime){
	//Step 1 -----------定位当前位置--------------
	int tID;
	if((tID=location(curLng,curLat))==-1){
		MAP_PRINT("没有找到合适直线 ID为%d\n",-1);
		return false;
	}
	curID=tID;

	//Step 2 -----------是否进入新的道路(节点)--------------
	if(curID!=lastID){              //进入新的路口或道路 此时记录
		if(!checkLoaction(curID)){ //不是预期目标,跳过
			errTime++;
			curID=lastID;goto CONTINUE;
		}
		_planPath.cur++;
		_historyPath.push_back(_planPath.planPathQueue[_planPath.cur]);//记录当前路径
		MAP_PRINT("=========================:%s\n","");
		if(_planPath.cur%2==0){
			MAP_PRINT("进入新路口:[路口%d]",MapTools::ID2Code(curID));
			MAP_PRINT("(%d)\n",curID);
		}
		else{
			MAP_PRINT("进入新道路:[道路%d]",MapTools::ID2Code(curID));
			MAP_PRINT("(%d)\n",curID);
		}

		size_t tcur=_planPath.cur;
		if(tcur<_planPath.planPathQueue.size()-1){//最近一次规划的路线没走完，读新路段文件
			MAP_PRINT("%s没有读完最近一次规划的节点\n","");
			bool isNode=(tcur%2==0);  //偶数是路口
			//读取指定段序列
			_mapFile->ReadMapGPS(_planPath.planPathQueue[tcur-1],
				_planPath.planPathQueue[tcur+1],_GPSList,
				isNode);
			//更新上一个路口编号
			if(isNode)
				lastNode=_planPath.planPathQueue[tcur-2];
			else
				lastNode=_planPath.planPathQueue[tcur-1];
			lastID=curID; //更新 lastID
			recordWrite(curID,lastID,sIndex); //记录最新状态
		}else{	//走到最近一次规划的最后一个节点 tcur==_planPath.planPathQueue.size()-1
			MAP_PRINT("%s新的规划路线\n","");
			int lastRoad=_planPath.planPathQueue[_planPath.cur-1]; //记录最近一次规划走过的最后一段路，来构建路口过渡
			lastNode=_planPath.planPathQueue[_planPath.cur-2]; 
			sIndex++;
			if(sIndex+1==int(_mapTaskNode.size()-1)){//最后一个节点是任务节点
				MAP_PRINT("到达最后的路口!%s","\n");
				remove(RECORD_FILE_NAME);  //删除记录文件
				return true;
			}
			pathPlaning(_mapTaskNode[sIndex].resultCode,_mapTaskNode[sIndex+1].resultCode);//规划路径，结果存在planPath
			//读取过渡 路口信息
			_mapFile->ReadMapGPS(lastRoad,
				_planPath.planPathQueue[1],_GPSList,
				true);
			lastID=curID;
			recordWrite(curID,lastID,sIndex); //记录最新状态
		}
	}//if find new ID
	CONTINUE:

	//Step 3 -----------广播道路或路口信息--------------
	if(_planPath.cur%2==0){  //路口
		_frameNum++;
		int lastIndex=MapTools::GetNodeIndexByID(_map.mapNode,lastNode-1+START_NODE_ID);
		int nextIndex=MapTools::GetNodeIndexByID(_map.mapNode,_planPath.planPathQueue[_planPath.cur+2]-1+START_NODE_ID);
		sendNode(curLng,curLat,curID,lastIndex,nextIndex);
	}else{                  //道路
		int theNestIndex=MapTools::GetNodeIndexByID(_map.mapNode,_planPath.planPathQueue[_planPath.cur+1]-1+START_NODE_ID);
		_frameNum++;
		sendRoad(curLng,curLat,curID,theNestIndex);
	}
	return false;
}

bool MapApp::processDriveBack(double curLng,double curLat){
	//需要退回到上一个道路中
	int locationID=-1; //当前位置道路或者节点ID
	int hisLen=-1; //历史路径长度
	int code=-1; //当前位置道路或者节点编号

	//Step 1 -----------定位当前位置--------------
	locationID=location(curLng,curLat);
	if(locationID==-1) //倒车模式不记录定位错误
		return false;
	hisLen=_historyPath.size();         
	code=MapTools::ID2Code(locationID);//转成编号
	DEBUG_PRINT("locationID: %d \n",locationID);

	//Step 2 -----------判断倒车指令触发时,是在节点还是道路上--------------
	if(hisLen%2==0){ //停在道路
		if(_historyPath[hisLen-3]==code){
			return true;
		}
	}else{			//停在节点
		if(locationID<START_LINE_ID){ //定位到节点 忽略
			return false;
		}
		if(_historyPath[hisLen-2]==code){
			return true;
		}
	}
	return false;
}

void MapApp::processRePlan(int taskIndex){
	int sCode,eCode; //断路两端的节点ID
	int restartCode;//重新开始规划的节点ID
	int hisLen=-1; //历史路径长度
	int cur=_planPath.cur;
	hisLen=_historyPath.size();         

	DEBUG_PRINT("\n重规划 cur:%d\n",cur);
	DEBUG_PRINT("重规划 hisLen:%d\n",hisLen);

	DEBUG_PRINT("重规划 _historyPath:%s","\n");
	for(int i=0;i<hisLen;i++)
		DEBUG_PRINT("%d ",_historyPath[i]);
	DEBUG_PRINT("%s","\n");
	//Step 1 -----------判断倒车指令触发时,是在节点还是道路上--------------
	if(hisLen%2==0){ //停在道路
		eCode=_planPath.planPathQueue[cur+1];
		sCode=_historyPath[hisLen-1-1];
		restartCode=sCode;
		DEBUG_PRINT("restartCode: %d\n",restartCode);
		DEBUG_PRINT("sCode: %d\n",sCode);
		DEBUG_PRINT("eCode: %d\n",eCode);
		_historyPath.pop_back();  //去除不可通道路
		_historyPath.pop_back();  //去除不可通道路的起始点
	}else{ //停在节点
		sCode=_planPath.planPathQueue[cur];
		eCode=_planPath.planPathQueue[cur+2];
		restartCode=sCode;
		DEBUG_PRINT("restartCode: %d\n",restartCode);
		DEBUG_PRINT("sCode: %d\n",sCode);
		DEBUG_PRINT("eCode: %d\n",eCode);
		_historyPath.pop_back();  //去除不可通道路的起始点
	}

	//Step 2 -----------切断不可走的路--------------
	int sIndex=MapTools::GetNodeIndexByID(_map.mapNode,MapTools::Code2ID(sCode,1));
	int eIndex=MapTools::GetNodeIndexByID(_map.mapNode,MapTools::Code2ID(eCode,1));
	int nodeCount=_map.mapNode.size();
	_map.adjMat[sIndex+eIndex*nodeCount]=INF;
	_map.adjMat[eIndex+sIndex*nodeCount]=INF;

	//Step 3 -----------重规划--------------
	pathPlaning(restartCode,_mapTaskNode[taskIndex+1].resultCode);//规划路径，结果存在planPath
	DEBUG_PRINT("next Node: %d\n",_mapTaskNode[taskIndex+1].resultCode);
	//Step 4 -----------填充前段信息,此步为了符合NORMAL模式下的逻辑--------------
	vector<int>::iterator it=_planPath.planPathQueue.begin();
	hisLen=_historyPath.size();
	_planPath.planPathQueue.insert(it,_historyPath[hisLen-1]);
	it=_planPath.planPathQueue.begin();
	_planPath.planPathQueue.insert(it,_historyPath[hisLen-2]);
	_planPath.cur=1;//此时在该道路结尾处
}

void MapApp::simulate(){
	double curLng,curLat;    //GPS数据
	int SIndex=-1;                           //在任务节点中的位置索引
	int lastID;								 //最近离开道路或路口的 ID
	int curID;								 //当前所在的道路或节点 ID
	vector<MAP_DOUBLE_POINT> pathList;        //模拟运行中的输出结果
	vector<MAP_DOUBLE_POINT> wholeList;      //保存全局结果
	
	///Step 1 -----------启动初始化--------------
	SIndex=0;
	pathPlaning(_mapTaskNode[SIndex].resultCode,_mapTaskNode[SIndex+1].resultCode);//规划路径，结果存在planPath

	//启动时的初始化
	lastID=MapTools::Code2ID(_planPath.planPathQueue[0],1);
	curID=lastID;
	//启动的特殊处理 认为启动点是路段 节点1-》节点2之间的读点
	_mapFile->ReadMapGPS(_planPath.planPathQueue[0],_planPath.planPathQueue[2],_GPSList,false);
	curLng=_GPSList[0].x/60;
	curLat=_GPSList[0].y/60;
	//Step 2 -----------进入运行状态--------------
	while(1){
			if((curID=location(curLng,curLat))==-1){
				MAP_PRINT("没有找到合适直线 ID为%d\n",curID);
				continue;
			}
			if(curID!=lastID){              //进入新的路口或道路 此时记录
				if(!checkLoaction(curID)){
					curID=lastID;goto CONTINUE;
				}
				_planPath.cur++;
				MAP_PRINT("=========================:%s\n","");
				if(_planPath.cur%2==0){
					MAP_PRINT("进入新路口:[路口%d]",MapTools::ID2Code(curID));
					MAP_PRINT("(%d)\n",curID);
				}
				else{
					MAP_PRINT("进入新道路:[道路%d]",MapTools::ID2Code(curID));
					MAP_PRINT("(%d)\n",curID);
				}

				size_t tcur=_planPath.cur;
				if(tcur<_planPath.planPathQueue.size()-1){//最近一次规划的路线没走完，读新路段文件
					MAP_PRINT("%s没有读完最近一次规划的节点\n","");
					bool isNode=(tcur%2==0);  //偶数是路口
					//读取指定段序列
					_mapFile->ReadMapGPS(_planPath.planPathQueue[tcur-1],
						_planPath.planPathQueue[tcur+1],_GPSList,
						isNode);
					lastID=curID; //更新 lastID
				}else{								   //走到最近一次规划的最后一个节点
					MAP_PRINT("%s新的规划路线\n","");
					int lastRoad=_planPath.planPathQueue[_planPath.cur-1]; //记录最近一次规划走过的最后一段路，来构建路口过渡
					SIndex++;
					if(SIndex+1==int(_mapTaskNode.size()-1)){
						MAP_PRINT("到达最后的路口!%s","\n");
						goto END;
					}
					pathPlaning(_mapTaskNode[SIndex].resultCode,_mapTaskNode[SIndex+1].resultCode);//规划路径，结果存在planPath
					//读取过渡 路口信息
					_mapFile->ReadMapGPS(lastRoad,
						_planPath.planPathQueue[1],_GPSList,
						true);
				}
			}//if find new ID
			CONTINUE:

			//Step 3 -----------定位记录GPS--------------
			simMakeGPS(curLng,curLat,pathList);
		    for(size_t i=0;i<pathList.size();i++){
				wholeList.push_back(pathList[i]);
			}
			DEBUG_PRINT("输出+%s","\n");
			//更新GPS 跨越三个GPS
			curLng=pathList.back().x/60; //经度维度
			curLat=pathList.back().y/60;
			DEBUG_PRINT("更新GPS lng:%lf \n",curLng);
			DEBUG_PRINT("更新GPS lat:%lf ",curLat);
	}//while
	END:
	simOutResult(wholeList);//输出结果
	exit(0); 
	return ;
}

void MapApp::simOutResult(const vector<MAP_DOUBLE_POINT> &list){
	const char *logName="simResult.path";  
	unsigned int i;
	FILE *pFile;

	pFile = fopen(logName,"w+"); 
	for(i=0;i<list.size();i++){
		MAP_DOUBLE_POINT tp=list[i];
		tp.x=tp.x/60;  
		tp.y=tp.y/60;
		fwrite(&tp,sizeof(MAP_DOUBLE_POINT),1,pFile);
	}
	fclose(pFile);
	return ;  
}

void MapApp::simMakeGPS(double lng,double lat,vector<MAP_DOUBLE_POINT> &GPSList){
	size_t indexGPS=locationGPS(lng,lat);
	GPSList.clear();
	//最多读20个GPS点
	MAP_DOUBLE_POINT point;
	for(size_t i=indexGPS;i<indexGPS+20;i++){
		if(i<_GPSList.size()){
			point.x = _GPSList[i].x;
			point.y =_GPSList[i].y;
			GPSList.push_back(point);
		}
	}
}

void MapApp::initalize(const char* loadpath){
	//Step 0 -----------声明--------------
	MapCommunion communion;

	//Step 1 -----------初始化通信模块--------------
#ifdef _NJUST_OS_USE_LINUX_
	communion.RegisterMap();//注册本模块
	communion.ReciveModuleMsg("MO",MOCallBack);	//响应MO命令
	communion.ReciveModuleMsg("PL",PLCallBack);	//响应MO命令
	communion.ReciveBroadcastMsg(MCCallBack);//对MC发出的数据，进行记录
#endif

#ifdef _NJUST_OS_USE_WINDOWS_
	communion.RegisterOffLineSys();
#endif 
	_frameNum=0;//帧号初始化

	//Step 1 -----------初始化日志系统--------------
	initDeBug();
	initLog();
	intOFFlineData();

	//Step 2 -----------从文件中读取地图数据--------------
	//try{
		_mapFile=new(std::nothrow) MapFileStream(loadpath);
		if(_mapFile==NULL){
			MAP_PRINT("%s 实例化文件流对象失败\n","MapApp::Inialize");
			exit(-1);
		}
	//}catch(const std::bad_alloc& e){
		//MAP_PRINT("%s 实例化文件流对象失败\n","MapApp::Inialize");
		//exit(-1);
	//}
	_mapFile->LoadMapNode(_map);//加载自建地图:路口和道路信息
	_mapFile->LoadAdjMat(_map);//加载地图中的邻接表
	_mapFile->LoadMapTask(_mapTaskNode); //加载需要进一步规划的路径信息

	//Step 3 -----------接收的GPS信息初始化--------------
	MapApp::s_GPSInfo.curLongtitude=INITL_GPS_VALUE;
	memset(&s_mapPackage,0,sizeof(MAP_PACKAGE));
	
	//Step 4 -----------初始化记录文件--------------
	if((ACCESS(RECORD_FILE_NAME,F_OK))==0){//如果存在说明中断过
		_checkExpOut=true;
	}else{
		_checkExpOut=false;
	}
	//Step 5 -----------发给PL的命令号ID--------------
	_nCurToMapCmdID=0;
	//Step 6 -----------发送状态包--------------------
	send2Mo();
}


void MapApp::startPlan(int& start){
	double curLng,curLat;
	double dis=0;//启动点附近距离 单位m

	//Step 1 -----------等待接收到有效数据--------------
	while(1){
#ifdef _NJUST_OS_USE_LINUX_
		pthread_mutex_lock(&gMutex);
#endif
#ifdef _NJUST_OS_USE_WINDOWS_
		EnterCriticalSection(&g_csThreadGPS);//进入各子线程互斥区域 
#endif

		curLng=MapApp::s_GPSInfo.curLongtitude; //最新经度 (度)
		curLat=MapApp::s_GPSInfo.curLatitude;   //最新维度 (度)

#ifdef _NJUST_OS_USE_LINUX_
		pthread_cond_signal(&cond);
		pthread_mutex_unlock(&gMutex);
#endif
#ifdef _NJUST_OS_USE_WINDOWS_
		LeaveCriticalSection(&g_csThreadGPS);//进入各子线程互斥区域 
#endif
		if(curLng==INITL_GPS_VALUE)
			MapTools::ms_sleep(100);
		else
			break;
		MAP_PRINT("%s挂起100ms 等待MC发送坐标\n","");

	}

	//Step 2 -----------获取最近的节点--------------
	DEBUG_PRINT("当前URLNG %lfm \n",curLng);
	DEBUG_PRINT("当前URLAT %lfm \n",curLat);
	for(unsigned int i=0;i<_mapTaskNode.size()-1;i++){
		dis=MapTools::GetDistanceByGPS(curLng,curLat,_mapTaskNode[i].longtitude,_mapTaskNode[i].latitude);
		MAP_PRINT("dis:%lf \n",dis);
		MAP_PRINT("code:%d \n",_mapTaskNode[i].resultCode);
		if(dis<START_DIS_THRESHOLD_M){
			MAP_PRINT("离最近节点相差%lfm \n",dis);
			start=i;
			break;
		}
	}
	DEBUG_PRINT("搜索失败最近节点相差%lfm \n",dis);
	return;
}

int MapApp::location(double lng,double lat){
	//Step 1 -----------寻找离该GPS最近的路口--------------
	double min=5000;//5km
	double dis=0;
	int minNodeIndex=-1; //最近节点索引
	for(size_t i=0;i<_map.mapNode.size();i++){
		dis=MapTools::GetDistanceByGPS(lng,lat,_map.mapNode[i].gpsx,_map.mapNode[i].gpsy);
		/*MAP_PRINT("curLng%lf \n",lng);
		MAP_PRINT("curLat%lf \n",lat);
		MAP_PRINT("gpsx %lf\n",_map.mapNode[i].gpsx);
		MAP_PRINT("gpsy %lf\n",_map.mapNode[i].gpsy);*/
		if(dis<min){
			min=dis;
			minNodeIndex=i;
		}
	}
	//DEBUG_PRINT("min dis:%lf \n",dis);

	//Step 2 -----------确定当前位置是否在路口上--------------
	if(min<NODE_DIS_THRESHOLD_M){				// 在100m范围之内
		return _map.mapNode[minNodeIndex].idself;
	}
	
	//Step 3 -----------确定最近路口邻接的路口是否有符合条件的,路口优先--------------
	for(int i=0;i<_map.mapNode[minNodeIndex].neigh;i++){
		int ID=_map.mapNode[minNodeIndex].NeighNoteID[i];
		int index=MapTools::GetNodeIndexByID(_map.mapNode,ID);
		dis=MapTools::GetDistanceByGPS(lng,lat,_map.mapNode[index].gpsx,_map.mapNode[index].gpsy);
		if(dis<NODE_DIS_THRESHOLD_M){ //10米
			return ID;
		}
	}

	//Step 4 -----------返回最近路口邻接的道路和该位置最近的道路--------------
	double minCM=100*1000;//1KM
	int LineID=-1;

	//Step 5 -----------找距离最近的道路--------------
	for(int i=0;i<_map.mapNode[minNodeIndex].neigh;i++){
		int ID=_map.mapNode[minNodeIndex].NeighLineID[i];  //先按照经纬度舍弃直线
		int index=MapTools::GetLineIndexByID(_map.mapLine,ID);
		if(isInLine(lng,lat,index)){   //在符合条件的道路中 寻找最近道路
			double b=_map.mapLine[index].b;
			double c=_map.mapLine[index].c;
			double k=_map.mapLine[index].k;
			int ex,ey;
			double dex,dey;
			MapTools::GPS2Earthy(lat,lng,ex,ey);
			dex=ex-_map.adjustPoint.x;dey=ey-_map.adjustPoint.y; //防止溢出和统一坐标系	
			double temp = abs((k*dex + b*dey + c)) / sqrt(b*b + k*k);
			if(temp<minCM){
				minCM=temp;
				LineID=ID;
			}
		}
	}
	return LineID;
}

int MapApp::locationGPS(double lng,double lat){
	double min=100000; //10W km
	int index=-1;
	for(size_t i=0;i<_GPSList.size();i++){
		double dis=MapTools::GetDistanceByGPS(lng,lat,_GPSList[i].x/60,_GPSList[i].y/60);
		if(dis<min){
			min=dis;
			index=i;
		}
	}
	return index;
}


bool MapApp::isInLine(double lng,double lat,int index){
	double tlnglat=0;
	//Step 1 -------------获取前后两个路口的ID------------
	int IDs=_map.mapLine[index].idstart;			//起点终点节点 ID
	int IDe=_map.mapLine[index].idend;
	
	//Step 2 ------------ID换算成路口数组索引-------------
	int indexS=MapTools::GetNodeIndexByID(_map.mapNode,IDs); //起点终点 index
	int indexE=MapTools::GetNodeIndexByID(_map.mapNode,IDe);

	//Step 3 -----------获取经纬度--------------
	double lngS=_map.mapNode[indexS].gpsx;
	double latS=_map.mapNode[indexS].gpsy;
	double lngE=_map.mapNode[indexE].gpsx;
	double latE=_map.mapNode[indexE].gpsy;

	//Step 4 -----------把s到e调整为经纬度递增--------------
	if(lngS>lngE){
		tlnglat=lngS;
		lngS=lngE;
		lngE=tlnglat;
	}
	if(latS>latE){
		tlnglat=latS;
		latS=latE;
		latE=tlnglat;
	}
	
	//Step 5 -----------判断是否在主要经纬度方向内--------------
	if((lngE-lngS)>(latE-latS)){ //道路主要走向为 经度方向 东西
		if(lng>lngS&&lng<lngE)
			return true;
	}else{						//道路主要走向为 维度方向 南北
		if(lat>latS&&lat<latE)
			return true;
	}
	return false;
}


void MapApp::pathPlaning(int s,int e){
	//Step 0 -----------声明--------------
	vector<int> pathNode;  //节点序列
	vector<int> pathWhole; //最终结果
	pathNode.reserve(10);
	int tempID,tempID2;   //临时道路ID
	size_t i;
	MAP_BUTTON_NOTE	theNode1,theNode2; //临时存储节点
	tempID2=tempID=0;

	//Step 1 -----------根据最短路径算法得到节点序列--------------
	dijkstra(s,e,pathNode);
	
	//Step 2 -----------在道路序列中插入对于的道路编号--------------
	pathWhole.reserve(pathNode.size()*2);
	for(i=0;i<pathNode.size()-1;i++){
		pathWhole.push_back(pathNode[i]); //当前节点编号
		theNode1=MapTools::GetNodeByID(_map.mapNode
									,MapTools::Code2ID(pathNode[i],1));
		for(int j=0;j<theNode1.neigh;j++){ //遍历该节点相邻路口之间的道路ID
			tempID=theNode1.NeighLineID[j];
			//找到公共边
			theNode2=MapTools::GetNodeByID(_map.mapNode
							,MapTools::Code2ID(pathNode[i+1],1));
			for(int k=0;k<theNode2.neigh;k++){ //下一个节点
				tempID2=theNode2.NeighLineID[k];
				if(tempID==tempID2){         //找到公共边ID
					break;
				}
			}
			if(tempID==tempID2){
					break;
			}
		}
		
		pathWhole.push_back(MapTools::ID2Code(tempID)); //添加查询到的道路编号
	}
	pathWhole.push_back(pathNode.back());  //添加最后一个节点

	//Step 3 -----------结果赋给指定变量--------------
	_planPath.planPathQueue=pathWhole;
	_planPath.cur=0;

	//打印
	for(size_t i=0;i<_planPath.planPathQueue.size();i++){
		if(i%2==0){
			MAP_PRINT("[%d] ",_planPath.planPathQueue[i]);
		}else{
			MAP_PRINT("%d ",_planPath.planPathQueue[i]);
		}
	}
	MAP_PRINT("%s","\n");
}


void MapApp::dijkstra(int s,int e,vector<int> &pathV){	
	if(s==e){
		DEBUG_PRINT("规划路径头尾不能相同:%d \n",s);
		exit(0);
	}
		
	//Step 0 -----------把节点编号转化成adj矩阵中的索引--------------
	int sID=MapTools::Code2ID(s,1);
	int eID=MapTools::Code2ID(e,1);
	int sIndex=MapTools::GetNodeIndexByID(_map.mapNode,sID);
	int eIndex=MapTools::GetNodeIndexByID(_map.mapNode,eID);

	int tIndex=eIndex;

	//Step 1 -----------初始化算法参数--------------
	int nodeCount=_map.mapNode.size();
	vector<int> pi(nodeCount,-1);      //最短路径 前驱节点
	vector<int> S(nodeCount,0);        //集合S S[i]=1表示i节点在S中
	vector<int> d(nodeCount,INF);//原点到某一点 的最短代价
	S[sIndex]=1;
	d[sIndex]=0;

	//Step 2 -----------初始化距离数组d 和前缀数组pi --------------
	for(int i=0;i<nodeCount;i++){
		if(_map.adjMat[sIndex+i*nodeCount]<INF){
			d[i]=_map.adjMat[sIndex+i*nodeCount];
			pi[i]=sIndex;
		}
	}
 	d[sIndex]=0;

	//Step 3 -----------迭代求解最短路径--------------
	for(int i=1;i<nodeCount;i++){ //迭代 n-1 次
		int min=INF;
		int newNode=-1;     //本轮迭代新加入的节点
		for(int j=0;j<nodeCount;j++){ 
			if(S[j]!=1){       //在S集合中添加新节点				 
				if(d[j]<min){
					min=d[j];
					newNode=j;
				}
			}//end S[j]!=1
		}//end for
		//用新加入节点更新 U中的d值和pi
		if(newNode==-1) //找不到最近点了 退出迭代
			break;
		S[newNode]=1;
		for(int j=0;j<nodeCount;j++){ 
			if(S[j]!=1){  //U
				if(_map.adjMat[newNode+j*nodeCount]+d[newNode]<d[j]){ //松弛变量:更新d[j]和pi[j]
					pi[j]=newNode;
					d[j]=_map.adjMat[newNode+j*nodeCount]+d[newNode];
				}

			}
		}
	}

	//Step 4 -----------利用最短路径算法结果,获取路径编号序列--------------
	pathV.push_back(e); //加入尾编号
	int code; //要转化编号
	while(pi[tIndex]!=sIndex){
		tIndex=pi[tIndex];				//索引(0...)换回编号(1...)
		code=MapTools::ID2Code(_map.mapNode[tIndex].idself);
		pathV.push_back(code); 
	}
	pathV.push_back(s);  //等价s
	reverse(pathV.begin(),pathV.end()); //反转回正序
}

#ifdef _USING_NEW_IP_COMM
i32  MapApp::MCCallBack(void* mc_to_map, u32 size, void* args){
#endif
#ifdef _USING_OLD_IP_COMM
int  MapApp::MCCallBack(void* mc_to_map, size_t size, void* args){
#endif
#ifdef _NJUST_OS_USE_LINUX_
	NJUST_MC_STATE_INFO  *pState; //当不是状态数据时,值为NULL
	NJUST_MC_NAV_INFO    *pNav; //当不是导航信息时,值为NULL
	NJUST_MC_DRIVE_INFO  *pDrive; //当不是执行数据时,值为NULL

	//Step 1 -----------调用MC解码函数--------------
	NJUST_MC_Decode_IP_Data(mc_to_map,  //IP数据,目标模块从网络收到的数据
		size,     //pIPData数据的字节个数
		&pState, //当不是状态数据时,值为NULL
		&pNav, //当不是导航信息时,值为NULL
		&pDrive  //当不是执行数据时,值为NULL
		);

	//Step 2 -----------对导航信息进行解析--------------
	if (pNav)//收到导航信息
	{
		pthread_mutex_lock(&gMutex);
		//更新curGPS
		if(MapApp::s_GPSInfo.curLongtitude!=INITL_GPS_VALUE){  //不是第一次取值
			MapApp::s_GPSInfo.lastLongtitude=MapApp::s_GPSInfo.curLongtitude;
			MapApp::s_GPSInfo.lastlatitudel=MapApp::s_GPSInfo.curLatitude;
		}else                                               //第一次取值
		{
			MapApp::s_GPSInfo.lastLongtitude=pNav->Longitude_degree;
			MapApp::s_GPSInfo.lastlatitudel=pNav->Latitude_degree;
			MapApp::s_mapPackage.startID=pNav->navID;

		}
		MapApp::s_mapPackage.endID=pNav->navID;
		MapApp::s_GPSInfo.curLongtitude = pNav->Longitude_degree;
		MapApp::s_GPSInfo.curLatitude = pNav->Latitude_degree;
		MapTools::SaveGPSDataForOffLine(MapApp::s_GPSInfo.curLongtitude,MapApp::s_GPSInfo.curLatitude);//写离线数据
		//计算丢包率相关
		MapApp::s_mapPackage.count++;
		MapApp::s_mapPackage.startTime=NJUST_IP_get_time();

		pthread_cond_signal(&cond);
		pthread_mutex_unlock(&gMutex);
	}
#endif
	return 0;
}

#ifdef _USING_NEW_IP_COMM
i32 MapApp::MOCallBack(void* mo_to_map, u32 size, void* args){
#endif
#ifdef _USING_OLD_IP_COMM
int MapApp::MOCallBack(void* mo_to_map, size_t size, void* args){
#endif
#ifdef _NJUST_OS_USE_LINUX_
		NJUST_FROM_MO_COMMAND *pCmd = NULL;
		NJUST_FROM_MO_CFG *pCfg=NULL;
		NJUST_MO_Decode_IP_Data_CMD(mo_to_map, size, &pCmd,&pCfg);
		if(pCmd!=NULL){
			if (pCmd->cmd == NJUST_MO_COMMAND_TYPE_COMPUTER_RESTART)
			{
				NJUST_IP_req_pc_reboot();
			}
			else if (pCmd->cmd == NJUST_MO_COMMAND_TYPE_COMPUTER_SHUTDOWN)
			{
				NJUST_IP_req_pc_poweroff();
			}
			else if (pCmd->cmd == NJUST_MO_COMMAND_TYPE_MODULE_RESTART)
			{
				NJUST_IP_req_mod_reboot();
			}
			else if (pCmd->cmd == NJUST_MO_COMMAND_TYPE_MODULE_DEBUG)
			{
				//SetDebug();
			}
			else if (pCmd->cmd == NJUST_MO_COMMAND_TYPE_MODULE_RELEASE)
			{
				//SetRelease();
			}
		}
		if(pCfg!=NULL){
			for(int i=0;i<pCfg->nCFG;i++){
				//DEBUG_PRINT("cfg:%d ",pCfg->pCFG[i].cfg);
				//DEBUG_PRINT("valuse:%d \n",pCfg->pCFG[i].value);
			}
		}
#endif
		return 1;
}

//************************************
// 描述：监听PL的回调函数，主要接受PL命令
// 返回值: int
// 参数:   void * pl_to_map 数据包
// 参数:   size_t size      大小
// 参数:   void * args      额外参数
//************************************
#ifdef _USING_NEW_IP_COMM
i32 MapApp::PLCallBack(void* pl_to_map, u32 size, void* args){
#endif
#ifdef _USING_OLD_IP_COMM
int MapApp::PLCallBack(void* pl_to_map, size_t size, void* args){
#endif
#ifdef _NJUST_OS_USE_LINUX_
	NJUST_PL_TO_MAP * plcome = (NJUST_PL_TO_MAP *)pl_to_map;
	NJUST_IP_TIME time;
	NJUST_PL_MAP_COMMAND_TYPE  command;
	time = plcome->synTime;
	command = plcome->cmd;
	if (command == NJUST_PL_MAP_COMMAND_TYPE_REPLAN_ASK)
	{
		MapApp::s_WorkModel=MAP_WORK_BACKDRAW;//倒车模式
	}
#endif
	return 0;
}

void MapApp::sendRoad(double lng,double lat,int curID,int nextIndex){
#ifdef _NJUST_OS_USE_LINUX_
	NJUST_MAP_INFO_ROAD   road;
	NJUST_MAP_INFO_ROAD  *proad = &road;
	char buff[1024];
	//int index=curID-START_LINE_ID;
	int index=MapTools::GetLineIndexByID(_map.mapLine,curID);
	MapTools::StructTransformLine(&_map.mapLine[index],&proad);
	MAP_BUTTON_NOTE nextNode=_map.mapNode[nextIndex];
	double dis=	road.distToNextNodeM =MapTools::GetDistanceByGPS(lng,lat,
		nextNode.gpsx,
		nextNode.gpsy);//转化为度

	//填写前后节点的经纬度
	int lastNodeID; //道路上一个节点的ID
	if(_map.mapNode[nextIndex].idself==_map.mapLine[index].idstart){
		lastNodeID=_map.mapLine[index].idend;
	}else{
		lastNodeID=_map.mapLine[index].idstart;
	}

	MAP_BUTTON_NOTE lastNode=MapTools::GetNodeByID(_map.mapNode,lastNodeID);
	road.lastNodeGps.latitude=lastNode.gpsx;
	road.lastNodeGps.longtitude=lastNode.gpsy;
	road.nextNodeGps.latitude=nextNode.gpsx;
	road.nextNodeGps.longtitude=nextNode.gpsy;

	
	size_t indexGPS=locationGPS(lng,lat);
	//最多读20个GPS点
	int len=0;
	for(size_t i=indexGPS;i<indexGPS+20;i++){
		if(i<_GPSList.size()){
			road.nextGPSPointQueue[i-indexGPS].longtitude = _GPSList[i].x;
			road.nextGPSPointQueue[i-indexGPS].latitude =_GPSList[i].y;
			len++;
		}
	}
	road.GPSPointQueuelength = len;
	road.distToNextNodeM = dis;
	road.synTime = NJUST_IP_get_time();
	road.frameID = _frameNum;
	
	//添加障碍物
	vector<NJUST_MAP_OBSTACLE> outObs; //在特定范围内的障碍物
	MapTools::GetObsByDistance(lng,lat,
									NJUST_MAP_DETECT_OBSTACLE_RADIUS_M,
								_map.mapObs,
								outObs);
	road.obstacleQueueLength=outObs.size();
	for(size_t i=0;i<outObs.size();i++){
		road.mapObstacleQueue[i]=outObs[i];
		//MAP_PRINT("发送障碍物%d\n",i);
	}

	MapTools::NJUST_MAP_Encode_IP_Data(&road, 0, buff);
	//NJUST_IP_get_timeStr(road.synTime, Timeget1);
	
	if(_frameNum%2==0){
		if (len>3){
#ifdef _USING_NEW_IP_COMM
			NJUST_IP_send_to("", buff, 1024);
#endif
#ifdef _USING_OLD_IP_COMM
			NJUST_IP_udp_send_to("", buff, 1024);
#endif
		}
		MapTools::ms_sleep(10);
		send2Mo();
	}
#endif
}


void MapApp::sendNode(double lng,double lat,int curID,int lastIndex,int nextIndex){
#ifdef _NJUST_OS_USE_LINUX_
	NJUST_MAP_INFO_NODE  node;
	NJUST_MAP_INFO_NODE  *pnode = &node;
	char buff[1024];
	int index=MapTools::GetNodeIndexByID(_map.mapNode,curID);
	//获取节点属性
	MAP_BUTTON_NOTE theNode=MapTools::GetNodeByID(_map.mapNode,curID);
	MapTools::StructTransformNote(&theNode, &pnode);

	if(lastIndex==-1){ //没有上一个节点,也就是开始处
		node.nodepassType = NJUST_MAP_NODE_PASS_TYPE_NONE;
	}else{                     //正常判断方向
		getDirection(node,index,lastIndex,nextIndex);
	}

	size_t indexGPS=locationGPS(lng,lat);
	//最多读20个GPS点
	int len=0;
	for(size_t i=indexGPS;i<indexGPS+20;i++){
		if(i<_GPSList.size()){
			node.nextGPSPointQueue[i-indexGPS].longtitude = _GPSList[i].x;
			node.nextGPSPointQueue[i-indexGPS].latitude =_GPSList[i].y;
			len++;
		}
	}
	node.GPSPointQueuelength = len;
	node.synTime = NJUST_IP_get_time();
	node.frameID = _frameNum;

	//添加障碍物
	vector<NJUST_MAP_OBSTACLE> outObs; //在特定范围内的障碍物
	MapTools::GetObsByDistance(lng,lat,
									NJUST_MAP_DETECT_OBSTACLE_RADIUS_M,
								_map.mapObs,
								outObs);
	node.obstacleQueueLength=outObs.size();
	for(size_t i=0;i<outObs.size();i++){
		node.mapObstacleQueue[i]=outObs[i];
		//MAP_PRINT("发送障碍物%d\n",i);
	}

	//发送PL,MO
	MapTools::NJUST_MAP_Encode_IP_Data(&node, 1, buff);
	//NJUST_IP_get_timeStr(node.synTime, Timeget1);
	//
	
	if(_frameNum%2==0)
	{	
		if (len>3){
#ifdef _USING_NEW_IP_COMM
		NJUST_IP_send_to("", buff, 1024);//广播
#endif
#ifdef _USING_OLD_IP_COMM
		NJUST_IP_udp_send_to("", buff, 1024);//广播
#endif
		}
		MapTools::ms_sleep(10);		
		send2Mo();
	}
#endif
}

void MapApp::sendPLCmd(NJUST_PL_MAP_COMMAND_TYPE cmdType){
#ifdef _NJUST_OS_USE_LINUX_
		size_t len=_mapTaskNode.size();
		int size=0;
		NJUST_PL_TO_MAP cmd;
		cmd.cmdID=_nCurToMapCmdID;
		cmd.synTime = NJUST_IP_get_time();
		cmd.cmd = cmdType;
		cmd.nSize = sizeof(NJUST_PL_TO_MAP);
		cmd.endPoint.longtitude=_mapTaskNode[len-2].longtitude;//倒数第二个点
		cmd.endPoint.latitude=_mapTaskNode[len-2].latitude;
		cmd.checksum = 0;
#ifdef _USING_NEW_IP_COMM
		size = NJUST_IP_send_to("PL",(void*)&cmd,sizeof(NJUST_PL_TO_MAP));
#endif
#ifdef _USING_OLD_IP_COMM
		size = NJUST_IP_tcp_send_to("PL",(void*)&cmd,sizeof(NJUST_PL_TO_MAP));
#endif
		if (size > 0)
		{
			DEBUG_PRINT("MAP Send To PL. CMD Type:%d \n",cmdType);
		}
		else
		{
			DEBUG_PRINT("MAP Faile Send To PL. CMD Tyep:%d.",cmdType);
		}
		if(cmdType==NJUST_PL_MAP_COMMAND_TYPE_REACH_ENDPOINT){
			MAP_PRINT("发出了寻找终点命令%s","\n");
			for(size_t i=0;i<_mapTaskNode.size();i++){
				MAP_PRINT("%lu ",i);
				MAP_PRINT("%lf ",_mapTaskNode[i].longtitude);;
				MAP_PRINT("%lf \n",_mapTaskNode[i].latitude);;
			}
		}
		_nCurToMapCmdID ++;
		return ;
#endif
}


void MapApp::getDirection(NJUST_MAP_INFO_NODE &node,int curIndex,int lastIndex,int nextIndex)
{
	//报出方向
	MAP_TURN m_turn;
	//STEP2 ...算出该线的方向向量int a,b   (a,b)(经度，纬度)
	double x1=_map.mapNode[curIndex].gpsx-_map.mapNode[lastIndex].gpsx;
	double y1=_map.mapNode[curIndex].gpsy-_map.mapNode[lastIndex].gpsy;
	//STEP4 ...算出该线的方向向量int c,d   (c,d)
	double x2=_map.mapNode[nextIndex].gpsx-_map.mapNode[lastIndex].gpsx;
	double y2=_map.mapNode[nextIndex].gpsy-_map.mapNode[lastIndex].gpsy;

	//STEP5 ...求出从第一个方向向量到第二个方向向量逆时针旋转的夹角
	double degree = MapTools::GetRotateAngle(x1 / 100, y1 / 100, x2 / 100, y2 / 100);
	//MAP_PRINT("degree:%lf\n",degree);
	if ((degree < 45 && degree>0) || (degree >= 315 && degree <= 360))//直行
	{
		m_turn.turn = 0;
		m_turn.turndegree = 0;
	}
	if (degree >= 45 && degree < 135)//左拐
	{
		m_turn.turn = 1;
		m_turn.turndegree = degree;
	}
	if (degree >= 135 && degree < 180)//左Uturn
	{
		m_turn.turn = 3;
		m_turn.turndegree = 180;
	}
	if (degree >= 180 && degree < 225)//右Uturn
	{
		m_turn.turn = 4;
		m_turn.turndegree = 180;
	}
	if (degree >= 225 && degree < 315)//右拐
	{
		m_turn.turn = 2;
		m_turn.turndegree = 360 - degree;//右转的角度为锐角
	}

	switch (m_turn.turn)
	{
	case 0://直行
		node.nodepassType = NJUST_MAP_NODE_PASS_TYPE_STRAIGHTLINE;
		break;
	case 1://左转
		node.nodepassType = NJUST_MAP_NODE_PASS_TYPE_TURNLEFT;
		break;
	case 2://右转
		node.nodepassType = NJUST_MAP_NODE_PASS_TYPE_TURNRIGHT;
		break;
	case 3://左Uturn
		node.nodepassType = NJUST_MAP_NODE_PASS_TYPE_TURNAROUND;
		break;
	case 4://右Uturn
		node.nodepassType = NJUST_MAP_NODE_PASS_TYPE_TURNAROUND;
		break;
	case 5:
		node.nodepassType = NJUST_MAP_NODE_PASS_TYPE_NONE;
		break;
	}
}


void MapApp::send2Mo(){
#ifdef _NJUST_OS_USE_LINUX_
		NJUST_TO_MO_WORKSTAT moState;
		NJUST_IP_TIME timeCur;
		timeCur = NJUST_IP_get_time();
		moState.moduleID = 5; //模块ID 地图为5
		moState.myselfTimeOutMS = 2000; //模块离线时间
		moState.stat = NJUST_MO_WORK_STAT_VAILD;
		moState.PELR = (MapApp::s_mapPackage.endID - MapApp::s_mapPackage.startID + 1 - MapApp::s_mapPackage.count) * 1000 /
						(MapApp::s_mapPackage.endID - MapApp::s_mapPackage.startID + 1);
		moState .timeConsumingMS = (int)NJUST_IP_get_time_GAP_ms(MapApp::s_mapPackage.startTime, timeCur);
		moState.errCode = NJUST_MO_ERRCODE_NOERR;
		sprintf(moState.pErrMsg, "%s", "/0");
		int nByte = 0;
		void *pStat = NULL;
		pStat = NJUST_MO_Encode_STA_IP_Data(&moState, &nByte);
#ifdef _USING_NEW_IP_COMM
		NJUST_IP_send_to("MO", pStat, nByte);
#endif
#ifdef _USING_OLD_IP_COMM
		NJUST_IP_udp_send_to("MO", pStat, nByte);
#endif
#endif
}

void MapApp::recordWrite(int curID,int lastID,int sIndex){
	int i,code;
	MAP_DOUBLE_POINT  point;
	_pRecord=fopen(RECORD_FILE_NAME,"wb");
	
	//Step 1 -----------记录当前状态和状态索引--------------
	fwrite(&curID,sizeof(int),1,_pRecord);    //curID
	fwrite(&lastID,sizeof(int),1,_pRecord);   //lastID
	fwrite(&MapApp::s_WorkModel,sizeof(NJUST_MAP_WORK_MODEL),1,_pRecord);//work model
	fwrite(&sIndex,sizeof(int),1,_pRecord);   //lastID
	//Step 2 -----------记录当前规划路线--------------
	int len=_planPath.planPathQueue.size();
	fwrite(&len,sizeof(int),1,_pRecord); 
	fwrite(&_planPath.cur,sizeof(int),1,_pRecord); 
	for(i=0;i<len;i++){
		code=_planPath.planPathQueue[i];
		fwrite(&code,sizeof(int),1,_pRecord);
	}
	//Step 3 -----------记录定位的GPS序列--------------
	len=_GPSList.size();
	fwrite(&len,sizeof(int),1,_pRecord); 
	for(i=0;i<len;i++){
		point = _GPSList[i];
		fwrite(&point,sizeof(MAP_DOUBLE_POINT),1,_pRecord);
	}
	//Step 4 -----------记录历史信息--------------
	len=_historyPath.size();
	fwrite(&len,sizeof(int),1,_pRecord); 
	for(i=0;i<len;i++){
		code = _historyPath[i];
		fwrite(&code,sizeof(int),1,_pRecord);
	}
	fclose(_pRecord);
}

void MapApp::recordRead(int &curID,int &lastID,int &sIndex){
	if((ACCESS(RECORD_FILE_NAME,F_OK))!=0){
		MAP_PRINT("没有找到记录文件[%s]!\n",RECORD_FILE_NAME);
		return;
	}
	int len,code; 
	MAP_DOUBLE_POINT point;
	_pRecord=fopen(RECORD_FILE_NAME,"rb");

	//Step 1 -----------读取状态--------------
	fread(&curID,sizeof(int),1,_pRecord);    //curID
	fread(&lastID,sizeof(int),1,_pRecord);   //lastID
	fread(&MapApp::s_WorkModel,sizeof(NJUST_MAP_WORK_MODEL),1,_pRecord);//work model
	fread(&sIndex,sizeof(int),1,_pRecord);   //sIndex
	//Step 2 -----------读取当前规划路线--------------
	fread(&len,sizeof(int),1,_pRecord); 
	fread(&_planPath.cur,sizeof(int),1,_pRecord); 
	for(int i=0;i<len;i++){
		fread(&code,sizeof(int),1,_pRecord);
		_planPath.planPathQueue.push_back(code);
	}
	//Step 3 -----------读取规划的GPS点集--------------
	fread(&len,sizeof(int),1,_pRecord); 
	for(int i=0;i<len;i++){
		fread(&point,sizeof(MAP_DOUBLE_POINT),1,_pRecord);
		_GPSList.push_back(point);
	}
	//Step 4 -----------记录历史信息--------------
	fread(&len,sizeof(int),1,_pRecord); 
	for(int i=0;i<len;i++){
		fread(&code,sizeof(int),1,_pRecord);
		_historyPath.push_back(code);
	}
	fclose(_pRecord);
}

bool MapApp::checkLoaction(int curID){
	if((_planPath.cur+1)%2==1) {     //应该发现新道路
		if(curID-START_LINE_ID+1!=_planPath.planPathQueue[_planPath.cur+1]){ //新节点无效
			MAP_PRINT("定位 ID为%d\n",curID);
			MAP_PRINT("期待 ID为%d\n",_planPath.planPathQueue[_planPath.cur+1]+START_LINE_ID-1);
			return false;
		}
	}else{
	   if(curID-START_NODE_ID+1!=_planPath.planPathQueue[_planPath.cur+1]){ //新节点无效
		   MAP_PRINT("定位 ID为%d\n",curID);
		   MAP_PRINT("期待 ID为%d\n",_planPath.planPathQueue[_planPath.cur+1]+START_NODE_ID-1);
		   return false;
	  }
    }
	return true;
}


bool MapApp::locationTaskPoint(double lng,double lat,double dis){
	double tlng= _mapTaskNode.back().longtitude;
	double tlat= _mapTaskNode.back().latitude;
	double tdis=100000;
	tdis=MapTools::GetDistanceByGPS(lng,lat,tlng,tlat);
	if(tdis<dis){
		return true;
	}
	return false;
}




void MapApp::release(){
	if(_mapFile!=NULL){
		delete _mapFile;
	}
	vector<MAP_TASK_NODE_ZZ> tTasks;
	tTasks.swap(_mapTaskNode);
	vector<MAP_DOUBLE_POINT> tGPSList;
	tGPSList.swap(_GPSList);
}




