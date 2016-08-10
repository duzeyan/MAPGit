#include"MapTools.h"
#include"MapFileStream.h"

extern FILE* gDEBUG_OUT;

///
/// MapFileStream
///

#define MAP_FILEPATH_MAX 255 //文件路径长度上限
#define MAP_STRUCT_FILENAME "board.db" //地图结构二进制文件
#define MAP_TASK_FILENAME "InitialNodeQueue.db" //解析过的路点文件
#define MAP_ADJUST_FILENAME "adjust.db" //邻接矩阵
#define MAP_TASK_NUM_MAX  1000 //任务点个数上限

//构造函数 需要传入 数据文件的目录
MapFileStream::MapFileStream(const char* loadpath){
	strcpy(this->loadpath,loadpath);
}

//加载自建地图 路点信息
void MapFileStream::LoadMapNode(NJUST_MAP_BUILD_MAP &map){
	MAP_BUILD_FILE_HEAD      mapHead;
	vector<MAP_NODE>::iterator itNode;
	vector<MAP_ROAD>::iterator itRoad;
	//中间变量
	MAP_NODE tNode;										  
	MAP_ROAD tRoad;
	MAP_BUTTON_NOTE tButtonNode;
	MAP_BUTTON_LINE tButtonLine;
	NJUST_MAP_OBSTACLE obs;

	//Step 1 -----------拼接文件路径--------------
	char filename[]=MAP_STRUCT_FILENAME;
	char path[MAP_FILEPATH_MAX];
	strcpy(path,loadpath);
	strcat(path,filename);  //拼接完整目录
	//Step 2 -----------打开文件--------------
	FILE *pFile = fopen(path, "rb");
	if(pFile==NULL){
		MAP_PRINT("打开地图结构文件失败[%s] \n",path);
		return ;
	}
	//Step 3 -----------读取文件头--------------
	fread(&mapHead,sizeof(MAP_BUILD_FILE_HEAD),1,pFile);  //读取文件头 包含了节点和道路的个数
	map.adjustPoint.x=mapHead.m_adjustx;
	map.adjustPoint.y=mapHead.m_adjusty;
	MAP_PRINT("X:%lf\n",map.adjustPoint.x);
	MAP_PRINT("Y:%lf\n",map.adjustPoint.y);
	//Step 4 -----------读取节点序列--------------
	for(int i=0;i<mapHead.notecounter;i++){              //读道路信息 路口
		fread(&tNode, sizeof(MAP_NODE), 1, pFile);
		MapTools::Node2ButtonNode(tNode,tButtonNode);
		map.mapNode.push_back(tButtonNode);
	}
	//Step 5 -----------读取道路序列--------------
	for(int i=0;i<mapHead.linecounter;i++){              //读道路信息  道路
		fread(&tRoad, sizeof(MAP_ROAD), 1, pFile);
		MapTools::Line2ButtonLine(tRoad,tButtonLine);
		map.mapLine.push_back(tButtonLine);
	}
	//Step 6 -----------读取障碍物--------------
	for(int i=0;i<mapHead.obstaclecounter;i++){              //读道路信息  道路
		fread(&obs, sizeof(NJUST_MAP_OBSTACLE), 1, pFile);
		map.mapObs.push_back(obs);
	}
	fclose(pFile);
	MAP_PRINT("完成加载地图结构信息%s","\n");
}

//加载地图中的任务路点(需要规划) 起点,路口,路口,终点,任务点
void MapFileStream::LoadMapTask(vector<MAP_TASK_NODE_ZZ> &mapTaskNode){
	//Step 1 -----------拼接文件路径--------------
	char filename[]=MAP_TASK_FILENAME;
	char path[MAP_FILEPATH_MAX];
	strcpy(path,loadpath);
	strcat(path,filename);  //拼接完整目录
	//Step 2 ----------打开文件--------------
	FILE *pf = fopen(path ,"rb");
	if(pf==NULL){
		MAP_PRINT("打开地图任务路点失败[%s]\n",path);
		return ;
	}
	//Step 3 -----------读取文件内容--------------
	MAP_TASK_NODE_ZZ buff[MAP_TASK_NUM_MAX];
	fseek(pf, 0L, SEEK_END);
	int len = ftell(pf) / sizeof(MAP_TASK_NODE_ZZ);
	fseek(pf, 0L, SEEK_SET);
    for(int i=0;i<len;i++)
	{
		fread(&(buff[i]), sizeof(MAP_TASK_NODE_ZZ), 1, pf);
		mapTaskNode.push_back(buff[i]);
    }
    fclose(pf);
	//TEST
	/*MAP_PRINT("%s待规划路径：","");
	FILE *flog=fopen("logInitNode" ,"w");
	fprintf(pf,"size:%d \n",mapTaskNode.size());
	for(unsigned int i=0;i<mapTaskNode.size();i++){
		fprintf(pf,"code %d ",mapTaskNode[i].resultCode);
		fprintf(pf,"lng %lf ",mapTaskNode[i].longtitude);
		fprintf(pf,"lat %lf ",mapTaskNode[i].latitude);
		fprintf(pf,"%s","\n");
	}
	fclose(flog);*/
	MAP_PRINT("完成读取任务点文件[%s]\n",path);
}

//加载自建地图的邻接矩阵 注意，调用本方法前必须已经调用LoadMapNode
void MapFileStream::LoadAdjMat(NJUST_MAP_BUILD_MAP &map){
	int noteCount=map.mapNode.size();
	map.adjMat.reserve(noteCount*noteCount);
	int i;
	int buff=0;

	//打开文件
    char filename[]=MAP_ADJUST_FILENAME;
	char path[MAP_FILEPATH_MAX];
	strcpy(path,loadpath);
	strcat(path,filename);  //拼接完整目录

	FILE *pf = fopen(path ,"rb");
	if(pf==NULL){
		MAP_PRINT("加载邻接矩阵文件失败%s","\n");
		return ;
	}
	//读取矩阵
	//读块内存的方法
	/*int tAdj[noteCount*noteCount];
	fread(tAdj, sizeof(int), noteCount*noteCount, pf);
	for (i = 0; i<noteCount*noteCount; i++)
	{		
		map.adjMat.push_back(tAdj[i]);
	}*/
	//按字节的方法
	for (i = 0; i<noteCount*noteCount; i++)
	{
		fread(&buff, sizeof(int), 1, pf);
		map.adjMat.push_back(buff);
	}
	fclose(pf);
	MAP_PRINT("完成加载邻接矩阵信息%s","\n");
}

//加载指定路段gps序列
void MapFileStream::ReadMapGPS(int a,int b,vector<MAP_DOUBLE_POINT> &GPSList,bool isNode){
	GPSList.clear();
	char cj=isNode?'+':'-';
	char path[MAP_FILEPATH_MAX];
	char filename[MAP_FILEPATH_MAX];
	bool isOrder=true;		//是否按照文件名顺序读取
	int GPSnum=0;
	MAP_DOUBLE_POINT tPoint;
	GPSList.reserve(2048);
	sprintf(filename,"%d%c%d.db",a,cj,b); //1-2.db or 1+2.db
	strcpy(path,loadpath);
	strcat(path,filename);  //拼接完整目录
	
	//Step 1 -----------正序打开文件--------------
	FILE *pf = fopen(path ,"rb");
	if(pf==NULL){
		MAP_PRINT("加载GPS序列文件[%s]失败,尝试逆序\n",path);
	}
	//Step 2 -----------正序失败,查找逆序文件--------------
	if (pf == NULL)
	{
		isOrder=false;
		memset(path,0,MAP_FILEPATH_MAX);
		sprintf(filename,"%d%c%d.db",b,cj,a);
		strcpy(path,loadpath);
		strcat(path,filename);  //拼接完整目录
		pf = fopen(path ,"rb");
	}
	if(pf==NULL){
		MAP_PRINT("加载[%s]文件失败(检查是否存在)\n",filename);
		return ;
	}
	//Step 3 -----------读取GPS序列--------------
	fseek(pf, 0L, SEEK_END);
	GPSnum = ftell(pf) / sizeof(MAP_DOUBLE_POINT);
	fseek(pf, 0L, SEEK_SET);
	for(int i=0;i<GPSnum;i++){
		fread(&tPoint, sizeof(MAP_DOUBLE_POINT), 1, pf);
		GPSList.push_back(tPoint);
	}
	//若逆序处理,则对GPS序列翻转
	if(!isOrder){
		reverse(GPSList.begin(),GPSList.end());
	}
	fclose(pf);
	MAP_PRINT("加载[%s]文件\n",filename);
}


