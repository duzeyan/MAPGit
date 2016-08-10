#ifndef _NJUST_MAP_COMMUNIEN_H_
#define _NJUST_MAP_COMMUNIEN_H_


#include "NJUST_ALV_BYD_H/ALV_BYD.h"
#include"MAP_BASIC_data.h"





///
///Map模块通信工具类
///
class MapCommunion{
public:
	//初始化
	int Intialize();

	//在通信网络中 注册MAP模块
	int RegisterMap();

	//针对特定模块，注册回调函数
	int ReciveModuleMsg(const char * modulename,func_t func);

	//接收广播
	int ReciveBroadcastMsg(func_t func);
	
	//给PL发送规划路径
	int SendPL();

///////////////////////////以下为wiindows下使用代码//////////////////////////
	int RegisterOffLineSys();//读取离线数据,模拟MC模块

};

#endif