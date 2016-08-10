#include"MapCommunion.h"

#include "MapModule.h"
///
/// MAP通信模块
///
#ifdef _NJUST_OS_USE_WINDOWS_
extern DWORD  WINAPI readOffLineData(LPVOID p);
#endif
//注册MAP
int MapCommunion::RegisterMap(){
#ifdef _NJUST_OS_USE_LINUX_
#ifdef _USING_NEW_IP_COMM
	if (NJUST_IP_set_moduleName("MAP",0,0)!=0)
#endif
#ifdef _USING_OLD_IP_COMM
		if (NJUST_IP_set_moduleName("MAP",0)!=0)
#endif
	{
		return 0;
	}
#endif
	return 1;
}

//针对特定模块，注册回调函数
int MapCommunion::ReciveModuleMsg(const char * modulename,func_t func){
#ifdef _NJUST_OS_USE_LINUX_
#ifdef _USING_NEW_IP_COMM
	if (-1 == NJUST_IP_set_callBack(modulename, func, NULL))
#endif
#ifdef _USING_OLD_IP_COMM
	if (-1 == NJUST_IP_set_tcp_callBack(modulename, func, NULL))
#endif
	{
		return 0;
	}
#endif
	return 1;

}

//接收广播
int MapCommunion::ReciveBroadcastMsg(func_t func){
#ifdef _NJUST_OS_USE_LINUX_
#ifdef _USING_NEW_IP_COMM
	NJUST_IP_set_brd_callBack(func, NULL);
#endif
#ifdef _USING_OLD_IP_COMM
	NJUST_IP_set_broadcast_callBack(func,NULL);
#endif
#endif
	return 1;

}

int MapCommunion::RegisterOffLineSys(){
#ifdef _NJUST_OS_USE_WINDOWS_
	CreateThread(NULL, 0,readOffLineData, NULL, 0, NULL);

#endif
	return 0;
}


