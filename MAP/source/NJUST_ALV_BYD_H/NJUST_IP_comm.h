////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Copyright (C), 2016, 南京理工大学计算机科学与工程学院, 软件工程系
//  FileName:  NJUST_IP_comm.h
//  Author: 杜鹏帧
//  Date:   2016.08.01
//  Description: 各模块之间的网络通信、处理函数
//  Functions: 
//          注册:
//          i32 NJUST_IP_set_moduleName( const i8* pModuleName, i32 reboot, i32 msec );
//          i32 NJUST_IP_moduleName_exist( const i8* pModuleName );
//          时间:
//          NJUST_IP_TIME NJUST_IP_get_time();
//          i32 NJUST_IP_get_timeStr( NJUST_IP_TIME t, char* pStr );
//			u64 NJUST_IP_get_time_GAP_ms(NJUST_IP_TIME t1, NJUST_IP_TIME t2);     //add by zqc 2015/08/05
//          通信:
//          i32 NJUST_IP_send_to( const i8* pModuleName, const void* pData, const u32 nBytes );
//          处理:
//          i32 NJUST_IP_set_callBack( const i8* pModuleName, func_t func, void* arg );
//          i32 NJUST_IP_set_brd_callBack( func_t func, void* arg );
//			请求：
//			i32 NJUST_IP_req_pc_reboot();
//			i32 NJUST_IP_req_pc_poweroff();
//			i32 NJUST_IP_req_mod_reboot();
////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef _NJUST_IP_COMM_H__20160721
#define _NJUST_IP_COMM_H__20160721


#include <limits.h>

////////////////////////////////////////////////////////////////////////////////////////////////
// 
//  设置对齐方式
//
////////////////////////////////////////////////////////////////////////////////////////////////
#pragma pack(push) //保存对齐状态
#pragma pack(1)    //按1字节对齐,必须放在第一句

typedef char               i8;
typedef unsigned char      u8;

#if USHRT_MAX == 0xffffU
typedef short              i16;
typedef unsigned short     u16;
#elif UINT_MAX == 0xffffU
typedef int                i16;
typedef unsigned int       u16;
#elif ULONG_MAX == 0xffffU
typedef long               i16;
typedef unsigned long      u16;
#endif

#if UINT_MAX == 0xffffffffU
typedef int                i32;
typedef unsigned int       u32;
#elif ULONG_MAX == 0xffffffffU
typedef long               i32;
typedef unsigned long      u32;
#elif ULLONG_MAX == 0xffffffffU
typedef long long          i32;
typedef unsigned long long u32;
#endif

#if ULONG_MAX == 0xffffffffffffffffUL
typedef long               i64;
typedef unsigned long      u64;
#elif ULLONG_MAX == 0xffffffffffffffffUL
typedef long long          i64;
typedef unsigned long long u64;
#endif

/** 模块名最大长度 */
#define NJUST_IP_MAX_MODULE_NAME_LEN          7

/** 回调函数类型 */
typedef i32 ( *func_t )( void*, u32, void* );

/** 时间类型 */
typedef struct tagNJUST_IP_TIME
{
	i32  s;         //单位万秒
	i32  ms;        //毫秒(保证不足万秒)
} NJUST_IP_TIME;


#ifdef __cplusplus
extern "C" {
#endif
////////////////////////////////////////////////////////////////////////////////////////////////
//
//  向本机守护进程发起注册
//
////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 向本机守护进程发起注册
 * @param pModuleName {const i8*} [in] 模块名
 * @param reboot {i32} [in] 重启模块
 * @param msec {i32} [in] 重启延迟时间（单位：ms）
 * @return {i32}, 成功返回0, 否则-1
 * @note 该函数必须在本通信库的其它函数前调用, 调用后不能再调用fork()
 * @note 模块名最长NJUST_IP_MAX_MODULE_NAME_LEN
 * @note 守护进程对程序状态进行检测，若reboot为0，则程序异常时不做处理。否则程序异常时，重启该模块。
 * @note 重启模块时的延迟时间，可设置为（100~200ms之间）,若reboot=0，则该参数没有任何意义
 */
i32 NJUST_IP_set_moduleName( const i8* pModuleName, i32 reboot, i32 msec );
////////////////////////////////////////////////////////////////////////////////////////////////
//
//  查询模块是否存在
//
////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 查询模块是否存在
 * @param pModuleName {const i8*} [in] 模块名
 * @return {i32}, 存在返回0, 否则-1
 * @see NJUST_IP_set_moduleName
 */
i32 NJUST_IP_moduleName_exist( const i8* pModuleName );
////////////////////////////////////////////////////////////////////////////////////////////////
//
//  获取时间戳
//
////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 获取时间戳
 */
NJUST_IP_TIME NJUST_IP_get_time();
/**
 * @brief 将时间戳转换成字符串
 * @param t {NJUST_IP_TIME} [in] 时间戳
 * @param pStr[24] {i8} [out] 字符串
 * @return {int}, 成功返回0 
*/
i32 NJUST_IP_get_timeStr( NJUST_IP_TIME t, i8 pStr[24] );
////////////////////////////////////////////////////////////////////////////////////////////////
//
//  返回两个时间差函数  
//  
//
////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 返回两个时间差函数
 * @param t1 {NJUST_IP_TIME} [in] 时间结构体 （记录从1970/01/01 到当前时刻的毫秒数）
 * @param t2 {NJUST_IP_TIME} [in] 时间结构体 （记录从1970/01/01 到当前时刻的毫秒数）
 * @return {unsigned long long}, 返回毫秒数之差（绝对值）
 * @note 该函数会返回毫秒数之差 是绝对值  
 */
u64 NJUST_IP_get_time_GAP_ms( NJUST_IP_TIME t1, NJUST_IP_TIME t2 );
////////////////////////////////////////////////////////////////////////////////////////////////
//
//  通过TCP或是UDP协议将pData指向的nBytes个字节发送至模块pModuleName
//
////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 通过TCP或是UDP协议将pData指向的nBytes个字节发送至模块pModuleName
 * @param pModuleName {const i8*} [in] 模块名
 * @param pData {const void*} [in] 数据指针
 * @param nBytes {const u32} [in] 数据长度
 * @return {i32}, 成功返回发送字节数, 否则-1 
 * @note 若pModuleName为空, 返回0
 * @see NJUST_IP_udp_send_to
 */
i32 NJUST_IP_send_to( const i8* pModuleName, const void* pData, const u32 nBytes );
////////////////////////////////////////////////////////////////////////////////////////////////
//
//  指定当前模块在收到pModuleName送达数据时的处理函数func
//
////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 指定当前模块在收到pModuleName送达数据时的处理函数func
 * @param pModuleName {const i8*} [in] 模块名
 * @param func {func_t} [in] 用于处理数据的函数
 * @param arg {void*} [in, out] 用户数据指针
 * @return {i32}, 调用失败返回-1 调用成功返回值等于0
 * @note 该函数会新创新一个线程, 处理函数的调用在新线程中进行, 请自行处理线程同步
 * @note 若未调用该函数, 对方模块在发送时会直接返回错误
 */
i32 NJUST_IP_set_callBack( const i8* pModuleName, func_t func, void* arg );

////////////////////////////////////////////////////////////////////////////////////////////////
//
//  指定当前模块在收到广播时的处理函数
//
////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 指定当前模块在收到广播到时的处理函数
 * @param func {func_t} [in] 用于处理数据的函数
 * @param arg {void*} [in, out] 用户数据指针
 * @return {i32}, 调用失败返回-1,否则返回0
 * @note 该函数会新创新一个线程, 处理函数的调用在新线程中进行, 请自行处理线程同步
 * @note 若未调用该函数, 广播依然会收到, 但直接丢弃
 */
i32 NJUST_IP_set_brd_callBack( func_t func, void* arg );
////////////////////////////////////////////////////////////////////////////////////////////////
//
//  模块请求主机重启  
//  
////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief  模块请求主机重启
 * @return {i32},成功返回0
 */
i32 NJUST_IP_req_pc_reboot();
////////////////////////////////////////////////////////////////////////////////////////////////
//
//  模块请求主机关机
//
////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief  模块请求主机关机
 * @return {i32},成功返回0
 */
i32 NJUST_IP_req_pc_poweroff();
////////////////////////////////////////////////////////////////////////////////////////////////
//
//  模块请求重启
//
////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief  模块请求重启  
 * @return {i32},成功返回0
 */
i32 NJUST_IP_req_mod_reboot();
////////////////////////////////////////////////////////////////////////////////////////////////
//
//  模块询问是否可写数据
//  Add by 赵起超   Date:2015/09/02
//
////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief  模块询问是否可写数据  
 * @return {int},若fileName文件夹下剩余可用空间大于10G 返回true 否则 返回 false
 * @example:     NJUST_IP_IsMemAvailable("/home"); 则 若home下 可用空间不足10G时 会返回false
 * 否则 可在 /home下创建数据文件 并往里面写数据
 */
int NJUST_IP_IsMemAvailable(const char *fileName);

#ifdef __cplusplus
}
#endif

////////////////////////////////////////////////////////////////////////////////////////////////
// 
//  恢复对齐方式
//
////////////////////////////////////////////////////////////////////////////////////////////////
#pragma pack(pop)//恢复对齐状态

#endif /* _NJUST_IP_COMM_H__20160721 */
