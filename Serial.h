#ifndef _SERIAL_H_  
#define _SERIAL_H_  
  
#include <stdio.h>  
#include <string.h>  
#include <sys/types.h>  
#include <sys/stat.h>
#include <sys/epoll.h>
#include <fcntl.h>  
#include <unistd.h>  
#include <termios.h>  
#include <stdlib.h>  
#include <errno.h>  
#include <pthread.h>

class CSerial
{
private:
	//通讯线程标识符ID
	pthread_t pid;
	// 串口数据接收线程
	static void* ReceiveThreadFunc( void* lparam );
public:
	CSerial();
	virtual ~CSerial();

public:
	int m_fd; 							// 已打开的串口文件描述符
	int m_DatLen;
	char DatBuf[1500];
	int m_ExitThreadFlag;
	
	int epfd; 							//epoll句柄
	epoll_event event;
	epoll_event events[6];				//事件集合
 
	// 按照指定的串口参数打开串口，并创建串口接收线程
	int OpenPort( int PortNo, int baudrate, char databits, char stopbits, char parity );
	// 关闭串口并释放相关资源
	void ClosePort( );
	// 向串口写数据
	int WritePort( char* Buf, int WriteLen );
	//读取串口数据
	int ReadPort( char* Buf, int ReadLen );
	// 接收串口数据处理函数
	virtual void PackagePro(char* Buf, int DataLen );
};

#endif