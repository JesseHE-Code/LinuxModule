#include "Serial.h"

using namespace std;

#define MAXLEN 7

CSerial::CSerial()
{
	this->m_fd = 0;
	this->m_DatLen = 0;
	this->DatBuf[1499] = '\n';
	this->m_ExitThreadFlag = 0;
	this->epfd = epoll_create(6);
}

CSerial::~CSerial()
{
	this->ClosePort();
}

int CSerial::OpenPort(int PortNo, int baudrate, char databits, char stopbits, char parity)
{
	bool result = false;
	char pathname[20];
	
	sprintf(pathname, "/dev/ttyUSB%d", PortNo);
	
	this->m_fd = open(pathname, O_RDWR | O_NOCTTY | O_NDELAY);

	if (this->m_fd < 0) 							//如果文件操作符小于0表示打开文件失败。
	{
		printf("Can‘t open serial port\n");
		return result;
	}
	struct termios newtio;

	bzero(&newtio, sizeof(newtio));

	newtio.c_cflag |= CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;

	//设置波特率。
	switch (baudrate) {
	case 2400:
		cfsetispeed(&newtio, B2400);
		cfsetospeed(&newtio, B2400);
		break;
	case 4800:
		cfsetispeed(&newtio, B4800);
		cfsetospeed(&newtio, B4800);
		break;
	case 9600:
		cfsetispeed(&newtio, B9600);
		cfsetospeed(&newtio, B9600);
		break;
	case 19200:
		cfsetispeed(&newtio, B19200);
		cfsetospeed(&newtio, B19200);
		break;
	case 38400:
		cfsetispeed(&newtio, B38400);
		cfsetospeed(&newtio, B38400);
		break;
	case 115200:
		cfsetispeed(&newtio, B115200);
		cfsetospeed(&newtio, B115200);
		break;
	default:
		cfsetispeed(&newtio, B115200);
		cfsetospeed(&newtio, B115200);
	}

	//设置数据位，只支持7，8
	switch (databits) {
	case '7':
		newtio.c_cflag |= CS7;
		break;
	case '8':
		newtio.c_cflag |= CS8;
		break;
	default:
		printf("Unsupported Data_bits\n");
		return result;
	}
	
	//设置校验位
	switch (parity) {
	default:
	case 'N':
	case 'n': {
		newtio.c_cflag &= ~PARENB;
		newtio.c_iflag &= ~INPCK;
		}
		break;
	case 'o':
	case 'O': {
		newtio.c_cflag |= (PARODD | PARENB);
		newtio.c_iflag |= INPCK;
		}
		break;
	case 'e':
	case 'E': {
		newtio.c_cflag |= PARENB;
		newtio.c_cflag &= ~PARODD;
		newtio.c_iflag |= INPCK;
		}
		break;

	case 's':
	case 'S': {
		newtio.c_cflag &= ~PARENB;
		newtio.c_cflag &= ~CSTOPB;
		}
		break;
	}
	//设置停止位，值为1 or 2
	printf("stopbits:%c\n", stopbits);
	switch (stopbits) {
	case '1': {
		newtio.c_cflag &= ~CSTOPB;
		break;
		}
	case '2': {
		newtio.c_cflag |= CSTOPB;
		break;
		}
	default:
		printf("Unsupported stopbits.\n");
		return result;
	}
 
	//设置最少字符和等待时间，对于接收字符和等待时间没有特别的要求时，可设为0：
	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 0;

	//刷清输入和输出队列
	tcflush(0, TCIOFLUSH);

	//激活配置，TCSANOW表示更改后立即生效。
	if ((tcsetattr(this->m_fd, TCSANOW, &newtio)) != 0) { //判断是否激活成功。
		printf("Com set error\n");
		return result;
	}
	
	 //下面开始创建接受线程。
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	// 设置线程绑定属性
	int res = pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);
	// 设置线程分离属性
	res += pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	//创建线程
	pthread_create(&pid, &attr, ReceiveThreadFunc, (void *) this);
	
	result = true;
	return result;
}

void* CSerial::ReceiveThreadFunc(void* lparam)
{
	CSerial* serial_port = (CSerial *)lparam;
	serial_port->event.data.fd = serial_port->m_fd;
	serial_port->event.events =  EPOLLET | EPOLLIN;
	
	if (epoll_ctl(serial_port->epfd, EPOLL_CTL_ADD, serial_port->m_fd, &serial_port->event) != 0) 
	{ 
		//将读事件添加到epoll的事件队列中
		printf("set epoll error!\n");
		return NULL;
	}
	
	int i = 0, waiteNum = 0;
	//printf("Strat Receive thread!\n");
	while (true) {
		waiteNum = epoll_wait(serial_port->epfd, serial_port->events, 6, 10);
		//printf("waiteNum:%d\n", waiteNum);
		for (i = 0; i < waiteNum; i++) {
			if (serial_port->events[i].events & EPOLLIN) 
			{ 
				//判断是否有数据进入
				//接受数据
				//printf("Receive!\n");
				serial_port->ReadPort(serial_port->DatBuf, MAXLEN);
				//数据处理
				serial_port->PackagePro(serial_port->DatBuf, MAXLEN);
			}
		}
	}
	return NULL;
}

int CSerial::ReadPort( char* Buf, int ReadLen )
{
	if(this->m_fd > 0)
	{
		int len = 0;
		int rdlen = 0;
		while (true) {
			rdlen = read(this->m_fd, Buf + len, ReadLen);
			printf("rdlen:%d\n", rdlen);
			len += rdlen;
			if (len == ReadLen) {
				this->WritePort(Buf, len);
				return len;
			}
		}
	}
}

int CSerial::WritePort(char* Buf, int WriteLen)
{
	int wlen = write(this->m_fd, Buf, WriteLen);
	return wlen;
}

void CSerial::PackagePro(char* Buf, int DataLen )
{
	int i = 0;
	printf("I am ReadDataProc!I have get %d chars.\n", DataLen);
	printf("RecvData is :\t");
	for(i = 0; i < DataLen; i++)
		printf("%02x\t", Buf[i]);
	printf("\n");
}

void CSerial::ClosePort()
{
	if (this->m_fd > 0) 
	{ 
		//判断文件描述符是否存在
		close(this->m_fd);
	}
}

int main( int argc,char* argv[] )
{ 
	int i1;
	int portno, baudRate;
	char cmdline[256];
	
	CSerial m_Serial;
	
	printf( "Step2_SerialTest V1.0\n"); 
	// 解析命令行参数：串口号 波特率
	/*
	if( argc > 1 ) strcpy( cmdline, argv[1] );
	else portno = 1;
	if( argc > 2 )
	{ 
		strcat( cmdline, ' ' );
		strcat( cmdline, argv[2] );
		scanf( cmdline, '%d %d', &portno, &baudRate );
	}
	else 
	{
		baudRate = 115200;
	}*/
	portno = 0;
	baudRate = 115200;
	
	printf("port:%d baudrate:%d\n", portno, baudRate);
	//打开串口相应地启动了串口数据接收线程
	i1 = m_Serial.OpenPort( portno, baudRate, '8','1','N');
	if( i1<0 )
	{
		printf("serial open fail\n");
		return -1;
	}
	//进入主循环，这里每隔1s输出一个提示信息
	for( i1=0; i1<10000;i1++)
	{
		sleep(1);
		printf( "%d \n", i1+1);
	}
	m_Serial.ClosePort( );
	return 0; 
}




