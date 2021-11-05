# socket编程

![在这里插入图片描述](https://img-blog.csdnimg.cn/20210325100123535.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70)
## 网络字节序
>小端法: 高位存在高地址, 低位存在低地址(计算机本地采用);
>大端法: 高位存在低地址, 低位存在高地址(网络通信采用);

调用库函数做**网络字节序**和**主机字节序**的转换;

```C
#include<arpa/inet.h>
uint32_t htonl(uint32_t hostlong);			//主要针对IP
uint16_t htons(uint16_t hostshort);			//主要针对port
uint32_t ntohl(uint32_t netlong);
uint16_t ntohs(uint16_t netshort);
```
早期没有**int**类型, 与之等效的是**long**类型;
 ## IP地址转换函数
 由于如`192.168.45.2`这种的IP地址为点分十进制表示, 需要转化为`uint32_t`型, 有现成的函数(IPv4和IPv6都可以转换):

```C
int inet_pton(int af,const char* src,void* dst);				//p表示点分十进制的ip,n网络上的二进制ip
const char* inet_ntop(int af,const char* src,char* dst,socklen_t size);
```

`int inet_pton(int af,const char* src,void* dst);`

参数:

- af:
1. AF_INET
2. AF_INET6
- src:传入参数, 待转换的点分十进制的IP地址;
- dst:传出参数, 转换后符合网络字节序的IP地址;

返回值:

- 成功返回1;
- 若参2无效返回0(异常);
- 失败返回-1



`const char* inet_ntop(int af,const char* src,char* dst,socklen_t size)`

参数:

- af:

1. AF_INET
2. AF_INET6

- src:传入参数, 待转换的网络字节序的IP地址;
- dst:传出参数, 转换后的点分十进制IP地址, 是一块缓冲区;
- size指定了缓冲区的大小;

返回值:

- 成功返回dst指针;
- 失败返回NULL指针, 设置errorno;

## sockaddr地址结构
bind函数:

```c
#include<sys/types.h>
#include<sys/socket.h>
int bind(int sockfd,const struct sockaddr* addr,socklen_t addrlen);
/*struct sockaddr是早已废弃的数据结构,已不再使用,用新的是注意强转一下*/
struct sockaddr_in addr;
int bind(int sockfd,(struct sockaddr*)&addr,size);
```

```c
/*相关结构体定义,在man 7 ip*/
struct sockaddr_in{
	sa_family_t		sin_family;
	in_port_t		sin_port;
	struct in_addr	sin_addr;
};
struct in_addr{
    uint32_t s_addr;
};
```

初始化方法:

```C
addr.sin_family=AF_INET/AF_INET6;
addr.sin_port=htons(9527);					//端口号为short类型(16bit)

int dst;
inet_pton(AF_INET,"192.168.10.2",(void*)&dst);
addr.sin_addr.s_addr=dst;
/*或者采取下面的方法*/
addr.sin_addr.s_addr=htonl(INADDR_ANY)		//取出系统中任意有效的IP地址
```

## socket和bind
socket(创建一个套接字):

```c
#include<sys/types.h>
#include<sys/socket.h>
int socket(int domain,int type,int protocol);
```

1.domain指定使用的协议(IPv4或IPv6)

- AF_INET;
- AF_INET6;
- AF_UNIX或AF_LOCAL;

2.type指定数据传输协议(流式或报式)

- SOCK_STREAM;
- SOCK_DGRAM;

3.指定代表协议(一般传0)

- 流式以TCP为代表;
- 报式以UDP为代表;

成功返回新套接字的fd, 失败返回-1并设置errno;



bind(给socket**绑定**一个地址结构(IP+port)):

```c
#include<sys/types.h>
#include<sys/socket.h>
int bind(int sockfd,const struct sockaddr* addr,socklen_t addrlen);


struct sockaddr_in addr;

addr.sin_family=AF_INET/AF_INET6;
addr.sin_port=htons(9527);					//端口号为short类型(16bit)

int dst;
inet_pton(AF_INET,"192.168.10.2",(void*)&dst);
addr.sin_addr.s_addr=dst;
/*或者采取下面的方法*/
addr.sin_addr.s_addr=htonl(INADDR_ANY)		//取出系统中任意有效的IP地址

/*struct sockaddr是早已废弃的数据结构,已不再使用,用新的是注意强转一下*/
int bind(int sockfd,(struct sockaddr*)&addr,size);
```

addr.family应该与domain保持一致;

`size=sizeof(addr)`--地址结构的大小;

成功返回0, 失败返回-1;

## listen和accept
listen(设置最大连接数或者说能同时进行三次握手的最大连接数):

```c
int listen(int sockfd,int backlog);
```

`sockfd`--仍然传入socket函数的返回值;

`backlog`--上限数值, 最大128;

成功返回0, 失败返回-1并设置errno;



accept(阻塞等待客户端建立连接, 成功的话返回一个**与客户端成功连接**的socket文件描述符):

```c
int accept(int sockfd,sockaddr* addr,int socklen_t* addrlen);
```

`sockfd`--socket函数的返回值;

`addr`--传出参数, 成功与Sever建立连接的那个**客户端**的地址结构;

`addrlen`--传入传出参数;

	socklen_t clit_addr_len=sizeof(addr);
	
	入: 传入addr的大小;
	
	出: 客户端addr的实际大小;

返回值:

	成功: 返回能与Server进行通信的socket对应的文件描述符;
	
	失败: 返回-1并设置errno;



## connect

connect(使用现有的socket与服务器建立连接)

```c
int connect(int sockfd,const struct sockaddr* addr,socklen_t addrlen);
```

`sockfd`--socket函数返回值;

`addr`--传入参数, **服务器**的地址结构;

`addrlen`--**服务器**地址结构的长度;

返回值:

	成功返回0;
	
	失败返回-1并设置errno;

如果不适用`bind()`函数绑定客户端的地址结构, 会采用**"隐式绑定"**;


## CS模型的TCP通信分析
Server:

1. `socket()`--创建socket;
2. `bind()`--绑定Server地址结构;
3. `listen()`--设置监听上限;
4. `accept()`--阻塞监听客户端建立连接;
5. `read()`--读socket获取客户端数据;
6. `toupper()`--事务处理;
7. `write()`--写回数据到客户端;
8. `close()`

Client:

`socket()`--创建socket;

`connect()`--与服务器建立连接;

`write()`--向socket(Server)写入数据;

`read()`--读出处理后的数据;

`close()`;

## Server的实现

```c
#include <arpa/inet.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

int SERV_PORT = 11990;
const char* ip = "127.0.0.1";

/*错误处理函数*/
void sys_err(const char* str) {
  perror(str);
  exit(1);
}

int main() {
  int dst;
  int res = inet_pton(AF_INET, ip, (void*)&dst);

  struct sockaddr_in serv_addr;
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(SERV_PORT);
  serv_addr.sin_addr.s_addr = dst;

  int link_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (link_fd == -1) {
    sys_err("socket error");
  }

  int ret =
      bind(link_fd, (const struct sockaddr*)&serv_addr, sizeof(serv_addr));
  if (ret == -1) {
    sys_err("bind error");
  }

  if (listen(link_fd, 12) == -1) {
    sys_err("listen error");
  }

  struct sockaddr_in client_addr;
  socklen_t client_addr_len = sizeof(client_addr);
  int connect_fd =
      accept(link_fd, (struct sockaddr*)&client_addr, &client_addr_len);
  if (connect_fd == -1) {
    sys_err("accept error");
  }

  char client_ip[128];
  printf("client IP:%s,client port:%d",
         inet_ntop(AF_INET, &client_addr.sin_addr.s_addr, client_ip,
                   sizeof(client_ip)),
         ntohs(client_addr.sin_port));

  char buff[1024];
  while (true) {
    int num = read(connect_fd, buff, sizeof(buff));
    printf("\nnum=%d\n", num);
    write(1, buff, num);
    for (int i = 0; i < num; i++) {
      buff[i] = toupper(buff[i]);
    }
    write(connect_fd, buff, num);
    sleep(1);
  }
  close(connect_fd);
  close(link_fd);
  return 0;
}
```

## Client的实现
```c
#include <arpa/inet.h>
#include <ctype.h>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

int SERV_PORT = 11990;
const char *ip = "127.0.0.1";

int main() {
  int dst;
  int r = inet_pton(AF_INET, ip, (void *)&dst);
  if (r == -1) {
    printf("inet_pton error\n");
    return -1;
  }

  struct sockaddr_in serv_addr;
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(SERV_PORT);
  serv_addr.sin_addr.s_addr = dst;

  int client_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (client_fd == -1) {
    printf("socket error\n");
    return -1;
  }

  int res =
      connect(client_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr));
  if (res == -1) {
    printf("connect error\n");
    return -1;
  }

  int count = 10;
  char buff[1024];
  const char *str = "hello world\n";
  for (int i = 0; i < count; i++) {
    write(client_fd, str, 12);
    int num = read(client_fd, buff, sizeof(buff));
    write(1, buff, num);  //打印到屏幕
    sleep(1);
  }

  close(client_fd);
}
```