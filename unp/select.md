# select![在这里插入图片描述](https://img-blog.csdnimg.cn/20210325130215795.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70)
>  这种思想很想CPU对IO的处理的发展历程, select的地位就像中断管理器, IO设备有中断请求时才通知CPU, 对应的, 只有当客户端有连接请求时才会通知server进行处理. 也就是说只要server收到通知, 就一定有数据待处理或连接待响应, 不会再被阻塞而浪费资源了;

## select函数
```c
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
int select(int nfds, fd_set* readfds, fd_set* writefds,fd_set* exceptfds, struct timeval* timeout);
```
nfds-所监听的套接字文件描述符的最大值+1, 在select内部用作一个for循环的上限;
fd_set*-都是传入传出参数;
- readfds-读文件描述符监听集合;
- writefds-写文件描述符监听集合;
- exceptfds-异常文件描述符监听集合;

重点在于readfds:当客户端有数据发到服务器上时, 触发服务器的读事件. 后面两个一般传NULL;

三个传入传出参数都是位图, 每个二进制位代表了一个文件描述符的状态;

传入的是你想监听的文件描述符集合(对应位置一), 传出来的是实际有事件发生的文件描述符集合(将没有事件发生的位置零);

返回值: 

- 所有你所监听的文件描述符当中有事件发生的总个数(读写异常三个参数综合考虑);
- -1说明发生异常, 设置errno;

关于timeout:
![在这里插入图片描述](https://img-blog.csdnimg.cn/2021032513085185.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70)
由于对位图的运算涉及到位操作, 系统提供了相应的函数:
```c
void FD_CLR(int fd,fd_set* set);			//将给定的套接字fd从位图set中清除出去
int FD_ISSET(int fd,fd_set* set);			//检查给定的套接字fd是否在位图里面,返回0或1
void FD_SET(int fd,fd_set* set);			//将给定的套接字fd设置到位图set中
void FD_ZERO(fd_set* set);					//将整个位图set置零
```

## select实现多路IO转接设计思路
```c
listenFd=Socket();								//创建套接字
Bind();											//绑定地址结构
Listen();										//设置监听上限
fd_set rset;									//创建读监听集合
fd_set allset;
FD_ZERO(&allset);								//将读监听集合清空
FD_SET(listenFd,&allset);						//将listenFd添加到所有读集合当中
while(1){
    rset=allset;								//保存监听集合
	ret=select(listenFd,&rset,NULL,NULL,NULL);		//监听文件描述符集合对应事件
	if(ret>0){
    	if(FD_ISSET(listenFd,&rset)){
        	cfd=accept();
        	FD_SET(cfd,&allset);					//添加到监听通信描述符集合中
    	}
        for(i=listenFd+1;i<=cfd;++i){
            FD_ISSET(i,&rset);						//有read,write事件
            read();
            toupper();
            write();
        }
	}
}
```

## select实现多路IO转接代码实现
> 服务器端
```c
#include <arpa/inet.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "wrap.h"

#define SERV_PORT 16666

int main(int argc, char *argv[]) {
  int i, j, n, maxi;

  int nready, client[FD_SETSIZE]; /* 自定义数组client, 防止遍历1024个文件描述符
                                     FD_SETSIZE默认为1024 */
  int maxfd, listenfd, connfd, sockfd;
  char buf[BUFSIZ], str[INET_ADDRSTRLEN]; /* #define INET_ADDRSTRLEN 16 */

  struct sockaddr_in clie_addr, serv_addr;
  socklen_t clie_addr_len;
  fd_set rset, allset; /* rset 读事件文件描述符集合 allset用来暂存 */

  listenfd = Socket(AF_INET, SOCK_STREAM, 0);

  int opt = 1;
  setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  bzero(&serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  //serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
   inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr);
  serv_addr.sin_port = htons(SERV_PORT);

  Bind(listenfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr));
  Listen(listenfd, 128);

  maxfd = listenfd; /* 起初 listenfd 即为最大文件描述符 */

  maxi = -1; /* 将来用作client[]的下标, 初始值指向0个元素之前下标位置 */
  for (i = 0; i < FD_SETSIZE; i++) client[i] = -1; /* 用-1初始化client[] */

  FD_ZERO(&allset);
  FD_SET(listenfd, &allset); /* 构造select监控文件描述符集 */

  while (1) {
    rset = allset; /* 每次循环时都从新设置select监控信号集 */

    nready = select(maxfd + 1, &rset, NULL, NULL, NULL);  // 2  1--lfd 1--connfd
    printf("nready=%d\n",nready);
    if (nready < 0) perr_exit("select error");

    if (FD_ISSET(listenfd, &rset)) { /* 说明有新的客户端链接请求 */

      clie_addr_len = sizeof(clie_addr);
      connfd = Accept(listenfd, (struct sockaddr *)&clie_addr,
                      &clie_addr_len); /* Accept 不会阻塞 */
      printf("received from %s at PORT %d\n",
             inet_ntop(AF_INET, &clie_addr.sin_addr, str, sizeof(str)),
             ntohs(clie_addr.sin_port));

      for (i = 0; i < FD_SETSIZE; i++)
        if (client[i] < 0) { /* 找client[]中没有使用的位置 */
          client[i] = connfd; /* 保存accept返回的文件描述符到client[]里 */
          break;
        }

      if (i == FD_SETSIZE) { /* 达到select能监控的文件个数上限 1024 */
        fputs("too many clients\n", stderr);
        exit(1);
      }

      FD_SET(connfd, &allset);
      /* 向监控文件描述符集合allset添加新的文件描述符connfd */

      if (connfd > maxfd) maxfd = connfd; /* select第一个参数需要 */

      if (i > maxi) maxi = i; /* 保证maxi存的总是client[]最后一个元素下标 */

      if (--nready == 0) continue;
    }

    for (i = 0; i <= maxi; i++) { /* 检测哪个clients 有数据就绪 */

      if ((sockfd = client[i]) < 0) continue;
      if (FD_ISSET(sockfd, &rset)) {
        if ((n = Read(sockfd, buf, sizeof(buf))) ==
            0) { /* 当client关闭链接时,服务器端也关闭对应链接 */
          Close(sockfd);
          FD_CLR(sockfd, &allset); /* 解除select对此文件描述符的监控 */
          client[i] = -1;
        } else if (n > 0) {
          for (j = 0; j < n; j++) buf[j] = toupper(buf[j]);
          Write(sockfd, buf, n);
          Write(STDOUT_FILENO, buf, n);
        }
        if (--nready == 0) break; /* 跳出for, 但还在while中 */
      }
    }
  }
  Close(listenfd);
  return 0;
}

```

> 客户端
```c

/* client.c */
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "wrap.h"

#define MAXLINE 80
#define SERV_PORT 16666

int main(int argc, char *argv[]) {
  struct sockaddr_in servaddr;
  char buf[MAXLINE];
  int sockfd, n;

  //   if (argc != 2) {
  //     printf("Enter: ./client server_IP\n");
  //     exit(1);
  //   }

  sockfd = Socket(AF_INET, SOCK_STREAM, 0);

  bzero(&servaddr, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  inet_pton(AF_INET, "127.0.0.1", &servaddr.sin_addr);
  servaddr.sin_port = htons(SERV_PORT);

  Connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr));
  printf("------------connect ok----------------\n");

  while (fgets(buf, MAXLINE, stdin) != NULL) {
    Write(sockfd, buf, strlen(buf));
    n = Read(sockfd, buf, MAXLINE);
    if (n == 0) {
      printf("the other side has been closed.\n");
      break;
    } else
      Write(STDOUT_FILENO, buf, n);
  }
  Close(sockfd);

  return 0;
}
```