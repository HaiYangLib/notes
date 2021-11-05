散布读/聚集写
用法：#include <sys/uio.h>
函数原形：
 ssize_t readv(int filedes,const struct iovec*iov,int iovcnt);
 ssize_t writev(int filedes,const struct iovec*iov,int iovcnt);
参数：
filedes  文件描述符

iov   指向iovec结构数组的一个指针。

iovcnt  数组元素的个数
 
返回值：
 若成功则返回已读、写的字节数，若出错则返回-1
readv和writev函数用于在一次函数调用中读、写多个非连续缓冲区。有时也将这两个函数成为散布读和聚集写。

   这两个函数的第二个参数是指向iovec结构数组的一个指针：

   struct iovec{

​       void *iov_base;

​       size_t iov_len;

   };

   writev以顺序iov[0]至iov[iovcnt-1]从缓冲区中聚集输出数据。writev返回输出的字节总数，通常，它应等于所有缓冲区长度之和。

   readv则将读入的数据按上述同样顺序散布读到缓冲区中。readv总是先填满一个缓冲区，然后再填写下一个。readv返回读到的总字节数。如果遇到文件结尾，已无数据可读，则返回0。

下面就是读多个缓冲区的例子：

 

\#include <sys/uio.h>

\#include <stdio.h>

\#include <fcntl.h>

int main(int argc,char *argv[])

{

ssize_t size;

char buf1[9];

char buf2[9];

struct iovec iov[2];

 

fd1=open(argv[1],O_RDONLY);

fd2=open(argv[2],O_RDONLY);

fd3=open(argv[3],O_WRONLY);

 

size=read(fd1,buf1,sizeof(buf1));

printf(“%s size is:%d\n”,argv[1],size);

size=read(fd2,buf2,sizeof(buf2));

printf(“%s size is:%d\n”,argv[2],size);

 

iov[0].iov_base=buf1;

iov[0].iov_len=sizeof(buf1);

iov[1].iov_base=buf2;

iov[1].iov_len=sizeof(buf2);

 

size=writev(fd3,iov,2));

printf(“%s size is:%d\n”,argv[3],size);

 

close(fd1);

close(fd2);

close(fd3);

}
 


先用vi或cat建立三个文件（test1,test2,test3），test写入123456789，test写入abcdefghi.test3为空。

然后运行命令：

\#./12_4 test1 test2 test3

在屏幕上会输出：

test1 size is:9

test2 size is:9

test3 size is:18

打开test3，文件内容为123456789abcdefghi.

程序先把test1和test2的内容分别读到缓冲区buf1和buf2中。然后用write把buf1和buf2的内容写至test3.