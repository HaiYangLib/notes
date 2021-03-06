# 文件IO
## 文件描述符
**是一个非负数。当打开一个现有文件或创建一个新文件时，内核向进程返回一个我文件描述符。当读写一个文件时，使用open或create返回的文件描述符标识该文件，将其作为参数传送给read或write。**
```
	- 0： 标准输入 `STDIN_FILENO
	- 1： 标准输出  STDOUT_FILENO
	- 2： 标准错误输出  STDERR_FILENO
````
## open`和`openat函数：打开文件
```
	#include<fcntl.h>
	int open(const char* path,int oflag,.../*mode_t mode*/);
	int openat(int fd,const char*path,int oflag,.../*mode_t mode */);
	
	- 参数：
		- `path`:要打开或者创建文件的名字
		- `oflag`：用于指定函数的操作行为：
			- `O_RDONLY`常量：文件只读打开
			- `O_WRONLY`常量：文件只写打开
			- `O_RDWR`常量：文件读、写打开
			- `O_EXEC`常量：只执行打开
			- `O_SEARCH`常量：只搜索打开（应用于目录）。本文涉及的操作系统都没有支持该常量
	
			在上面五个常量中必须指定且只能指定一个。下面的常量是可选的（进行或运行）：
			- `O_APPEND`：每次写时都追加到文件的尾端
			- `O_CLOEXEC`：将`FD_CLOEXEC`常量设置为文件描述符标志
			- `O_CREAT`：若此文件不存在则创建它。在使用此选项时，需要同时说明参数`mode`（指定该文件的访问权限）
			- `O_DIRECTORY`：若`path`引用的不是目录，则出错
			- `O_EXCL`：若同时指定了`O_CREAT`时，且文件已存在则出错。根据此可以测试一个文件是否存在。若不存在则创建此文件。这使得测试和创建两者成为一个原子操作
			- `O_NOCTTY`：若`path`引用的是终端设备，则不将该设备分配作为此进程的控制终端
			- `O_NOFOLLOW`：若`path`引用的是一个符号链接，则出错
			- `O_NONBLOCK`：如果`path`引用的是一个`FIFO`、一个块特殊文件或者一个字符特殊文件，则文件本次打开操作和后续的 I/O 操作设为非阻塞模式。
			- `O_SYNC`：每次 `write` 等待物理 I/O 完成，包括由 `write` 操作引起的文件属性更新所需的 I/O 
			- `O_TRUNC`： 如果此文件存在，且为`O_WRONLY`或者`O_RDWR`成功打开，则将其长度截断为0
			- `O_RSYNC`：使每一个`read`操作等待，直到所有对文件同一部分挂起的写操作都完成。
			- `O_DSYNC`：每次 `write` 等待物理 I/O 完成，但不包括由 `write` 操作引起的文件属性更新所需的 I/O 
	
		-  `mode`：文件访问权限。文件访问权限常量在 `<sys/stat.h>` 中定义，有下列九个：
			- `S_IRUSR`：用户读
			- `S_IWUSR`：用户写
			- `S_IXUSR`：用户执行
			- `S_IRGRP`：组读
			- `S_IWGRP`：组写			
			- `S_IXGRP`：组执行			
			- `S_IROTH`：其他读
			- `S_IWOTH`：其他写
			- `S_IXOTH`：其他执行 
	
	- 对于`openat`函数，被打开的文件名由`fd`和`path`共同决定：
		- 如果`path`指定的是绝对路径，此时`fd`被忽略。`openat`等价于`open`
		- 如果`path`指定的是相对路径名，则`fd`是一个目录打开的文件描述符。被打开的文件的绝对路径由该`fd`描述符对应的目录加上`path`组合而成
		- 如果`path`是一个相对路径名，而`fd`是常量`AT_FDCWD`，则`path`相对于当前工作目录。被打开文件在当前工作目录中查找。
	
	- 返回值：	
		- 成功：返回文件描述符。
		- 失败：返回 -1
	
```

**由 open/openat` 返回的文件描述符一定是最小的未使用的描述符数字。**

## creat函数：创建一个新文件

```
	#include<fcntl.h>
	int creat(const char*path,mode_t mode);
	
	- 参数：
		- `path`:要创建文件的名字
		- `mode`：指定该文件的访问权限文件访问权限常量在 `<sys/stat.h>` 中定义，有下列九个：
			- `S_IRUSR`：用户读
			- `S_IWUSR`：用户写
			- `S_IXUSR`：用户执行
			- `S_IRGRP`：组读
			- `S_IWGRP`：组写			
			- `S_IXGRP`：组执行			
			- `S_IROTH`：其他读
			- `S_IWOTH`：其他写
			- `S_IXOTH`：其他执行  
	- 返回值：
		- 成功： 返回`O_WRONLY`打开的文件描述符
		- 失败： 返回 -1
	
	该函数等价于`open(path,O_WRONLY|O_CREAT|O_TRUNC,mode)`。注意：
	- 它以只写方式打开，因此若要读取该文件，则必须先关闭，然后重新以读方式打开。
	- 若文件已存在则将文件截断为0。
```
## close函数：关闭文件
```
	#include<unistd.h>
	int close(int fd);

	
	- 参数：
		- `fd`：待关闭文件的文件描述符
	- 返回值：
		- 成功：返回 0
		- 失败：返回 -1
	
	注意：
	- 进程关闭一个文件会释放它加在该文件上的所有记录锁。
	- 当一个进程终止时，内核会自动关闭它所有的打开的文件。
```

## lseek函数：设置打开文件的偏移量
```
	#include<unistd.h>
	off_t lseek(int fd, off_t offset,int whence);
		
	- 参数：
		- `fd`：打开的文件的文件描述符
		- `whence`：必须是 `SEEK_SET`、`SEEK_CUR`、`SEEK_END`三个常量之一
		- `offset`：
			- 如果 `whence`是`SEEK_SET`，则将该文件的偏移量设置为距离文件开始处`offset`个字节
			- 如果 `whence` 是 `SEEK_CUR`，则将该文件的偏移量设置为当前值加上`offset`个字节，`offset`可正，可负
			- 如果 `whence` 是 `SEEK_END`，则将该文件的偏移量设置为文件长度加上`offset`个字节，`offset`可正，可负
	- 返回值：
		- 成功： 返回新的文件偏移量
		- 失败：返回 -1
	
	每个打开的文件都有一个与其关联的“当前文件偏移量”。它通常是个非负整数，用于度量从文件开始处计算的字节数。通常读、写操作都从当前文件偏移量处开始，并且使偏移量增加所读写的字节数。注意：
	- 打开一个文件时，除非指定`O_APPEND`选项，否则系统默认将该偏移量设为0
	- 如果文件描述符指定的是一个管道、FIFO、或者网络套接字，则无法设定当前文件偏移量，则`lseek`将返回 -1 ，并且将 `errno` 设置为 `ESPIPE`。
	- 对于普通文件，其当前文件偏移量必须是非负值。但是某些设备运行负的偏移量出现。因此比较`lseek`的结果时，不能根据它小于0 就认为出错。要根据是否等于 -1 来判断是否出错。
	- `lseek` 并不会引起任何 I/O 操作，`lseek`仅仅将当前文件的偏移量记录在内核中。
	- 当前文件偏移量可以大于文件的当前长度。此时对该文件的下一次写操作将家常该文件，并且在文件中构成一个空洞。空洞中的内容位于文件中但是没有被写过，其字节被读取时都被读为0
	- 文件中的空洞并不要求在磁盘上占据存储区。具体处理方式与操作系统有关
```
## read函数：读取文件内容
```
	#include<unistd.h>
	ssize_t read(int fd,void *buf,size_t nbytes);
		
	- 参数：
		- `fd`：打开的文件的文件描述符
		- `buf`：存放读取内容的缓冲区的地址（由程序员手动分配）
		- `nbytes`：期望读到的字节数
	- 返回值：
		- 成功：返回读到的字节数，若已到文件尾则返回 0
		- 失败：返回 -1
	
	读操作从文件的当前偏移量开始，在成功返回之前，文件的当前偏移量会增加实际读到的字节数。有多种情况可能导致实际读到的字节数少于期望读到的字节数：
	- 读普通文件时，在读到期望字节数之前到达了文件尾端
	- 当从终端设备读时，通常一次最多读取一行（终端默认是行缓冲的）
	- 当从网络读时，网络中的缓存机制可能造成返回值小于期望读到的字节数
	- 当从管道或者` FIFO `读时，若管道包含的字节少于所需的数量，则 `read`只返回实际可用的字节数
	- 当从某些面向记录的设备（如磁带）中读取时，一次最多返回一条记录
	- 当一个信号造成中断，而已读了部分数据时。
```
## write函数：向文件写数据
```
	#include<unistd.h>
	ssize_t write(int fd,const void *buf,size_t nbytes);
		
	- 参数：
		- `fd`：打开的文件的文件描述符
		- `buf`：存放待写的数据内容的缓冲区的地址（由程序员手动分配）
		- `nbytes`：期望写入文件的字节数
	- 返回值：
		- 成功：返回已写的字节数
		- 失败：返回 -1
	
	`write`的返回值通常都是与`nbytes`相同。否则表示出错。`write`出错的一个常见原因是磁盘写满，或者超过了一个给定进行的文件长度限制
	
	对于普通文件，写操作从文件的当前偏移量处开始。如果打开文件时指定了`O_APPEND`选项，则每次写操作之前，都会将文件偏移量设置在文件的当前结尾处。在一次成功写之后，该文件偏移量增加实际写的字节数。
````
## 文件共享
内核使用三种数据结构描述打开文件。它们之间的关系决定了一个进程与另一个进程在打开的文件之间的相互影响。
```
内核为每个进程分配一个进程表项（所有进程表项构成进程表），进程表中都有一个打开的文件描述符表。每个文件描述符占用一项，其内容为：
		- 文件描述符标志
		- 指向一个文件表项的指针
内核为每个打开的文件分配一个文件表项（所有的文件表项构成文件表）。每个文件表项的内容包括：
		- 文件状态标志（读、写、添写、同步和阻塞等）
		- 当前文件偏移量
		- 指向该文件 v 结点表项的指针
每个打开的文件或者设备都有一个 v 结点结构。 v 结点结构的内容包括： 
		- 文件类型和对此文件进行各种操作函数的指针。
		- 对于大多数文件， v 结点还包含了该文件的 i 结点。
```
![在这里插入图片描述](https://img-blog.csdnimg.cn/2021012014084250.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70)
这些信息都是在打开文件时从磁盘读入内存的。如 i 结点包含了文件的所有者、文件长度、指向文件实际数据在磁盘上所在位置的指针等等。 v 结点结构和 i 结点结构实际上代表了文件的实体。

现在假设进程 A 打开文件 `file1`，返回文件描述符 3；进程 B 也打开文件 `file2`，返回文件描述符 2：
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210120141237813.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70)

	- 内核在文件表上新增两个表项：
		- 这两个文件表项指向同一个 v 结点表项
		- 进程 A 、B 各自的文件描述符表项分别指向这两个文件表项；
	- 对文件的操作结果：
		- 进程 A 每次 `write` 之后，进程 A 对应的文件表项的当前文件偏移量即增加所写入的字节数。
		- 若这导致当前文件偏移量超过当前文件长度，则修改 i 节点的当前文件长度，设为当前文件偏移量
		- 如果进程 B 用 `O_APPEND`标志打开一个文件，在相应标志也设置到进程 B 对于的文件表项的文件状态标志中。
		- 每次进程 B 对具有追加写标志的文件执行写操作时，文件表项中的当前文件偏移量首先被置为 i 结点中的文件长度。
		- 若进程 B 用 `lseek` 定位到文件当前的尾端，则进程 B 对应的文件表项的当前文件偏移量设置为 i 结点中的当前长度
		- `lseek` 函数只是修改文件表项中的当前文件偏移量，不进行任何 I/O 操作

## 原子操作
### 追加到一个文件

考虑一个进程，它要将数据追加到-一个文件尾端。早期的UNIX系统版本并不支持open的O_ APPEND选项，所以程序被编写成下列形式:

```c
if (lseek(fd,OL， 2) < 0)

/*position to EOE*/

err_ sys("lseek error");

if (write(fd， buf, 100) != 100)

/*and write*/

err_ sys("write error");
```

**对单个进程而言，这段程序能正常工作，但若有多个进程同时使用这种方法将数据追加写到同一文件，则会产生问题(例如，若此程序由多个进程同时执行，各自将消息追加到-一个日志文件中，就会产生这种情况)。**

**假定有两个独立的进程A和B都对同一文件进行追加写操作。每个进程都已打开了该文件，但未使用o_ APPEND标志。此时，各数据结构之间的关系如图3-8中所示。每个进程都有它自己的文件表项，但是共享-一个v节点表项。假定进程A调用了lseek, 它将进程A的该文件当前偏移量设置为1500字节(当前文件尾端处)。然后内核切换进程,进程B运行。进程B执行lseek,也将其对该文件的当前偏移量设置为1500字节(当前文件尾端处)。然后B调用write,它将B .的该文件当前文件偏移量增加至1 600。因为该文件的长度已经增加了,所以内核将v节点中的当前文件长度更新为1 600。然后，内核又进行进程切换，使进程A恢复运行。当A调用write时，就从其当前文件偏移量(1500)处开始将数据写入到文件。这样也就覆盖了进程B刚才写入到该文件中的数据。
问题出在逻辑操作***“先定位到文件尾端，然后写”***，它使用了两个分开的函数调用。解决问题的方法是使这两个操作对于其他进程而言成为-“个原 子操作。任何要求多于一个函数 调用的操作都不是原子操作，因为在两个函数调用之间，内核有可能会临时挂起进程(正如我们前面所假定的)。UNIX系统为这样的操作提供了一种原子操作方法，即在打开文件时设置O_ APPEND 标志。**

### pread/pwrite函数 原子定位读和原子定位写
```
	#include<unistd.h>
	ssize_t pread(int fd,void*buf,size_t nbytes,off_t offset);
	ssize_t pwrite(int fd,const void*buf,size_t nbytes,off_t offset);
	
	- 参数：
		- `fd`：打开的文件描述符
		- `buf`：读出数据存放的缓冲区/ 写到文件的数据的缓冲区
		- `nbytes`：预期读出/写入文件的字节数
		- `offset`：从文件指定偏移量开始执行`read/write`
	- 返回：
		- 成功：读到的字节数/已写的字节数
		- 失败： -1
	
	调用`pread`相当于先调用`lseek`再调用`read`.但是调用`pread`时，无法中断其定位和读操作，并且不更新当前文件偏移量；
	调用`pwrite`相当于先调用`lseek`再调用`write`.但是调用`pwrite`时，但无法中断其定位和写操作，并且不更新当前文件偏移量
```


## dup和dup2函数 复制一个现有的文件描述符：
```
	#include<unistd.h>
	int dup(int fd);
	int dup2(int fd,int fd2);
	
	- 参数：
		- `fd`：被复制的文件描述符（已被打开）
		- `fd2`：指定的新的文件描述符（待生成）
	- 返回值：
		- 成功： 返回新的文件描述符
		- 失败： 返回 -1
	
	对于`dup`函数，返回的新的文件描述符一定是当前可用的文件描述符中最小的数字。对于`dup2`函数：
	- 如果 `fd2`已经是被打开的文件描述符且不等于`fd`，则先将其关闭，然后再打开（注意关闭再打开是一个原子操作）
	- 如果 `fd2`等于`fd`，则直接返回`fd2`（也等于`fd`），而不作任何操作
	
	任何情况下，这个返回的新的文明描述符与参数`fd`共享同一个文件表项（因此文件状态标志以及文件偏移量都会共享）。	
	任何情况下，这个返回的新的文明描述符的`close-on-exec`标志总是被清除
```

## sync、fsync、fdatasync函数
UNIX操作系统在内核中设有缓冲区，大多数磁盘 I/O 都通过缓冲区进行。当我们想文件写入数据时，内核通常都首先将数据复制到缓冲区中，然后排入队列，晚些时候再写入磁盘。这种方式称为延迟写。
```
	- 当内核需要重用缓冲区来存方其他数据时，它会把所有延迟写的数据库写入磁盘
	- 你也可以调用下列函数来显式的将延迟写的数据库写入磁盘
```
```
	#include<unistd.h>
	int fsync(int fd);
	int fdatasync(int fd);
	void sync(void);
```

	- 参数（前两个函数）：
		- `fd`：指定的打开的文件描述符
	
	- 返回值（前两个函数）：
		-  成功：返回 0
		- 失败： 返回 -1
	
	区别：
	- `sync`：将所有修改过的块缓冲区排入写队列，然后返回，它并不等待时机写磁盘结束
	- `fsync`：只对由`fd`指定的单个文件起作用，等待写磁盘操作结束才返回
	- `fdatasync`：只对由`fd`指定的单个文件起作用，等待写磁盘操作结束才返回，但是它只影响文件的数据部分（`fsync`会同步更新文件的属性）
	> `update` 守护进程会周期性的调用`sync`函数。命令`sync`也会调用`sync`函数
```
 