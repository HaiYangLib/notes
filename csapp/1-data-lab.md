# 实验资源
csapp实验资源，包括测试文件、实验指导书等
csapp实验资料(链接：链接：https://pan.baidu.com/s/1kz-ZaMN_ls6o55xcDZY_8A 
提取码：lfio
csapp电子书超清(链接：https://pan.baidu.com/s/1Q2iSvEDs9r0_pFF0RSGcow 
提取码：ky70)

# 准备阶段
## ubuntu虚拟机安装及设置
[linux学习连接](https://www.bilibili.com/video/BV1mW411i7Qf?from=search&seid=11668914935526301230)
[ubuntu设置连接](https://www.bilibili.com/medialist/play/ml492716966)
* 在ubuntu上安装gcc/g++编译环境
	1 更新软件列表
		```shell
		 sudo apt update
		```
	2 更新软件
		```shell
		sudo apt upgrade
		```
	3 安装g++/gcc所需要的环境
		```shell
		sudo   apt-get    bulid-essional (如果以前安装过就免了）
		```
	4 测试是否安装成功
		```shell
		gcc --version
		```
# 第一个实验 1-data-lab
* 文件解读
![datala在这里插入图片描述](https://img-blog.csdnimg.cn/20200608155038728.JPG?x-oss-processmage/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70#pic_center)
bits.c文件是实验文件，其他文件全为辅助文件（用于测试、和编译）
Makefile文件是编译配置文件,关于Makefile详情可以参考[Makefile教程](https://blog.csdn.net/weixin_38391755/article/details/80380786)
实验题在bits.c中包含,需要按照要求补全bits.c中的15个函数。
利用make指令，调用Makefile文件进行自动编译
```shell
	make
```
![编译结果](https://img-blog.csdnimg.cn/20200608155825829.JPG?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70#pic_center)
可以看到make生成了三个执行文件 btest fshow fshow ,其中btest用与对bits.c中的函数进行测试，需要调用指令
```shell
./btest bits.c
```
![测试](https://img-blog.csdnimg.cn/20200608160129897.JPG?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70#pic_center)
dlc文件用于测试bits.c中的代码是否符合要求（比如，运算符个数是否出现不满足要求的情况），dlc本身是已经编译好的可执行文件文件
利用命令
```	shell
./dlc bits.c
```
可能出现dlc不能执行的情况，这是由于dlc文件在复制过层中，权限被修改。只需要在执行前，用命令chmod 修改dlc文件权限即可
```shell
chmod 755 dlc
```

data-lab 实验答案
```c
/* 
 * CS:APP Data Lab 
 * 
 * <Please put your name and userid here>
 * 
 * bits.c - Source file with your solutions to the Lab.
 *          This is the file you will hand in to your instructor.
 *
 * WARNING: Do not include the <stdio.h> header; it confuses the dlc
 * compiler. You can still use printf for debugging without including
 * <stdio.h>, although you might get a compiler warning. In general,
 * it's not good practice to ignore compiler warnings, but in this
 * case it's OK.  
 */

#if 0
/*
 * Instructions to Students:
 *
 * STEP 1: Read the following instructions carefully.
 */

You will provide your solution to the Data Lab by
editing the collection of functions in this source file.

INTEGER CODING RULES:
 
  Replace the "return" statement in each function with one
  or more lines of C code that implements the function. Your code 
  must conform to the following style:
 
  int Funct(arg1, arg2, ...) {
      /* brief description of how your implementation works */
      int var1 = Expr1;
      ...
      int varM = ExprM;

      varJ = ExprJ;
      ...
      varN = ExprN;
      return ExprR;
  }

  Each "Expr" is an expression using ONLY the following:
  1. Integer constants 0 through 255 (0xFF), inclusive. You are
      not allowed to use big constants such as 0xffffffff.
  2. Function arguments and local variables (no global variables).
  3. Unary integer operations ! ~
  4. Binary integer operations & ^ | + << >>
    
  Some of the problems restrict the set of allowed operators even further.
  Each "Expr" may consist of multiple operators. You are not restricted to
  one operator per line.

  You are expressly forbidden to:
  1. Use any control constructs such as if, do, while, for, switch, etc.
  2. Define or use any macros.
  3. Define any additional functions in this file.
  4. Call any functions.
  5. Use any other operations, such as &&, ||, -, or ?:
  6. Use any form of casting.
  7. Use any data type other than int.  This implies that you
     cannot use arrays, structs, or unions.

 
  You may assume that your machine:
  1. Uses 2s complement, 32-bit representations of integers.
  2. Performs right shifts arithmetically.
  3. Has unpredictable behavior when shifting an integer by more
     than the word size.

EXAMPLES OF ACCEPTABLE CODING STYLE:
  /*
   * pow2plus1 - returns 2^x + 1, where 0 <= x <= 31
   */
  int pow2plus1(int x) {
     /* exploit ability of shifts to compute powers of 2 */
     return (1 << x) + 1;
  }

  /*
   * pow2plus4 - returns 2^x + 4, where 0 <= x <= 31
   */
  int pow2plus4(int x) {
     /* exploit ability of shifts to compute powers of 2 */
     int result = (1 << x);
     result += 4;
     return result;
  }

FLOATING POINT CODING RULES

For the problems that require you to implent floating-point operations,
the coding rules are less strict.  You are allowed to use looping and
conditional control.  You are allowed to use both ints and unsigneds.
You can use arbitrary integer and unsigned constants.

You are expressly forbidden to:
  1. Define or use any macros.
  2. Define any additional functions in this file.
  3. Call any functions.
  4. Use any form of casting.
  5. Use any data type other than int or unsigned.  This means that you
     cannot use arrays, structs, or unions.
  6. Use any floating point data types, operations, or constants.


NOTES:
  1. Use the dlc (data lab checker) compiler (described in the handout) to 
     check the legality of your solutions.
  2. Each function has a maximum number of operators (! ~ & ^ | + << >>)
     that you are allowed to use for your implementation of the function. 
     The max operator count is checked by dlc. Note that '=' is not 
     counted; you may use as many of these as you want without penalty.
  3. Use the btest test harness to check your functions for correctness.
  4. Use the BDD checker to formally verify your functions
  5. The maximum number of ops for each function is given in the
     header comment for each function. If there are any inconsistencies 
     between the maximum ops in the writeup and in this file, consider
     this file the authoritative source.

/*
 * STEP 2: Modify the following functions according the coding rules.
 * 
 *   IMPORTANT. TO AVOID GRADING SURPRISES:
 *   1. Use the dlc compiler to check that your solutions conform
 *      to the coding rules.
 *   2. Use the BDD checker to formally verify that your solutions produce 
 *      the correct answers.
 */


#endif
/* 
 * bitAnd - x&y using only ~ and | 
 *   Example: bitAnd(6, 5) = 4
 *   Legal ops: ~ |
 *   Max ops: 8
 *   Rating: 1
 */
int bitAnd(int x, int y) {
  return ~((~x)|(~y));
}
/* 
 * getByte - Extract byte n from word x
 *   Bytes numbered from 0 (LSB) to 3 (MSB)
 *   Examples: getByte(0x12345678,1) = 0x56
 *   Legal ops: ! ~ & ^ | + << >>
 *   Max ops: 6
 *   Rating: 2
 */
int getByte(int x, int n) {
	
	int left=n<<3;
	x>>=left;

	return x&0xff;

}
/* 
 * logicalShift - shift x to the right by n, using a logical shift
 *   Can assume that 0 <= n <= 31
 *   Examples: logicalShift(0x87654321,4) = 0x08765432
 *   Legal ops: ! ~ & ^ | + << >>
 *   Max ops: 20
 *   Rating: 3 
 */
int logicalShift(int x, int n) {
	
	int bit=0x1&(!!n);//n=0时，bit=0;
	int mask=(bit<<31)>>(n+~1+1);
	mask=~mask;

	return (x>>n)&mask;
}
/*
 * bitCount - returns count of number of 1's in word
 *   Examples: bitCount(5) = 2, bitCount(7) = 3
 *   Legal ops: ! ~ & ^ | + << >>
 *   Max ops: 40
 *   Rating: 4
 */
int bitCount(int x) {
 	int mask=0x01|0x01<<8;
	int res;
	mask=mask|mask<<16;//mask=0x01010101
	//将x分成四段，分别求1的个数
	res=mask&x;

	res+=(mask&x>>1);
	res+=mask&x>>2;
	res+=mask&x>>3;
        res+=mask&x>>4;
	res+=mask&x>>5;
        res+=mask&x>>6;
	res+=mask&x>>7;
	
	res+=res>>16;
	res+=res>>8;
  		
	return res&0xff;
}
/*
 * bang - Compute !x without using !
 *   Examples: bang(3) = 0, bang(0) = 1
 *   Legal ops: ~ & ^ | + << >>
 *   Max ops: 12
 *   Rating: 4 
 */
int bang(int x) {
	x|=x>>16;
	x|=x>>8;
	x|=x>>4;
	x|=x>>2;
	x|=x>>1;
	
  return (x&0x1)^0x1;
;}
/*
 * tmin - return minimum two's complement integer 
 *   Legal ops: ! ~ & ^ | + << >>
 *   Max ops: 4
 *   Rating: 1
 */
int tmin(void) {
	
  return (1<<31);
}
/* 
 * fitsBits - return 1 if x can be represented as an 
 *  n-bit, two's complement integer.
 *   1 <= n <= 32
 *   Examples: fitsBits(5,3) = 0, fitsBits(-4,3) = 1
 *   Legal ops: ! ~ & ^ | + << >>
 *   Max ops: 15
 *   Rating: 2
 */
int fitsBits(int x, int n) {
	//注意n可能超过31
	int is32=(n&0x20)>>5;//is32为0或1，即n的第六位
	//如果n=32，那么肯定可以表示x，所以返回1
	//如果n<32,如果 (x<<(32-n)>>(32-n))==x,说明能表示，否则不能表示。
	//考虑到不能出现负号、减号以及==
	//在n<32的情况下 用n^0x1f+1来代替32-n，用^和!代替==
	int offset=(n^0x1f)+1;
	int x_temp=x;
	//显然，若(x<<offset>>offset)==x时，!((x<<offset>>offset)^x)返回1，否则返回0
	return is32|!((x<<offset>>offset)^x_temp);

}
/* 
 * divpwr2 - Compute x/(2^n), for 0 <= n <= 30
 *  Round toward zero
 *   Examples: divpwr2(15,1) = 7, divpwr2(-33,4) = -2
 *   Legal ops: ! ~ & ^ | + << >>
 *   Max ops: 15
 *   Rating: 2
 */
int divpwr2(int x, int n) {
	//负数需要加偏置
	int flag=(0x1<<31)&x;
	int bias=(flag>>31)&((1<<n)+~1+1);
    return (x+bias)>>n;
}
/* 
 * negate - return -x 
 *   Example: negate(1) = -1.
 *   Legal ops: ! ~ & ^ | + << >>
 *   Max ops: 5
 *   Rating: 2
 */
int negate(int x) {

  return ~x+1;
}
/* 
 * isPositive - return 1 if x > 0, return 0 otherwise 
 *   Example: isPositive(-1) = 0.
 *   Legal ops: ! ~ & ^ | + << >>
 *   Max ops: 8
 *   Rating: 3
 */
int isPositive(int x) {
	//注意，x=0时，返回 0；
	int notneg=!((0x1<<31)&x);
	//if x>=0 notget=1,otherwise 0
  return notneg&!!x;
}
/* 
 * isLessOrEqual - if x <= y  then return 1, else return 0 
 *   Example: isLessOrEqual(4,5) = 1.
 *   Legal ops: ! ~ & ^ | + << >>
 *   Max ops: 24
 *   Rating: 3
 */
int isLessOrEqual(int x, int y) {
	int xs=(x>>31)&0x1;
	int ys=(y>>31)&0x1;

	int p=xs^ys;//p=1表示x y异号
	int f=p&xs;//f=1表示x<0,y>=0;
	int diff=y+(~x+1);//计算由y-x;
	diff=!((diff>>31)&0x1)&!p;//在同号的情况下计算由y-x;


	return f|diff;

}
/*
 * ilog2 - return floor(log base 2 of x), where x > 0
 *   Example: ilog2(16) = 4
 *   Legal ops: ! ~ & ^ | + << >>
 *   Max ops: 90
 *   Rating: 4
 */
int ilog2(int x) {

	int mask,res;
	//找到最高位的1 x非负
	x|=x>>16;
	x|=x>>8;
	x|=x>>4;
	x|=x>>2;
	x|=x>>1;
	x=(x>>1)+1;//此时x只含有最高位的1；
	x=~x+1;
	x=~x;
	//接下来数1
	//参考bitcount()
	mask=0x01|0x01<<8;
	mask=mask|mask<<16;
        
        res=mask&x;
        res+=mask&x>>1;
        res+=mask&x>>2;
        res+=mask&x>>3;
        res+=mask&x>>4;
        res+=mask&x>>5;
        res+=mask&x>>6;
        res+=mask&x>>7;

        res+=res>>16;
        res+=res>>8;
        return res&0xff;
	
}
/* 
 * float_neg - Return bit-level equivalent of expression -f for
 *   floating point argument f.
 *   Both the argument and result are passed as unsigned int's, but
 *   they are to be interpreted as the bit-level representations of
 *   single-precision floating point values.
 *   When argument is NaN, return argument.
 *   Legal ops: Any integer/unsigned operations incl. ||, &&. also if, while
 *   Max ops: 10
 *   Rating: 2
 */
unsigned float_neg(unsigned uf) {
	
	unsigned tmp=0x7fffffff&uf;
	int res=uf^(1<<31);
	if(tmp>0x7f800000)
		res=uf;

	return res;
}
/* 
* float_i2f - Return bit-level equivalent of expression (float) x
 *   Result is returned as unsigned int, but
 *   it is to be interpreted as the bit-level representation of a
 *   single-precision floating point values.
 *   Legal ops: Any integer/unsigned operations incl. ||, &&. also if, while
 *   Max ops: 30
 *   Rating: 4
 */
unsigned float_i2f(int x) {

        int tmp=0x80000000;
        int bias=127;
        int exp,sign=0,frac,len=0;
	int mask,exp_frac,offset,mask2;
	int round_mid,round_part;
	int exp_frac_tmp;
        if(x==tmp)
                return 0xcf000000;
        if(x==0)
                return 0;
        if(x<0)
        {
                x=-x;
                sign=1;
        }
	//printf("x=%x\n",x);
	if(x>=0x40000000)
		len=31;
	else
	{
		 while(x>=(1<<len))
      		         len++;

	}
        len-=1;//len表示x的小数长度
	//printf("len=%d\n",len);
        mask=tmp>>(31-len);
	//printf("mask=%x\n",mask);
        mask=~mask;
	//printf("mask=%x\n",mask);
        frac=mask&x;//得到小数部分
	//printf("frac=%x\n",frac);
        exp=bias+len;
	exp<<=23;
	
	//offset=23-len;
	if(len<=23)
	{	offset=23-len;
		exp_frac=exp|(frac<<offset);
	}
	else //小数部分超过23位，此时需要向偶舍入
	{
		offset=len-23;
		exp_frac=exp|(frac>>offset);
		round_mid=1<<(offset-1);
		//printf("round_mid=%x\n",round_mid);
		mask2=tmp>>(31-offset);
		mask2=~mask2;
		//printf("mask2=%x\n",mask2);
		round_part=mask2&frac;
		//printf("round_part=%x\n",round_part);
		exp_frac_tmp=exp_frac++;
		if(round_part>round_mid)
			exp_frac=exp_frac_tmp;
		if(round_part==round_mid)
		{
			if(exp_frac&0x1)
				exp_frac=exp_frac_tmp;//这样做可以减少一个运算符
		}

	}
        return (sign<<31)|exp_frac;
}



/* 
 * float_twice - Return bit-level equivalent of expression 2*f for
 *   floating point argument f.
 *   Both the argument and result are passed as unsigned int's, but
 *   they are to be interpreted as the bit-level representation of
 *   single-precision floating point values.
 *   When argument is NaN, return argument
 *   Legal ops: Any integer/unsigned operations incl. ||, &&. also if, while
 *   Max ops: 30
 *   Rating: 4
 */

unsigned float_twice(unsigned uf) {
	
	int tmp=0x80000000;
        unsigned  exp=(uf>>23)&0xff;
	unsigned  sign=tmp&uf;
	int frac=uf&0x7fffff;
	if(exp==0xff)
		return uf;
	else if(exp==0)
		frac=frac<<1;
	else if(exp==0xff-1)
	{
		exp=0xff;
		frac=0;
	}
	else
		exp++;
		

	return  sign|(exp<<23)|frac;
}
```
实验结果
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200619151603322.JPG?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70)