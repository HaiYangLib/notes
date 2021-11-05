# glog

[glog使用教程](https://mp.weixin.qq.com/s?src=11&timestamp=1611886875&ver=2857&signature=2QNDRwM8nnIG5iNLT54DlJMNS0lGlB55CzBwYN2fAkFCNDH4HPCQFHiXq8D0kgMVMVtQBRpLjLX09jCLcUuox70UZJnVCiLoQBpK8iJZYtwswuE*E2uhZO*rq3xWLlvy&new=1)
```cpp
#include <iostream>
#include <glog/logging.h>
#include <fcntl.h>
#include <string>

int main(int argc, char *argv[])
{
	/************************************************************
	 * 参数设置
	 * **********************************************************/
	google::InitGoogleLogging(argv[0]);		//设定日志文件名，一般与程序同名
	FLAGS_max_log_size = 10;				//设置最大日志文件大小（以MB为单位）
	FLAGS_log_dir = "../log";				//设置输出路径日志默认记录在/tmp下
	FLAGS_logtostderr = false;				//设置日志消息是否转到标准输出而不是日志文件
	FLAGS_log_prefix = true;				//设置日志前缀是否应该添加到每行输出
	FLAGS_logbufsecs = 0;					//设置可以缓冲日志的最大秒数，0指实时输出
	FLAGS_stop_logging_if_full_disk = true; //设置是否在磁盘已满时避免日志记录到磁盘

	#if NDEBUG
		std::cout << "Build-RELASE" << std::endl;
		FLAGS_alsologtostderr = false;
	#else
		std::cout << "Build-DEBUG" << std::endl;
		FLAGS_alsologtostderr = true;  //设置日志消息除日志文件之外同时输出到标准输出
		FLAGS_colorlogtostderr = true; //设置记录到标准输出的颜色消息（如果终端支持）
	#endif

	/**************************************************************************
	 * 日志输出
	 *************************************************************************/
	LOG(INFO) << "hello";
	LOG(WARNING) << "warning test";
	LOG(ERROR) << "error test";

	/**************************************************************************
	 * 条件输出
	 * ************************************************************************/
	int get_count=12;
	LOG_IF(WARNING, get_count > 10) << "Get too many times";   //当条件满足时输出日志

	for(int i=0;i<30;i++){
		// google::COUNTER 记录该语句被执行次数，从1开始，在第一次运行输出日志之后，
		//每隔 10 次再输出一次日志信息
		LOG_EVERY_N(INFO, 10) << "Got " << google::COUNTER << " times";
	}
 
	for(int i=0;i<100;i++){
		//当此语句执行的前 2 次都输出日志，然后不再输出
		LOG_FIRST_N(WARNING, 2) << "Got " << google::COUNTER << " times";
	}
	  
	for(int i=0;i<50;i++){
		//上述两者的结合，不过要注意，是先每隔 10 次去判断条件是否满足，如果是则输出日志；
		//而不是当满足某条件的情况下，每隔 10 次输出一次日志信息
		LOG_IF_EVERY_N(INFO, (i > 15), 10) << "Got the "
		 << google::COUNTER << " too big";   	
	}
	for(int i=0;i<50;i++){
		LOG_FIRST_N(WARNING, 2) << "Got the " << 
			google::COUNTER;  
		//当此语句执行的前 20 次都输出日志，然后不再输出

	}

	/************************************************************
	 * Debug输出 不会输出到日志文件中
	 ************************************************************/
	DLOG(INFO) << "Debug info";
	DLOG_IF(INFO, get_count > 10) << "Get too many times"; 
	DLOG_EVERY_N(INFO, 10) << "Got " << google::COUNTER << "times";

	/**********************************************************
	 * CHECK条件终止程序  功能类似于ASSERT 不满足条件时会终止程序
	 * ********************************************************/
	/*
	判定大小关系
		CHECK_EQ, CHECK_NE, CHECK_LE, CHECK_LT, CHECK_GE, CHECK_GT
		使用这些宏需要注意类型一致，如果出现类型不一致的，可使用static_cast转换

	判定指针是否为空
		CHECK_NOTNULL（some_ptr）

	判定字符串是否相等
		CHECK_STREQ, CHECK_STRNE 大小写敏感字符串来判定
		CHECK_STRCASEEQ, CHECK_STRCASENE 大小写不敏感字符串判定

	判定浮点是否相等或相近
		CHECK_DOUBLE_EQ，CHECK_NEAR
		这两个宏都需要指定一个可容忍的偏差上限。
	*/
	//CHECK(write(fp, ary, 4) == 4) << "Write failed!"; 
	// 出错时输出日志中包含 “write(fp, ary, 4) == 4” 和 “Write failed!”
	int * some_ptr=NULL;
	std::string str1="abc";

	CHECK_STREQ(str1.c_str(), "abc");
	CHECK_EQ(some_ptr, static_cast<int*>(NULL));



	/************************************************************
	 * perror风格日志
	 ************************************************************/
	int fp=open("tmp.txt",O_WRONLY|O_CREAT,777);
	// 当条件不成立时，会输出日志信息：
	// F0825 ...] Check failed: write(fp, NULL, 4) == 4 Write NULL failed: Bad address [X]
	//PLOG需要在函数调用后立即调用，要保证中间没有对errno产生影响。
	PCHECK(write(fp, NULL, 4) == 4) << "Write NULL failed";
	

	google::ShutdownGoogleLogging();

	return 0;
}
```
![在这里插入图片描述](assets/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FadFFFFF,t_70)