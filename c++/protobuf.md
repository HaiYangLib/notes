# protobuf使用教程
[转载于](https://blog.csdn.net/qq_41204464/article/details/95631781?ops_request_misc=%25257B%252522request%25255Fid%252522%25253A%252522161182002216780265453137%252522%25252C%252522scm%252522%25253A%25252220140713.130102334..%252522%25257D&request_id=161182002216780265453137&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~baidu_landing_v2~default-2-95631781.pc_search_result_hbase_insert&utm_term=protobuf%25E4%25BD%25BF%25E7%2594%25A8)
## 安装protobuf
```bash
sudo apt-get install autoconf automake libtool curl make g++ unzip git
git clone https://github.com/protocolbuffers/protobuf.git
cd protobuf
git submodule update --init --recursive
./autogen.sh
./configure
make
sudo make install
sudo ldconfig
protoc --version
```

## 生成目标文件
>***protoc --proto_path=IMPORT_PATH --cpp_out=DST_DIR --java_out=DST_DIR --python_out=DST_DIR path/to/file.proto***

>`MPORT_PATH`声明了一个.proto文件所在的具体目录。如果忽略该值，则使用当前目录。如果有多个目录则可以 对--proto_path 写多次，它们将会顺序的被访问并执行导入。`-I=IMPORT_PATH`是它的简化形式。当然也可以提供一个或多个输出路径：

>`--cpp_out` 在目标目录DST_DIR中产生C++代码，可以在 http://code.google.com/intl/zh-CN/apis/protocolbuffers/docs/reference /cpp-generated.html中查看更多。

>`--java_out` 在目标目录DST_DIR中产生Java代码，可以在 http://code.google.com/intl/zh-CN/apis/protocolbuffers/docs/reference /java-generated.html中查看更多。

>`--python_out`在目标目录 DST_DIR 中产生Python代码，可以在http://code.google.com/intl/zh-CN/apis/protocolbuffers /docs/reference/python-generated.html中查看更多。

>如果DST_DIR 是以.zip或.jar结尾的，编译器将输出结果打包成一个zip格式的归档文件。.jar将会输出一个 Java JAR声明必须的manifest文件。注：如果该输出归档文件已经存在，它将会被重写，编译器并没有做到足够的智能来为已经存在的归档文件添加新的文 件。

## 使用messages
```json
package tutorial;

message Person {
    required string name=1;
    required int32 id=2;
    optional string email=3;

    enum PhoneType {
        MOBILE=0;
        HOME=1;
        WORK=2;
    }

    message PhoneNumber {
        required string number=1;
        optional PhoneType type=2 [default=HOME];
    }

    repeated PhoneNumber phone=4;
}

message AddressBook {
    repeated Person person=1;
}
```

>1. 获取成员变量值直接采用使用成员变量名（全部为小写），
>2. 设置成员变量值，使用在成员变量名前加set_的方法。
>3. 对于普通成员变量（required和optional）提供has_方法判断变量值是否被设置；
>4. 提供clear_方法清除设置的变量值。
>5. 提供了一个mutable_方法，返回变量值的可修改指针。

```cpp
// name
inlinebool has_name()const;
inlinevoid clear_name();
inlineconst::std::string& name()const;
inlinevoid set_name(const::std::string& value);
inlinevoid set_name(constchar* value);
inline::std::string* mutable_name();

// id
inlinebool has_id()const;
inlinevoid clear_id();
inline int32_t id()const;
inlinevoid set_id(int32_t value);
 
// email
inlinebool has_email()const;
inlinevoid clear_email();
inlineconst::std::string& email()const;
inlinevoid set_email(const::std::string& value);
inlinevoid set_email(constchar* value);
inline::std::string* mutable_email();

// phone 
inlineint phone_size()const;
inlinevoid clear_phone();
inlineconst::google::protobuf::RepeatedPtrField< ::tutorial::Person_PhoneNumber >& phone() const;
inline::google::protobuf::RepeatedPtrField< ::tutorial::Person_PhoneNumber >* mutable_phone();
inlineconst::tutorial::Person_PhoneNumber& phone(int index)const;
inline::tutorial::Person_PhoneNumber* mutable_phone(int index);
inline::tutorial::Person_PhoneNumber* add_phone();
```
### 标准message方法
> 生成的.h文件中的class都继承自::google::protobuf::Message类，Message类提供了一些方法可以检查或者操作整个message，包括
```cpp
bool IsInitialized() const; //检查是否所有required变量都已经初始化；      
string DebugString() const; //返回message的可阅读的表示，主要用于调试程序；
void CopyFrom(const Person& from); //使用一个message的值覆盖本message；
void Clear();   //清空message的所有成员变量值。
```
### 编码和解码函数
>每个message类都提供了写入和读取message数据的方法，包括
```cpp
bool SerializeToString(string* output) const;把message编码进output。 
bool ParseFromString(const string& data);从string解码到message
bool SerializeToArray(char* buf,int size) const;把message编码进数组buf.
bool ParseFromArray(const char* buf,int size);把buf解码到message。此解码方法效率较ParseFromString高很多，所以一般用这种方法解码。
bool SerializeToOstream(ostream* output) const;把message编码进ostream
bool ParseFromIstream(istream* input);从istream解码到message
```
>备注：发送接收端所使用的加码解码方法不一定非得配对，即发送端用SerializeToString 接收端不一定非得用ParseFromString ，可以使用其他解码方法。
### 简单message生成的C++代码
```protobuf
  option optimize_for = LITE_RUNTIME;
  message LogonReqMessage {
          required int64 acctID = 1;
          required string passwd = 2;
      }
```
>由于我们在MyMessage文件中定义选项optimize_for的值为LITE_RUNTIME，因此由该.proto文件生成的所有C++类的父类均为::google::protobuf::MessageLite，而非::google::protobuf::Message。

>MessageLite类是Message的父类，在MessageLite中将缺少Protocol Buffer对反射的支持，而此类功能均在Message类中提供了具体的实现。使用LITE版本的Protocol Buffer。这样不仅可以得到更高编码效率，而且生成代码编译后所占用的资源也会更少。下面我们来看一下由message LogonReqMessage生成的C++类的部分声明，以及常用方法的说明性注释。
```cpp
class LogonReqMessage : public ::google::protobuf::MessageLite {
 
    public:
        LogonReqMessage();
        virtual ~LogonReqMessage();
 
 
        // implements Message ----------------------------------------------
        //下面的成员函数均实现自MessageLite中的虚函数。
        //创建一个新的LogonReqMessage对象，等同于clone。
        LogonReqMessage* New() const;
        //用另外一个LogonReqMessage对象初始化当前对象，等同于赋值操作符重载（operator=）
        void CopyFrom(const LogonReqMessage& from);
        //清空当前对象中的所有数据，既将所有成员变量置为未初始化状态。
        void Clear();
        //判断当前状态是否已经初始化。
        bool IsInitialized() const;
        //在给当前对象的所有变量赋值之后，获取该对象序列化后所需要的字节数。
        int ByteSize() const;
        //获取当前对象的类型名称。
        ::std::string GetTypeName() const;
 
 
 
        // required int64 acctID = 1;
        //下面的成员函数都是因message中定义的acctID字段而生成。
        //这个静态成员表示AcctID的标签值。命名规则是k + FieldName(驼峰规则) + FieldNumber。
        static const int kAcctIDFieldNumber = 1;
        //如果acctID字段已经被设置返回true，否则false。
        inline bool has_acctid() const;
        //执行该函数后has_acctid函数将返回false，而下面的acctid函数则返回acctID的缺省值。
        inline void clear_acctid();
        //返回acctid字段的当前值，如果没有设置则返回int64类型的缺省值。
        inline ::google::protobuf::int64 acctid() const;
        //为acctid字段设置新值，调用该函数后has_acctid函数将返回true。
        inline void set_acctid(::google::protobuf::int64 value);
 
   
 
        // required string passwd = 2;
        //下面的成员函数都是因message中定义的passwd字段而生成。这里生成的函数和上面acctid
        //生成的那组函数基本相似。因此这里只是列出差异部分。
        static const int kPasswdFieldNumber = 2;
        inline bool has_passwd() const;
        inline void clear_passwd();
        inline const ::std::string& passwd() const;
        inline void set_passwd(const ::std::string& value);
        //对于字符串类型字段设置const char*类型的变量值。
        inline void set_passwd(const char* value);
        inline void set_passwd(const char* value, size_t size);
        //可以通过返回值直接给passwd对象赋值。在调用该函数之后has_passwd将返回true。
        inline ::std::string* mutable_passwd();
        //释放当前对象对passwd字段的所有权，同时返回passwd字段对象指针。调用此函数之后，passwd字段对象
 
        //的所有权将移交给调用者。此后再调用has_passwd函数时将返回false。
        inline ::std::string* release_passwd();
 
    private:
 
        ... ...
 
    };
```
>下面是读写LogonReqMessage对象的C++测试代码和说明性注释。
```cpp
void testSimpleMessage()
 
    {
 
        printf("==================This is simple message.================\n");
        //序列化LogonReqMessage对象到指定的内存区域。
        LogonReqMessage logonReq;
        logonReq.set_acctid(20);
        logonReq.set_passwd("Hello World");
        //提前获取对象序列化所占用的空间并进行一次性分配，从而避免多次分配
        //而造成的性能开销。通过该种方式，还可以将序列化后的数据进行加密。
        //之后再进行持久化，或是发送到远端。
        int length = logonReq.ByteSize();
        char* buf = new char[length];
        logonReq.SerializeToArray(buf,length);
        //从内存中读取并反序列化LogonReqMessage对象，同时将结果打印出来。
        LogonReqMessage logonReq2;
        logonReq2.ParseFromArray(buf,length);
        printf("acctID = %I64d, password = %s\n",logonReq2.acctid(),logonReq2.passwd().c_str());
 
        delete [] buf;
 
    }
```


### 嵌套message生成的C++代码
```json
	  enum UserStatus {

          OFFLINE = 0;

          ONLINE = 1;

      }

      enum LoginResult {

          LOGON_RESULT_SUCCESS = 0;

          LOGON_RESULT_NOTEXIST = 1;

          LOGON_RESULT_ERROR_PASSWD = 2;

          LOGON_RESULT_ALREADY_LOGON = 3;

          LOGON_RESULT_SERVER_ERROR = 4;

      }

      message UserInfo {

          required int64 acctID = 1;

          required string name = 2;

          required UserStatus status = 3;

      }

      message LogonRespMessage {

          required LoginResult logonResult = 1;

          required UserInfo userInfo = 2; //这里嵌套了UserInfo消息。

      }
```
 >对于上述消息生成的C++代码，UserInfo因为只是包含了原始类型字段，因此和上例中的LogonReqMessage没有太多的差别，这里也就不在重复列出了。由于LogonRespMessage消息中嵌套了UserInfo类型的字段，在这里我们将仅仅给出该消息生成的C++代码和关键性注释。
 ```c
class LogonRespMessage : public ::google::protobuf::MessageLite {

    public:

        LogonRespMessage();

        virtual ~LogonRespMessage();

   

        // implements Message ----------------------------------------------

        ... ... //这部分函数和之前的例子一样。

       

        // required .LoginResult logonResult = 1;

        //下面的成员函数都是因message中定义的logonResult字段而生成。

        //这一点和前面的例子基本相同，只是类型换做了枚举类型LoginResult。   

        static const int kLogonResultFieldNumber = 1;

        inline bool has_logonresult() const;

        inline void clear_logonresult();

        inline LoginResult logonresult() const;

        inline void set_logonresult(LoginResult value);

       

        // required .UserInfo userInfo = 2;

        //下面的成员函数都是因message中定义的UserInfo字段而生成。

        //这里只是列出和非消息类型字段差异的部分。

        static const int kUserInfoFieldNumber = 2;

        inline bool has_userinfo() const;

        inline void clear_userinfo();

        inline const ::UserInfo& userinfo() const;

        //可以看到该类并没有生成用于设置和修改userInfo字段set_userinfo函数，而是将该工作

        //交给了下面的mutable_userinfo函数。因此每当调用函数之后，Protocol Buffer都会认为

        //该字段的值已经被设置了，同时has_userinfo函数亦将返回true。在实际编码中，我们可以

        //通过该函数返回userInfo字段的内部指针，并基于该指针完成userInfo成员变量的初始化工作。

        inline ::UserInfo* mutable_userinfo();

        inline ::UserInfo* release_userinfo();

    private:

        ... ...

    };
 ```
 >下面是读写LogonRespMessage对象的C++测试代码和说明性注释。
 ```cpp
 void testNestedMessage()

    {

        printf("==================This is nested message.================\n");

        LogonRespMessage logonResp;

        logonResp.set_logonresult(LOGON_RESULT_SUCCESS);

        //如上所述，通过mutable_userinfo函数返回userInfo字段的指针，之后再初始化该对象指针。

        UserInfo* userInfo = logonResp.mutable_userinfo();

        userInfo->set_acctid(200);

        userInfo->set_name("Tester");

        userInfo->set_status(OFFLINE);

        int length = logonResp.ByteSize();

        char* buf = new char[length];

        logonResp.SerializeToArray(buf,length);

   

        LogonRespMessage logonResp2;

        logonResp2.ParseFromArray(buf,length);

        printf("LogonResult = %d, UserInfo->acctID = %I64d, UserInfo->name = %s, UserInfo->status = %d\n"

            ,logonResp2.logonresult(),logonResp2.userinfo().acctid(),logonResp2.userinfo().name().c_str(),logonResp2.userinfo().status());

        delete [] buf;

    }
 ```
 ### repeated嵌套message生成的C++代码
 ```json
 message BuddyInfo {

          required UserInfo userInfo = 1;

          required int32 groupID = 2;

      }

      message RetrieveBuddiesResp {

          required int32 buddiesCnt = 1;

          repeated BuddyInfo buddiesInfo = 2;

      }
 ```
 >对于上述消息生成的代码，我们将只是针对RetrieveBuddiesResp消息所对应的C++代码进行详细说明，其余部分和前面小节的例子基本相同，可直接参照。而对于RetrieveBuddiesResp类中的代码，我们也仅仅是对buddiesInfo字段生成的代码进行更为详细的解释。

```cpp
class RetrieveBuddiesResp : public ::google::protobuf::MessageLite {

    public:

        RetrieveBuddiesResp();
        virtual ~RetrieveBuddiesResp();
        ... ... //其余代码的功能性注释均可参照前面的例子。
        // repeated .BuddyInfo buddiesInfo = 2;

        static const int kBuddiesInfoFieldNumber = 2;

        //返回数组中成员的数量。

        inline int buddiesinfo_size() const;

        //清空数组中的所有已初始化成员，调用该函数后，buddiesinfo_size函数将返回0。

        inline void clear_buddiesinfo();

        //返回数组中指定下标所包含元素的引用。

        inline const ::BuddyInfo& buddiesinfo(int index) const;

        //返回数组中指定下标所包含元素的指针，通过该方式可直接修改元素的值信息。

        inline ::BuddyInfo* mutable_buddiesinfo(int index);

        //像数组中添加一个新元素。返回值即为新增的元素，可直接对其进行初始化。

        inline ::BuddyInfo* add_buddiesinfo();

        //获取buddiesInfo字段所表示的容器，该函数返回的容器仅用于遍历并读取，不能直接修改。

        inline const ::google::protobuf::RepeatedPtrField< ::BuddyInfo >&

          buddiesinfo() const;

        //获取buddiesInfo字段所表示的容器指针，该函数返回的容器指针可用于遍历和直接修改。

        inline ::google::protobuf::RepeatedPtrField< ::BuddyInfo >*

          mutable_buddiesinfo();

    private:

        ... ...

    };
```
>下面是读写RetrieveBuddiesResp对象的C++测试代码和说明性注释。
```cpp
void testRepeatedMessage()

    {

        printf("==================This is repeated message.================\n");

        RetrieveBuddiesResp retrieveResp;

        retrieveResp.set_buddiescnt(2);

        BuddyInfo* buddyInfo = retrieveResp.add_buddiesinfo();

        buddyInfo->set_groupid(20);

        UserInfo* userInfo = buddyInfo->mutable_userinfo();

        userInfo->set_acctid(200);

        userInfo->set_name("user1");

        userInfo->set_status(OFFLINE);

   

        buddyInfo = retrieveResp.add_buddiesinfo();

        buddyInfo->set_groupid(21);

        userInfo = buddyInfo->mutable_userinfo();

        userInfo->set_acctid(201);

        userInfo->set_name("user2");

        userInfo->set_status(ONLINE);

   

        int length = retrieveResp.ByteSize();

        char* buf = new char[length];

        retrieveResp.SerializeToArray(buf,length);

   

        RetrieveBuddiesResp retrieveResp2;

        retrieveResp2.ParseFromArray(buf,length);

        printf("BuddiesCount = %d\n",retrieveResp2.buddiescnt());

        printf("Repeated Size = %d\n",retrieveResp2.buddiesinfo_size());

        //这里仅提供了通过容器迭代器的方式遍历数组元素的测试代码。

        //事实上，通过buddiesinfo_size和buddiesinfo函数亦可循环遍历。

        RepeatedPtrField<BuddyInfo>* buddiesInfo = retrieveResp2.mutable_buddiesinfo();

        RepeatedPtrField<BuddyInfo>::iterator it = buddiesInfo->begin();

        for (; it != buddiesInfo->end(); ++it) {

            printf("BuddyInfo->groupID = %d\n", it->groupid());

            printf("UserInfo->acctID = %I64d, UserInfo->name = %s, UserInfo->status = %d\n"

                , it->userinfo().acctid(), it->userinfo().name().c_str(),it->userinfo().status());

        }

        delete [] buf;

    }
```