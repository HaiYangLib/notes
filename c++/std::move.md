## 右值![在这里插入图片描述](assets/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70)
>右值引用是为了解决不必要的拷贝以及使能完美转发而引入的新的引用类型。当右边的赋值类型是一个右值，左边的对象可以从右边的对象中偷取资源而不是重新分配拷贝，这个偷取的过程叫做移动语义。上述给出了事例，a+b和临时对象就是右值，右值只能出现在右边，这里的complex类和string类是由C++作者写的，引入了不同的修改和赋值，没有遵守右值的定义，所以它们的事例没有报错。
>方便记忆，可以这里理解右值和左值，可以取地址，有名字的是左值，而不能取地址，没有名字的是右值。还有一种解释，右值由将亡值和纯右值组成，将亡值如a+b赋给a后就死掉，临时对象也是一样，纯右值指的是2，'a'，true等等。

>右值出现，对其进行资源的搬移是合理的，所以引出了两点，
>第一点是要有语法告诉编译器这是右值，
>第二点是被调用段需要写出一个专门处理右值的搬移赋值（move assignment）函数。

## std::move函数实现
以下代码来自STL

```cpp
 template<typename _Tp>
    constexpr typename std::remove_reference<_Tp>::type&&
    move(_Tp&& __t) noexcept
    { return static_cast<typename std::remove_reference<_Tp>::type&&>(__t); }
    
 template<typename _Tp>
    struct remove_reference
    { typedef _Tp   type; };

 template<typename _Tp>
    struct remove_reference<_Tp&>
    { typedef _Tp   type; };

 template<typename _Tp>
    struct remove_reference<_Tp&&>
    { typedef _Tp   type; };
```
## perfect forwarding
![在这里插入图片描述](assets/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3ca,size_16,color_FFFFFF,t_70)
>关注一下forward(2)的调用，2是纯右值，调用的是forward(int&& i)函数，但在forward(int&& i)函数里面使用i，i就会变为左值，这是我们不想看到的，左值意味着可能会有不必要的拷贝，所以有perfect forwarding.

![在这里插入图片描述](assets/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODYasc3,size_16,color_FFFFFF,t_70)

>perfect forwarding可以允许你写一个函数模板，有任意个参数，透明地转发给另一个函数，其中**参数的本质（可修改性，const，左值，右值）都会在转发过程中保留下来**，使用的是std::forward模板函数。

std::forward实现,来自STL
```cpp


template<typename _Tp>
    constexpr _Tp&&
    forward(typename std::remove_reference<_Tp>::type& __t) noexcept
    { return static_cast<_Tp&&>(__t); }
    
template<typename _Tp>
    constexpr _Tp&&
    forward(typename std::remove_reference<_Tp>::type&& __t) noexcept
    {
      static_assert(!std::is_lvalue_reference<_Tp>::value, "template argument"
		    " substituting _Tp is an lvalue reference type");
      return static_cast<_Tp&&>(__t);
    }
```

## 范例
```cpp
#include <iostream>
#include <cstring>
#include <string>
#include <vector>
#include <list>
#include <bits/stl_iterator_base_types.h>


class MyString{
public:
    static size_t dctor_;//累计默认构造函数调用次数
    static size_t ctor_; //累计构造函数
    static size_t cctor_;//累计拷贝构造函数
    static size_t casgn_;//累计拷贝赋值函数
    static size_t mctor_;//累计移动构造函数
    static size_t masgn_;//累计移动赋值函数
    static size_t dtor_; //累计析构函数
private:
    char * data_;
    size_t len_;

    void InitData(const  char *s){
        data_=new char[len_+1];
        memcpy(data_,s,len_);
        data_[len_]='\0';
    }
public:
    //默认构造函数
    MyString():data_(NULL),len_(0){dctor_++;};
    //构造函数
    MyString(const char *p):len_(strlen(p)){
        ctor_++;
        InitData(p);
    }
    //拷贝构造函数
    MyString(const MyString& str):len_(str.len_){
        cctor_++;
        InitData(str.data_);
    }
    // //移动构造函数 （move constructor）
    // MyString(MyString && str)noexcept:
    //         len_(str.len_),
    //         data_(str.data_){
    //     mctor_++;
    //     str.len_=0;
    //     str.data_=NULL;
    // }
    //拷贝赋值函数（copy assignment）
    MyString& operator=(const MyString& str){
        casgn_++;
        if(this!=&str){//检查是否为同一个
            if(data_) delete data_;
            len_=str.len_;
            InitData(str.data_);
        }
        return *this;
    }
    //移动赋值函数(move assignment)
    MyString& operator=(MyString&& str)noexcept{
        masgn_++;
        if(this!=&str){
            if(data_)delete data_;
            len_=str.len_;
            data_=str.data_;

            str.len_=0;
            str.data_=NULL;
        }
        return *this; 
    }

    virtual ~MyString(){
        dtor_++;
        if(data_)delete data_;
    }
};

size_t MyString::dctor_=0;//累计默认构造函数调用次数
size_t MyString::ctor_=0; //累计构造函数
size_t MyString::cctor_=0;//累计拷贝构造函数
size_t MyString::casgn_=0;//累计拷贝赋值函数
size_t MyString::mctor_=0;//累计移动构造函数
size_t MyString::masgn_=0;//累计移动赋值函数
size_t MyString::dtor_=0;//累计析构函数


template<typename T>
void Test(T t,std::string s){

    typedef typename 
        std::iterator_traits<typename T::iterator>
            ::value_type V_type;

    size_t n=1000000;
    char buf[20];
     
    for(int i=0;i<n;i++){
        sprintf(buf,"%d",rand());
        auto it=t.end();
        MyString tmp(buf);
        
        //t.insert(it, std::move(tmp));
        t.insert(it,tmp);
    }
    std::cout<<s<<std::endl;
    std::cout<<"not std::move"<<std::endl;
    std::cout<<"MyString::dctor_="<<MyString::dctor_<<std::endl;
    std::cout<<"MyString::ctor_="<<MyString::ctor_<<std::endl;
    std::cout<<"MyString::cctor_="<<MyString::cctor_<<std::endl;
    std::cout<<"MyString::casgn_="<<MyString::casgn_<<std::endl;
    std::cout<<"MyString::mctor_="<<MyString::mctor_<<std::endl;
    std::cout<<"MyString::masgn_="<<MyString::masgn_<<std::endl;
    std::cout<<"MyString::dtor_="<<MyString::dtor_<<std::endl;
    std::cout<<std::endl;
}
template<typename T>
using  Vec= std::vector<T>;

template<typename T>
using  Lst= std::list<T>;
 

int main(){
    
    // Vec<MyString> v;
    // Lst<MyString> l;

    std::vector<MyString> v;
    //std::list<MyString> l;
    Test(v,"vector is testing");
    //Test(l,"list is testing");
}
```
>由于未写移动函数构造，所以STL在扩张的过程中调用对象的拷贝构造函数

![在这里插入图片描述](assets/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTdvsY2ODY3,size_16,color_FFFFFF,t_70)
增加移动构造函数

```cpp
 //移动构造函数 （move constructor）
    MyString(MyString && str)noexcept:
            len_(str.len_),
            data_(str.data_){
        mctor_++;
        str.len_=0;
        str.data_=NULL;
    }
```
插入代码段不变
```cpp
for(int i=0;i<n;i++){
        sprintf(buf,"%d",rand());
        auto it=t.end();
        MyString tmp(buf);
        
        //t.insert(it, std::move(tmp));
        t.insert(it,tmp);
    }
```



>由于写了移动构造函数，STL的vector在扩张的过程中使用移动构造函数。但在插入过程中还是使用拷贝构造函数。

![在这里插入图片描述](assets/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTsvfY2ODY3,size_16,color_FFFFFF,t_70)
插入代码改为，并且删去移动构造函数

```cpp
for(int i=0;i<n;i++){
        sprintf(buf,"%d",rand());
        auto it=t.end();
        MyString tmp(buf);
        
        t.insert(it, std::move(tmp));
        //t.insert(it,tmp);
    }
```
>在没有移动构造函数的情况下，即使写了std::move(tmp)，vector也无法真正实现移动，实际上调用的还是拷贝构造函数。

![在这里插入图片描述](assets/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4dfOTY2ODY3,size_16,color_FFFFFF,t_70)
加上移动构造函数
![在这里插入图片描述](assets/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2fbODY3,size_16,color_FFFFFF,t_70)





## 参考链接
[C++之万能引用、完美转发、引用折叠](https://mp.weixin.qq.com/s?src=11&timestamp=1612757195&ver=2877&signature=TvghYahxLDPDTVaeUGBSbHgoXBGljE5gKcgjRCPIBoGU-5h0wrSFjVjUSnBhyhxQzSrwBGN-Q*XRsJEsnAtY1qxqGHnqMzAGFmDBTYCunny-3dCqAYIqdBI9gBS3FHHO&new=1)

[右值经过T&&传递类型保持不变还是右值，而左值经过T&&变为普通的左值引用.](https://blog.csdn.net/p942005405/article/details/84644069)