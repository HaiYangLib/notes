# shared_ptr



参考自https://www.jianshu.com/p/b6ac02d406a0

> shared_ptr类几乎什么都没有做，它是继承了__shared_ptr, __shared_ptr内部有一个类型为__shared_count类型的成员_M_refcount, __shared_count内部有类型为_Sp_counted_base*的_M_pi的成员， _Sp_counted_base才是整个shared_ptr功能的核心，通过_Sp_counted_base控制引用计数来管理托管的内存，由图可见_Sp_counted_base内部不持有托管内存的指针，这里__shared_count内部的成员其实是一个继承自_Sp_counted_base的_Sp_counted_ptr类型，_Sp_counted_ptr类型内部持有托管内存的指针_M_ptr, _M_pi是一个_Sp_counted_base基类对象指针，指向_Sp_counted_ptr子类对象内存，这样_M_pi内部就既可以控制引用计数，又可以在最后释放托管内存。



> **这里称_M_pi为管理对象，它内部的_M_ptr为托管对象**，管理同一块托管对象的多个shared_ptr内部**共用**一个管理对象(_M_pi), 这里的多个shared_ptr可能是通过第一个shared_ptr拷贝或者移动而来, 管理对象内部有两个成员变量_M_use_count和_M_weak_count, _M_use_count表示托管对象的引用计数，控制托管对象什么时候析构和释放，大概就是有N个shared_ptr的拷贝那引用计数就是N，当引用计数为0时调用托管对象的析构函数且释放内存。_M_weak_count表示管理对象的引用计数，管理对象也是一个内存指针，**这块指针是初始化第一个shared_ptr时new出来的**，到最后也需要delete，所以使用_M_weak_count来控制管理对象什么时候析构，我们平时用到的weak_ptr内部其实持有的就是这个管理对象的指针，当weak_ptr拷贝时，管理对象的引用计数_M_weak_count就会增加，当_M_weak_count为0时，管理对象_M_pi就会析构且释放内存。



## class shared_ptr 

```cpp
template <typename _Tp> class shared_ptr : public __shared_ptr<_Tp>
```

### 成员函数

```cpp
template <typename _Tp1>
shared_ptr &operator=(const shared_ptr<_Tp1> &__r) noexcept {
  this->__shared_ptr<_Tp>::operator=(__r);
  return *this;
}
```



## class __shared_ptr

```cpp
  template <typename _Tp, _Lock_policy _Lp> class __shared_ptr
```

### 成员变量

```cpp
_Tp *_M_ptr;                     // Contained pointer.
__shared_count<_Lp> _M_refcount; // Reference counter.
```
### 成员函数

```cpp
   // 重载赋值运算符 
   __shared_ptr &operator=(const __shared_ptr &) noexcept = default;

   template <typename _Tp1>
    __shared_ptr &operator=(const __shared_ptr<_Tp1, _Lp> &__r) noexcept {
      _M_ptr = __r._M_ptr;
      _M_refcount = __r._M_refcount; // __shared_count::op= doesn't throw
      return *this;
    }


  // 构造函数
   template <typename _Tp1>
    explicit __shared_ptr(_Tp1 *__p) : _M_ptr(__p), _M_refcount(__p) {
      __glibcxx_function_requires(
          _ConvertibleConcept<_Tp1 *, _Tp *>) static_assert(sizeof(_Tp1) > 0,
                                                            "incomplete type");
      __enable_shared_from_this_helper(_M_refcount, __p, __p);
    }

	// 析构函数
    ~__shared_ptr() = default;

```

## class __shared_count 

### 成员变量

```cpp
_Sp_counted_base<_Lp> *_M_pi;//_M_pi是一个_Sp_counted_base基类对象指针，指向_Sp_counted_ptr子类对象内存，这样_M_pi内部就既可以控制引用计数，又可以在最后释放托管内存。
```

### 成员函数

```cpp
   // 构造函数
  template <typename _Ptr> explicit __shared_count(_Ptr __p) : _M_pi(0) {
      __try {
        _M_pi = new _Sp_counted_ptr<_Ptr, _Lp>(__p); //   _M_pi指向_Sp_counted_ptr子类对象内存
      }
      __catch(...) {
        delete __p;
        __throw_exception_again;
      }
    }

  // 析构函数
    ~__shared_count() noexcept {
      if (_M_pi != nullptr)
        _M_pi->_M_release();
    }
 


   // 重载赋值运算符
   __shared_count &operator=(const __shared_count &__r) noexcept {
      _Sp_counted_base<_Lp> *__tmp = __r._M_pi;
      if (__tmp != _M_pi) {
        if (__tmp != 0)
          __tmp->_M_add_ref_copy();
        if (_M_pi != 0)
          _M_pi->_M_release();
        _M_pi = __tmp;
      }
      return *this;
    }


    explicit operator bool() const // never throws
    {
      return _M_ptr == 0 ? false : true;
    }

```



## class _Sp_counted_base

```cpp
template <_Lock_policy _Lp = __default_lock_policy>
class _Sp_counted_base : public _Mutex_base<_Lp>
```

### 成员变量

```cpp
private:
  _Atomic_word _M_use_count;  // #shared
  _Atomic_word _M_weak_count; // #weak + (#shared != 0)
```

### 成员函数

```cpp
 _Sp_counted_base() noexcept : _M_use_count(1), _M_weak_count(1) {}
//注意当shared_ptr拷贝或者移动时_M_weak_count是不会增加的，它表示的是管理对象的计数，只有当__M_use_count为0时_M_weak_count才会减1，除此之外_M_weak_count的数值是由weak_ptr控制的。

virtual ~_Sp_counted_base() noexcept {}


 void _M_add_ref_copy() {
      __gnu_cxx::__atomic_add_dispatch(&_M_use_count, 1);
    }


 void _M_release() noexcept {
      // Be race-detector-friendly.  For more info see bits/c++config.
      _GLIBCXX_SYNCHRONIZATION_HAPPENS_BEFORE(&_M_use_count);
      if (__gnu_cxx::__exchange_and_add_dispatch(&_M_use_count, -1) == 1) {
        _GLIBCXX_SYNCHRONIZATION_HAPPENS_AFTER(&_M_use_count);
        _M_dispose();
        // There must be a memory barrier between dispose() and destroy()
        // to ensure that the effects of dispose() are observed in the
        // thread that runs destroy().
        // See http://gcc.gnu.org/ml/libstdc++/2005-11/msg00136.html
        if (_Mutex_base<_Lp>::_S_need_barriers) {
          _GLIBCXX_READ_MEM_BARRIER;
          _GLIBCXX_WRITE_MEM_BARRIER;
        }

        // Be race-detector-friendly.  For more info see bits/c++config.
        _GLIBCXX_SYNCHRONIZATION_HAPPENS_BEFORE(&_M_weak_count);
        if (__gnu_cxx::__exchange_and_add_dispatch(&_M_weak_count, -1) == 1) {
          _GLIBCXX_SYNCHRONIZATION_HAPPENS_AFTER(&_M_weak_count);
          _M_destroy();
        }
      }
    }


	void _M_weak_add_ref() noexcept {
      __gnu_cxx::__atomic_add_dispatch(&_M_weak_count, 1);
    }


   void _M_weak_release() noexcept {
      // Be race-detector-friendly. For more info see bits/c++config.
      _GLIBCXX_SYNCHRONIZATION_HAPPENS_BEFORE(&_M_weak_count);
      if (__gnu_cxx::__exchange_and_add_dispatch(&_M_weak_count, -1) == 1) {
        _GLIBCXX_SYNCHRONIZATION_HAPPENS_AFTER(&_M_weak_count);
        if (_Mutex_base<_Lp>::_S_need_barriers) {
          // See _M_release(),
          // destroy() must observe results of dispose()
          _GLIBCXX_READ_MEM_BARRIER;
          _GLIBCXX_WRITE_MEM_BARRIER;
        }
        _M_destroy();
      }
    }

```

## class _Sp_counted_ptr

```cpp
template <typename _Ptr, _Lock_policy _Lp>
class _Sp_counted_ptr final : public _Sp_counted_base<_Lp>
```

### 成员变量

```cpp
protected:
  _Ptr _M_ptr; // 托管内存的指针
```

### 成员函数

```cpp
explicit _Sp_counted_ptr(_Ptr __p) : _M_ptr(__p) {}


 virtual void _M_dispose() noexcept { delete _M_ptr; }

 virtual void _M_destroy() noexcept { delete this; }
```



#  weak_ptr

参考自https://www.jianshu.com/p/b6ac02d406a0

> 注意当shared_ptr拷贝或者移动时_M_weak_count是不会增加的，它表示的是管理对象的计数，只有当__M_use_count为0时_M_weak_count才会减1，除此之外_M_weak_count的数值是由weak_ptr控制的。

> weak_ptr内部其实和shared_ptr内部持有的是同一个管理对象指针，即_Sp_counted_base的指针，当weak_ptr赋值、拷贝、析构时候，_Sp_counted_base内部的_M_weak_count会相应加减。当_M_weak_count为0时，表示不再需要管理对象来控制托管对象，调用_M_destroy()的delete this来释放管理对象内存。

> weak_ptr不会延长原生指针所指向的对象的生命周期，实际上weak_ptr也没有原生指针的构造函数，只能由shared_ptr或weak_ptr进行构造。是一种不已延长生命周期为目的的只能指针。



## class weak_ptr

```cpp
template <typename _Tp> class weak_ptr : public __weak_ptr<_Tp>
```

### 成员函数

```cpp
template <typename _Tp1>
weak_ptr &operator=(const weak_ptr<_Tp1> &__r) noexcept {
  this->__weak_ptr<_Tp>::operator=(__r);
  return *this;
}

 template <typename _Tp1,
              typename = typename std::enable_if<
                  std::is_convertible<_Tp1 *, _Tp *>::value>::type>
    weak_ptr(const weak_ptr<_Tp1> &__r) noexcept : __weak_ptr<_Tp>(__r) {}


// 用来延长生命周期
    shared_ptr<_Tp> lock() const noexcept {
#ifdef __GTHREADS
      if (this->expired())
        return shared_ptr<_Tp>();

      __try {
        return shared_ptr<_Tp>(*this);
      }
      __catch(const bad_weak_ptr &) { return shared_ptr<_Tp>(); }
#else
      return this->expired() ? shared_ptr<_Tp>() : shared_ptr<_Tp>(*this);
#endif
    } 

```

## class __weak_ptr

```cpp
template <typename _Tp, _Lock_policy _Lp> class __weak_ptr
```

### 成员变量

```cpp
_Tp *_M_ptr;                   // Contained pointer.
__weak_count<_Lp> _M_refcount; // Reference counter.
```

### 成员函数

```cpp
  template <typename _Tp1,
              typename = typename std::enable_if<
                  std::is_convertible<_Tp1 *, _Tp *>::value>::type>
    __weak_ptr(const __shared_ptr<_Tp1, _Lp> &__r) noexcept
        : _M_ptr(__r._M_ptr), _M_refcount(__r._M_refcount) {}


template <typename _Tp1>
__weak_ptr &operator=(const __weak_ptr<_Tp1, _Lp> &__r) noexcept {
  _M_ptr = __r.lock().get();
  _M_refcount = __r._M_refcount;
  return *this;
}

__weak_ptr(const __weak_ptr &) noexcept = default;

 ~__weak_ptr() = default;

bool expired() const noexcept {
  return _M_refcount._M_get_use_count() == 0;
}
```





##  class __weak_count

```cpp
template <_Lock_policy _Lp> class __weak_count
```

###  成员变量

```cpp
_Sp_counted_base<_Lp> *_M_pi;
```

### 成员函数

```cpp
__weak_count(const __weak_count<_Lp> &__r) noexcept : _M_pi(__r._M_pi) {
  if (_M_pi != 0)
    _M_pi->_M_weak_add_ref();
}

__weak_count<_Lp> &operator=(const __weak_count<_Lp> &__r) noexcept {
      _Sp_counted_base<_Lp> *__tmp = __r._M_pi;
      if (__tmp != 0)
        __tmp->_M_weak_add_ref();
      if (_M_pi != 0)
        _M_pi->_M_weak_release();
      _M_pi = __tmp;
      return *this;
 }

 ~__weak_count() noexcept {
      if (_M_pi != 0)
        _M_pi->_M_weak_release();
 }
```