# unique_ptr 

## class unique_ptr

```cpp
template <typename _Tp, typename _Dp = default_delete<_Tp>> class unique_ptr
```

### 成员变量

```cpp
  class _Pointer {
    template <typename _Up>
    static typename _Up::pointer __test(typename _Up::pointer *);

    template <typename _Up> static _Tp *__test(...);

    typedef typename remove_reference<_Dp>::type _Del;

  public:
    typedef decltype(__test<_Del>(0)) type;
  };

  typedef std::tuple<typename _Pointer::type, _Dp> __tuple_type;
  __tuple_type _M_t;

public:
  typedef typename _Pointer::type pointer;
  typedef _Tp element_type;
  typedef _Dp deleter_type;
```

### 成员函数

```cpp
 	constexpr unique_ptr() noexcept : _M_t() {
      static_assert(!is_pointer<deleter_type>::value,
                    "constructed with null function pointer deleter");
    }

    explicit unique_ptr(pointer __p) noexcept : _M_t(__p, deleter_type()) {
      static_assert(!is_pointer<deleter_type>::value,
                    "constructed with null function pointer deleter");
    }



	pointer get() const noexcept { return std::get<0>(_M_t); }


//unqiue_ptr还定义了两个很重要的函数：reset(pointer)和release()。reset(pointer)的功能是用一个新指针替换原来的指针，而release()则是是放弃原生指针的所有权。

  pointer release() noexcept {
      pointer __p = get();
      std::get<0>(_M_t) = pointer();
      return __p;
    }

    void reset(pointer __p = pointer()) noexcept {
      using std::swap;
      swap(std::get<0>(_M_t), __p);
      if (__p != pointer())
        get_deleter()(__p);
    }


    typename add_lvalue_reference<element_type>::type operator*() const {
      _GLIBCXX_DEBUG_ASSERT(get() != pointer());
      return *get();
    }


 pointer operator->() const noexcept {
      _GLIBCXX_DEBUG_ASSERT(get() != pointer());
      return get();
    }



  ~unique_ptr() noexcept {
      auto &__ptr = std::get<0>(_M_t);
      if (__ptr != nullptr)
        get_deleter()(__ptr);
      __ptr = pointer();
    }

deleter_type &get_deleter() noexcept { return std::get<1>(_M_t); }
```

# default_delete
```cpp
template <typename _Tp> struct default_delete
```

## 成员函数

```cpp
void operator()(_Tp *__ptr) const {
  static_assert(sizeof(_Tp) > 0, "can't delete pointer to incomplete type");
  delete __ptr;
}
```