# cmake install

![在这里插入图片描述](https://img-blog.csdnimg.cn/54ece2db0c0c4a7dbb222d39832235a9.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)

```clike
# CMake 最低版本号要求
cmake_minimum_required (VERSION 2.8)

# 项目信息
project (Demo6)

set (CMAKE_INCLUDE_CURRENT_DIR ON)

# 检查系统是否支持 pow 函数
include (${CMAKE_ROOT}/Modules/CheckFunctionExists.cmake)
check_function_exists (pow HAVE_POW)

message("CMAKE_ROOT:${CMAKE_ROOT}")

# 加入一个配置头文件，用于处理 CMake 对源码的设置
configure_file (
  "${PROJECT_SOURCE_DIR}/config.h.in"
  "${PROJECT_BINARY_DIR}/config.h"
  )


include_directories ("${PROJECT_SOURCE_DIR}/math")
add_subdirectory (math)
set (EXTRA_LIBS ${EXTRA_LIBS} MathFunctions)

# 查找当前目录下的所有源文件
# 并将名称保存到 DIR_SRCS 变量
aux_source_directory(. DIR_SRCS)

# 指定生成目标
add_executable(Demo ${DIR_SRCS})
target_link_libraries (Demo  ${EXTRA_LIBS})

# 指定安装路径
install (TARGETS Demo DESTINATION bin)
install (FILES "${PROJECT_BINARY_DIR}/config.h"
         DESTINATION include)

```
> 方法1-在执行cmake时指定：
> cmake -DCMAKE_INSTALL_PREFIX=<你想要安装的路径>
> 方法二-设置CMAKE_INSTALL_PREFIX 变量：
> set(CMAKE_INSTALL_PREFIX <install_path>)
> ![在这里插入图片描述](https://img-blog.csdnimg.cn/8539b6129dc04362b1c589522d3f0150.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)

![在这里插入图片描述](https://img-blog.csdnimg.cn/6ccc16af77df41ad8345d9927e309fab.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)
![在这里插入图片描述](https://img-blog.csdnimg.cn/ebe15118a31f4171bd1106358fcf542f.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)