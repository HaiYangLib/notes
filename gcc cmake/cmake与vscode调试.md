在工作空间的CMakeList下添加一下命令
```c
add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_FLAAGS_DEBUG "$ENV{CXXFLAGS} -00 -Wall -g -ggdb")
set(CMAKE_BUILD_FLAAGS_RELEASE "$ENV{CXXFLAGS} -03 -Wall")
```

