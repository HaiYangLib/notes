# gtest使用
```cpp
#include "gtest/gtest.h"

int add(int a,int b){
    return a+b;
}

int sub(int a,int b){
    return a-b;
}
TEST(testCase,add){
    EXPECT_EQ(add(2,3),3);
}
TEST(testCase,sub){

    EXPECT_EQ(sub(1,3),-2);
}
int main(int argc,char **argv){
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
```
`CMakeLists.txt`
```c
cmake_minimum_required(VERSION 3.10)
project(cpp)

find_package(Threads)

find_package(GTest)

message("GTEST_LIBRARIES=${GTEST_LIBRARIES}")

message("GTEST_MAIN_LIBRARIES=${GTEST_MAIN_LIBRARIES}")

add_executable(cpp src/gtest.cpp)

target_link_libraries(cpp Threads::Threads ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES})

set(EXECUTABLE_OUTPUT_PATH ./bin)
```
![在这里插入图片描述](assets/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2cODY3,size_16,color_FFFFFF,t_70)