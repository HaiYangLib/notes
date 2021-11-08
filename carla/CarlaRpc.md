[CarlaRpc](https://github.com/carla-simulator/rpclib)



**server**

```c++
/*********************************************************************************
 *File    :server
 *Author  :hanhy
 *Email   :hanhy20@mails.jlu.edu.cn
 *Desc    :
 **********************************************************************************/
#include <iostream>
#include <rpc/server.h>

void Foo() { std::cout << "foo was called!" << std::endl; }

int main(int argc, char *argv[]) {
  // Creating a server that listens on port 8080
  rpc::server srv(8080);

  // Binding the name "foo" to free function foo.
  // note: the signature is automatically captured
  srv.bind("Foo", &Foo);

  // Binding a lambda function to the name "add".
  srv.bind("Add", [](int a, int b) { return a + b; });

  // Run the server loop.
  std::cout << "begin run ..." << std::endl;
  srv.run();

  return 0;
}
```



**client**

```c++
/*********************************************************************************
 *File    :client
 *Author  :hanhy
 *Email   :hanhy20@mails.jlu.edu.cn
 *Desc    :
 **********************************************************************************/
#include <rpc/client.h>
#include <iostream>

int main() {
  // Creating a client that connects to the localhost on port 8080
  rpc::client client("127.0.0.1", 8080);

  // Calling a function with paramters and converting the result to int
  auto result1 = client.call("Add", 2, 3).as<int>();
  std::cout << "The result1 is: " << result1 << std::endl;
  client.call("Foo");
    return 0;
}

```

**CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 2.8.12)
project(rpc_test)
set(CMAKE_CXX_FLAGS -pthread)
find_package(rpclib REQUIRED)
if (${rpclib_FOUND})
    message("rpc found")
else ()
    message("rpc not found")
endif ()
include_directories(${rpclib_INCLUDE_DIRS})
add_executable(server server.cpp)
target_link_libraries(server rpc)

add_executable(client client.cpp)
target_link_libraries(client rpc)
```



