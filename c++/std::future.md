```cpp


/*
 * @Description: 使用std::future获取异步任务的返回值
 * @Version: 1.0
 * @Author: hanhy
 */

#include <future>
#include <iostream>
#include <thread>
#include <unistd.h>

using namespace std;

int find_the_answer(int i) {
  cout << "i=" << i << endl;
  return i;
}

void do_other() {
  std::cout << "do_other_stuff" << std::endl;

}

int main() {

  //有具体实现来选择何处运行
  std::future<int> the_answer1 = std::async(find_the_answer, 1);

  //在新线程中运行
  std::future<int> the_answer2 =
      std::async(std::launch::async, find_the_answer, 2);

  //在wait()或get()中运行
  std::future<int> the_answer3 =
      std::async(std::launch::deferred, find_the_answer, 3);

  //相当于the_answer1 默认选项
  std::future<int> the_answer4 = std::async(
      std::launch::deferred | std::launch ::async, find_the_answer, 4);

  do_other();

  cout << "当前主线程id=" << std::this_thread::get_id() << endl;
  std::cout << "The answer1 is " << the_answer1.get() << std::endl;
  std::cout << "The answer2 is " << the_answer2.get() << std::endl;
  std::cout << "The answer3 is " << the_answer3.get() << std::endl;
  std::cout << "The answer4 is " << the_answer4.get() << std::endl;
}


```