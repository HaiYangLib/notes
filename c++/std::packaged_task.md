```cpp

/*
 * @Description: 使用std::packaged_task在GUI线程上运行代码
 * @Version: 1.0
 * @Author: hanhy
 */

#include <deque>
#include <future>
#include <mutex>
#include <thread>
#include <utility> //这里有move函数

std::mutex m;
std::deque<std::packaged_task<void()>> tasks;

bool gui_shutdown_message_received();
void get_and_process_gui_message();

void gui_thread() {// 1
  while (!gui_shutdown_message_received()) { // 2
    get_and_process_gui_message();  // 3
    std::packaged_task<void()> task;
    {
      std::lock_guard<std::mutex> lk(m);
      if (tasks.empty())           //4
        continue;
      // move将对象的状态或对象转移到另一个对象，原来那个对象就为空了。
      task = std::move(tasks.front());  //5
      tasks.pop_front();
    }
    task();            // 6
  }
}

std::thread gui_bg_thread(gui_thread);

template <typename Func> std::future<void> post_task_for_gui_thread(Func f) {
  std::packaged_task<void()> task(f);  // 7
  std::future<void> res = task.get_future();  // 8
  std::lock_guard<std::mutex> lk(m);
  tasks.push_back(std::move(task));  // 9
  return res;
}
```

> std::packaged_task<> 类模板参数为函数签名，像int(std::string&,double*)表示接受对std::string的非const引用和指向double的指针，并返回int的函数。
> 返回值做为异步结果，存储在由get_future()获取的std::future中。因此可以将任务封装在std::packaged_task中，并在别的地方获取结果。