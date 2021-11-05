# 安装mqtt服务器 emqx
官网下载[emqx官网](https://www.emqx.io/cn/downloads#broker)

对于ubuntu18.04
```bashshell
wget https://www.emqx.cn/downloads/broker/v4.2.11/emqx-ubuntu20.04-4.2.11-x86_64.deb

sudo apt install ./emqx-ubuntu20.04-4.2.11-x86_64.deb

sudo emqx start
```
```shell
sudo emqx status #查看运行状态
sudo emqx start #启动
sudo emqx stop  #停止
sudo emqx console #进入emqx命令行窗口
```

![在这里插入图片描述](https://img-blog.csdnimg.cn/20210421205515602.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70)
# 浏览器登录
进入console查看ip和端口信息
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210421205724669.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70)
通过http:dashboard 的信息可以登录emqx界面
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210421205903901.png)
默认用户名 admin 密码 public
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210421205920262.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70)
![在这里插入图片描述](https://img-blog.csdnimg.cn/2021042121003156.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70)
# 利用工具mqttbox测试emqx能否正常通信
mqttbox下载地址
链接：https://pan.baidu.com/s/1-UyAzC_6Almzod2KogUsdQ 
提取码：nyhf 
复制这段内容后打开百度网盘手机App，操作更方便哦

打开mqttbox 点击Create MQTT Client
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210421210521518.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70)
client name 随便写
protocol选mqtt/tcp
host 填127.0.0.1:1883（可通过 sudo emqx console 查询得到）
username 填admin
密码public
然后点击保存
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210421211150548.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70)
添加话题和订阅者
topic to pubish 随便填写
点击Add subscriber添加订阅者,并填写相同的话题
在Payload中填写需要发送的信息

![在这里插入图片描述](https://img-blog.csdnimg.cn/20210421211534352.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70)

# c语言测试emqx
**my_test_pub.c**
```c
/*
 * @Description: 测试发布者
 * @Version: 1.0
 * @Author: hanhy
 */

#include "mqttclient.h"
#include <stdio.h>
#include <unistd.h>

int main() {
  mqtt_client_t *client = NULL;

  mqtt_log_init();
  client = mqtt_lease();

  mqtt_set_port(client, "1883");
  mqtt_set_host(client, "127.0.0.1");
  mqtt_set_client_id(client, "pub");
  mqtt_set_user_name(client, "admin");
  mqtt_set_password(client, "public");
  mqtt_set_clean_session(client, 1);

  mqtt_connect(client);

  char buf[100] = {0};
  mqtt_message_t msg;
  memset(&msg, 0, sizeof(msg));
  sprintf(buf, "welcome to mqttclient, this is a publish test...");

  sleep(2);

  mqtt_list_subscribe_topic(client);

  msg.payload = (void *)buf;
  msg.qos = 0;
  while (1) {
    sprintf(
        buf,
        "a rand number: %d ...",
        random_number());
    mqtt_publish(client, "test_topic", &msg);
    printf("sent!!!\n");
    sleep(2);
  }

  while (1) {
    sleep(100);
  }
}
```

**my_test_sub.c**
```c
/*
 * @Description: 测试 订阅者
 * @Version: 1.0
 * @Author: hanhy
 */

#include "mqttclient.h"
#include <unistd.h>

static void topic_handler(void *client, message_data_t *msg) {
  (void)client;
  printf("topic: %s message:%s\n", msg->topic_name,
             (char *)msg->message->payload);
}

int main() {
  mqtt_client_t *client = NULL;

  mqtt_log_init();
  client = mqtt_lease();

  mqtt_set_port(client, "1883");
  mqtt_set_host(client, "127.0.0.1");
  mqtt_set_client_id(client, "sub");
  mqtt_set_user_name(client, "admin");
  mqtt_set_password(client, "public");
  mqtt_set_clean_session(client, 1);


  mqtt_connect(client);

  mqtt_subscribe(client, "test_topic", QOS0, topic_handler);

  while (1) {
    sleep(100);
  }
}
```
 [测试用例及开源mqtt客户端源代码](https://github.com/hanhy2416/mqttclient)