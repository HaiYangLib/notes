创建本地数据库及表
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210430122824408.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70)

启动emqx服务器
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210430122330148.png)
浏览器输入
http://127.0.0.1:18083
登录到emqx
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210430122421272.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70)
在规则引擎创建资源
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210430122528804.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70)
在资源类型中选MySQL，填写数据库名称,用户名，密码。
![在这里插入图片描述](https://img-blog.csdnimg.cn/2021043012311434.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70)
在规则中创建规则
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210430123635320.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70)
在下面的添加相应动作,选择数据持久化，保存数据到MySQL，使用资源选直线创建的资源。添加一下模板

```sql
INSERT INTO 
	`stu` (`id`, `name`)
VALUES 
	(${id}, ${name});
```

![在这里插入图片描述](https://img-blog.csdnimg.cn/20210430124628128.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70)

![在这里插入图片描述](https://img-blog.csdnimg.cn/20210430124920414.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70)
使用测试软件mqttbox进行测试 ，mqttbox软件及使用方法参见[【mqttbox】](https://blog.csdn.net/qq_38966867/article/details/115978444)
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210430125403343.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70)
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210430125747135.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70)