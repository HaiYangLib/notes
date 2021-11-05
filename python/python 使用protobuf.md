# protobuf
## protobuf抽象语法
```python
syntax = "proto2";
package example;

message person {
  required int32 id = 1;
  required string name = 2;
}

message all_person {
  repeated person Per = 1;
}

```
![在这里插入图片描述](https://img-blog.csdnimg.cn/caf4232a23384191be61ef1825d43108.png)

```python
import person_pb2

# 为 all_person 填充数据
pers = person_pb2.all_person()
p1 = pers.Per.add()
p1.id = 1
p1.name = 'hhy'
p2 = pers.Per.add()
p2.id = 2
p2.name = 'hhh'

# 对数据进行序列化
data = pers.SerializeToString()

# 对已经序列化的数据进行反序列化
target = person_pb2.all_person()
target.ParseFromString(data)
print("-"*30)
print(target.Per[1].name)  # 打印第一个 person name 的值进行反序列化验证
print("-"*20)
print(pers)

```
![在这里插入图片描述](https://img-blog.csdnimg.cn/bc27e1b8dbce4ef197e5bd7058249650.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)