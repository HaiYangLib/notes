# 安装
```shell
 sudo apt-get install mysql-server
```

# 登录并设置密码
## 登录
刚安装好的mysql可以免密登录 
```shell
sudo mysql -u root -p
```
## 设置密码
①  输入use  mysql回车；

②  输入update  user  set  password  =  password(“123456”)where  user=’root’;

其中”123456“为密码，‘root’为用户名
对于MySQL5.7没有 password，所以会报错。 password改为authentication_string字段
执行
```SQL
update user set authentication_string =password("123456") where user="root";
flush privileges; 
```
或者
```SQL
update user set authentication_string=password("ln122920"),plugin='mysql_native_password' where user='root';
```
退出MySQL并修改配置文件
对于其他版本的MySQL配置文件为/etc/my.cnf.但MySQL5.7的配置文件为
 sudo vim /etc/mysql/mysql.conf.d/mysqld.cnf
 将skip-grant-tables行注释掉。
 重启MySQL 、
 ```shell
sudo service mysql restart
 ```

## 配置远程登录
你想myuser使用mypassword（密码）从任何主机连接到mysql服务器的话。
```shell
mysql>GRANT ALL PRIVILEGES ON *.* TO 'myuser'@'%'IDENTIFIED BY 'mypassword' WITH GRANT OPTION;
```
如果你想允许用户myuser从ip为192.168.1.6的主机连接到mysql服务器，并使用mypassword作为密码
```shell
mysql>GRANT ALL PRIVILEGES ON *.* TO 'myuser'@'192.168.1.3'IDENTIFIED BY 'mypassword' WITH GRANT OPTION;
```
使修改生效，就可以了
```shell
mysql>FLUSH PRIVILEGES
```

如果仍不能远程登录MySQL，需要修改配置文件
/etc/mysql/mysql.conf.d/mysqld.cnf
取消skip-grant-tables的行注释，并对bind-address           = 127.0.0.1行进行注释。
![在这里插入图片描述](https://img-blog.csdnimg.cn/2021042921045777.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70)

## 取消表名的大小写区分
lower_case_table_names=1