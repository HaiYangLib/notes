# JAVA进阶

## Object类

`java.lang.Object`类是Java语言中的根类，即所有类的父类。它中描述的所有方法子类都可以使用。在对象实例化的时候，最终找的父类就是Object。

* `public String toString()`：返回该对象的字符串表示。
* `public boolean equals(Object obj)`：指示其他某个对象是否与此对象“相等”。

### toString方法

* `public String toString()`：返回该对象的字符串表示。

toString方法返回该对象的字符串表示，其实该字符串内容就是对象的类型+@+内存地址值。

由于toString方法返回的结果是内存地址，而在开发中，经常需要按照对象的属性得到相应的字符串表现形式，因此也需要重写它。

```java
public class Person {  
    private String name;
    private int age;

    @Override
    public String toString() {
        return "Person{" + "name='" + name + '\'' + ", age=" + age + '}';
    }

    // 省略构造器与Getter Setter
}
```

### equals方法

* `public boolean equals(Object obj)`：指示其他某个对象是否与此对象“相等”。

调用成员方法equals并指定参数为另一个对象，则可以判断这两个对象是否是相同的。这里的“相同”有默认和自定义两种方式。

如果没有覆盖重写equals方法，那么Object类中默认进行`==`运算符的对象地址比较，只要不是同一个对象，结果必然为false。

```java
import java.util.Objects;

public class Person {	
	private String name;
	private int age;
	
    @Override
    public boolean equals(Object o) {
        // 如果对象地址一样，则认为相同
        if (this == o)
            return true;
        // 如果参数为空，或者类型信息不一样，则认为不同
        if (o == null || getClass() != o.getClass())
            return false;
        // 转换为当前类型
        Person person = (Person) o;
        // 要求基本类型相等，并且将引用类型交给java.util.Objects类的equals静态方法取用结果
        return age == person.age && Objects.equals(name, person.name);
    }
}
```



## 日期时间类

### Date类

```java
package com.itheima.demo02.Date;
/*
    java.util.Date:表示日期和时间的类
    类 Date 表示特定的瞬间，精确到毫秒。
    毫秒:千分之一秒 1000毫秒=1秒
    特定的瞬间:一个时间点,一刹那时间
    2088-08-08 09:55:33:333 瞬间
    2088-08-08 09:55:33:334 瞬间
    2088-08-08 09:55:33:334 瞬间
    ...
    毫秒值的作用:可以对时间和日期进行计算
    2099-01-03 到 2088-01-01 中间一共有多少天
    可以日期转换为毫秒进行计算,计算完毕,在把毫秒转换为日期

    把日期转换为毫秒:
        当前的日期:2088-01-01
        时间原点(0毫秒):1970 年 1 月 1 日 00:00:00(英国格林威治)
        就是计算当前日期到时间原点之间一共经历了多少毫秒 (3742767540068L)
    注意:
        中国属于东八区,会把时间增加8个小时
        1970 年 1 月 1 日 08:00:00

    把毫秒转换为日期:
        1 天 = 24 × 60 × 60 = 86400 秒  = 86400 x 1000 = 86400000毫秒
 */
public class Demo01Date {
    public static void main(String[] args) {
        System.out.println(System.currentTimeMillis());//获取当前系统时间到1970 年 1 月 1 日 00:00:00经历了多少毫秒
    }
}
```

```java
import java.util.Date;

public class Demo02Date {
    public static void main(String[] args) {
        demo03();
    }

    /*
        long getTime() 把日期转换为毫秒值(相当于System.currentTimeMillis()方法)
          返回自 1970 年 1 月 1 日 00:00:00 GMT 以来此 Date 对象表示的毫秒数。
     */
    private static void demo03() {
        Date date = new Date();
        long time = date.getTime();
        System.out.println(time);//3742777636267
    }

    /*
        Date类的带参数构造方法
        Date(long date) :传递毫秒值,把毫秒值转换为Date日期
     */
    private static void demo02() {
        Date date = new Date(0L);
        System.out.println(date);// Thu Jan 01 08:00:00 CST 1970

        date = new Date(3742767540068L);
        System.out.println(date);// Sun Aug 08 09:39:00 CST 2088
    }

    /*
        Date类的空参数构造方法
        Date() 获取当前系统的日期和时间
     */
    private static void demo01() {
        Date date = new Date();
        System.out.println(date);//Sun Aug 08 12:23:03 CST 2088
    }
}
```

```java

import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.Date;

/*
    java.text.DateFormat:是日期/时间格式化子类的抽象类
    作用:
        格式化（也就是日期 -> 文本）、解析（文本-> 日期）
    成员方法:
        String format(Date date)  按照指定的模式,把Date日期,格式化为符合模式的字符串
        Date parse(String source)  把符合模式的字符串,解析为Date日期
    DateFormat类是一个抽象类,无法直接创建对象使用,可以使用DateFormat类的子类

    java.text.SimpleDateFormat extends DateFormat

    构造方法:
        SimpleDateFormat(String pattern)
          用给定的模式和默认语言环境的日期格式符号构造 SimpleDateFormat。
        参数:
             String pattern:传递指定的模式
        模式:区分大小写的
            y   年
            M   月
            d   日
            H   时
            m   分
            s   秒
        写对应的模式,会把模式替换为对应的日期和时间
            "yyyy-MM-dd HH:mm:ss"
        注意:
            模式中的字母不能更改,连接模式的符号可以改变
             "yyyy年MM月dd日 HH时mm分ss秒"

 */
public class Demo01DateFormat {
    public static void main(String[] args) throws ParseException {
        demo02();
    }

    /*
         使用DateFormat类中的方法parse,把文本解析为日期
         使用步骤:
            1.创建SimpleDateFormat对象,构造方法中传递指定的模式
            2.调用SimpleDateFormat对象中的方法parse,把符合构造方法中模式的字符串,解析为Date日期
            注意:
                public Date parse(String source) throws ParseException
                parse方法声明了一个异常叫ParseException
                如果字符串和构造方法的模式不一样,那么程序就会抛出此异常
                调用一个抛出了异常的方法,就必须的处理这个异常,要么throws继续抛出这个异常,要么try catch自己处理
     */
    private static void demo02() throws ParseException {
        //1.创建SimpleDateFormat对象,构造方法中传递指定的模式
        SimpleDateFormat sdf = new SimpleDateFormat("yyyy年MM月dd日 HH时mm分ss秒");
        //2.调用SimpleDateFormat对象中的方法parse,把符合构造方法中模式的字符串,解析为Date日期
        //Date parse(String source)  把符合模式的字符串,解析为Date日期
        Date date = sdf.parse("2088年08月08日 15时51分54秒");
        System.out.println(date);
    }

    /*
        使用DateFormat类中的方法format,把日期格式化为文本
        使用步骤:
            1.创建SimpleDateFormat对象,构造方法中传递指定的模式
            2.调用SimpleDateFormat对象中的方法format,按照构造方法中指定的模式,把Date日期格式化为符合模式的字符串(文本)
     */
    private static void demo01() {
        //1.创建SimpleDateFormat对象,构造方法中传递指定的模式
        SimpleDateFormat sdf = new SimpleDateFormat("yyyy年MM月dd日 HH时mm分ss秒");
        //2.调用SimpleDateFormat对象中的方法format,按照构造方法中指定的模式,把Date日期格式化为符合模式的字符串(文本)
        //String format(Date date)  按照指定的模式,把Date日期,格式化为符合模式的字符串
        Date date = new Date();
        String d = sdf.format(date);
        System.out.println(date);//Sun Aug 08 15:51:54 CST 2088
        System.out.println(d);//2088年08月08日 15时51分54秒
    }
}

```



### Calendar类

Calendar为抽象类，由于语言敏感性，Calendar类在创建对象时并非直接创建，而是通过静态方法创建，返回子类对象，如下：

Calendar静态方法

* `public static Calendar getInstance()`：使用默认时区和语言环境获得一个日历

```java
import java.util.Calendar;

public class Demo06CalendarInit {
    public static void main(String[] args) {
        Calendar cal = Calendar.getInstance();
    }    
}
```

```java
/*
    Calendar类的常用成员方法:
        public int get(int field)：返回给定日历字段的值。
        public void set(int field, int value)：将给定的日历字段设置为给定值。
        public abstract void add(int field, int amount)：根据日历的规则，为给定的日历字段添加或减去指定的时间量。
        public Date getTime()：返回一个表示此Calendar时间值（从历元到现在的毫秒偏移量）的Date对象。
    成员方法的参数:
        int field:日历类的字段,可以使用Calendar类的静态成员变量获取
            public static final int YEAR = 1;	年
            public static final int MONTH = 2;	月
            public static final int DATE = 5;	月中的某一天
            public static final int DAY_OF_MONTH = 5;月中的某一天
            public static final int HOUR = 10; 		时
            public static final int MINUTE = 12; 	分
            public static final int SECOND = 13;	秒
 */
public class Demo02Calendar {
    public static void main(String[] args) {
        demo04();
    }

    /*
        public Date getTime()：返回一个表示此Calendar时间值（从历元到现在的毫秒偏移量）的Date对象。
        把日历对象,转换为日期对象
     */
    private static void demo04() {
        //使用getInstance方法获取Calendar对象
        Calendar c = Calendar.getInstance();

        Date date = c.getTime();
        System.out.println(date);
    }

    /*
        public abstract void add(int field, int amount)：根据日历的规则，为给定的日历字段添加或减去指定的时间量。
        把指定的字段增加/减少指定的值
        参数:
            int field:传递指定的日历字段(YEAR,MONTH...)
            int amount:增加/减少指定的值
                正数:增加
                负数:减少
     */
    
      /*
        public void set(int field, int value)：将给定的日历字段设置为给定值。
        参数:
            int field:传递指定的日历字段(YEAR,MONTH...)
            int value:给指定字段设置的值
     */
    
    
      /*
        public int get(int field)：返回给定日历字段的值。
        参数:传递指定的日历字段(YEAR,MONTH...)
        返回值:日历字段代表的具体的值
     */
    private static void demo03() {
        Calendar c = Calendar.getInstance();
        Date date = c.getTime();
        System.out.println(date); //Tue Apr 20 17:32:17 CST 2021

        c.add(Calendar.YEAR, 2);
        System.out.println(c.getTime()); //Thu Apr 20 17:32:17 CST 2023

        c.add(Calendar.MONTH, -3);
        System.out.println(c.getTime()); //Fri Jan 20 17:33:20 CST 2023

        c.set(Calendar.YEAR, 9999);
        System.out.println(c.getTime());//Wed Jan 20 17:35:54 CST 9999

        //同时设置年月日,可以使用set的重载方法
        c.set(8888, 8, 8);
        System.out.println(c.getTime());  //Wed Sep 08 17:36:45 CST 8888


        int year = c.get(Calendar.YEAR);
        System.out.println(year);  //8888

        int d = c.get(Calendar.DATE);
        System.out.println(d);  //8

        System.out.println(Calendar.getInstance().getTime()); //Tue Apr 20 17:33:44 CST 2021
    }
}
```

## System类

```java
/*
    java.lang.System类中提供了大量的静态方法，可以获取与系统相关的信息或系统级操作，在System类的API文档中，常用的方法有：
        public static long currentTimeMillis()：返回以毫秒为单位的当前时间。
        public static void arraycopy(Object src, int srcPos, Object dest, int destPos, int length)：将数组中指定的数据拷贝到另一个数组中。
 */
public class Demo01System {
    public static void main(String[] args) {
        demo02();
        StringBuilder sb = new StringBuilder();
    }

    /*
        public static void arraycopy(Object src, int srcPos, Object dest, int destPos, int length)：将数组中指定的数据拷贝到另一个数组中。
        参数:
            src - 源数组。
            srcPos - 源数组中的起始位置(起始索引)。
            dest - 目标数组。
            destPos - 目标数据中的起始位置。
            length - 要复制的数组元素的数量。
        练习:
            将src数组中前3个元素，复制到dest数组的前3个位置上
                复制元素前：
                src数组元素[1,2,3,4,5]，dest数组元素[6,7,8,9,10]
                复制元素后：
                src数组元素[1,2,3,4,5]，dest数组元素[1,2,3,9,10]
     */
    private static void demo02() {
        //定义源数组
        int[] src = {1,2,3,4,5};
        //定义目标数组
        int[] dest = {6,7,8,9,10};
        System.out.println("复制前:"+ Arrays.toString(dest));
        //使用System类中的arraycopy把源数组的前3个元素复制到目标数组的前3个位置上
        System.arraycopy(src,0,dest,0,3);
        System.out.println("复制后:"+ Arrays.toString(dest));
    }

    /*
        public static long currentTimeMillis()：返回以毫秒为单位的当前时间。
        用来程序的效率
        验证for循环打印数字1-9999所需要使用的时间（毫秒）
     */
    private static void demo01() {
        //程序执行前,获取一次毫秒值
        long s = System.currentTimeMillis();
        //执行for循环
        for (int i = 1; i <=9999 ; i++) {
            System.out.println(i);
        }
        //程序执行后,获取一次毫秒值
        long e = System.currentTimeMillis();
        System.out.println("程序共耗时:"+(e-s)+"毫秒");//程序共耗时:106毫秒
    }
}
```

## StringBuilder类

```java
/*
    java.lang.StringBuilder类:字符串缓冲区,可以提高字符串的效率
    构造方法:
        StringBuilder() 构造一个不带任何字符的字符串生成器，其初始容量为 16 个字符。
        StringBuilder(String str) 构造一个字符串生成器，并初始化为指定的字符串内容。
 */
public class Demo01StringBuilder {
    public static void main(String[] args) {
        //空参数构造方法
        StringBuilder bu1 = new StringBuilder();
        System.out.println("bu1:"+bu1);//bu1:""

        //带字符串的构造方法
        StringBuilder bu2 = new StringBuilder("abc");
        System.out.println("bu2:"+bu2);//bu2:abc
    }
}
```

```java
public class TestStu {
    public static void main(String args[]) throws ParseException {
        //创建StringBuilder对象
        StringBuilder bu = new StringBuilder();
//        使用append方法往StringBuilder中添加数据
//        append方法返回的是this,调用方法的对象bu,this==bu
        StringBuilder bu2 = bu.append("abc");//把bu的地址赋值给了bu2
        System.out.println(bu);//"abc"
        System.out.println(bu2);//"abc"
        System.out.println(bu == bu2);//比较的是地址 true
    }
}



/*
    StringBuilder和String可以相互转换:
        String->StringBuilder:可以使用StringBuilder的构造方法
            StringBuilder(String str) 构造一个字符串生成器，并初始化为指定的字符串内容。
        StringBuilder->String:可以使用StringBuilder中的toString方法
            public String toString()：将当前StringBuilder对象转换为String对象。
 */
public class Demo03StringBuilder {
    public static void main(String[] args) {
        //String->StringBuilder
        String str = "hello";
        System.out.println("str:"+str);
        StringBuilder bu = new StringBuilder(str);
        //往StringBuilder中添加数据
        bu.append("world");
        System.out.println("bu:"+bu);

        //StringBuilder->String
        String s = bu.toString();
        System.out.println("s:"+s);
    }
}
```