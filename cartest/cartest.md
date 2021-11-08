# RecvWarn

## 	mil RecvWarn 

插入接收警告信息，分别插入**test_millog**，**test_message**

```c
  
// 主车 test_millog
	sprintf(insert_rv,
                "INSERT INTO "
                "test_millog(timestamp,lat,lon,heading,speed,acc,carid,"
                "carlight,gear,angle,hrdistance,zdistance,hdistance,hrangle,"
                "ttc,warnif,warntype) VALUES "
                "(%lld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d);",
                recvWarning.timeStamp, recvWarning.rvData.lat,
                recvWarning.rvData.lon, recvWarning.rvData.heading,
                recvWarning.rvData.speed, recvWarning.rvData.accSet.accLon,
                recvWarning.rvData.id, recvWarning.rvData.light,
                recvWarning.rvData.transmission, recvWarning.rvData.steerAngle,
                recvWarning.distance, recvWarning.distance_y,
                recvWarning.distance_x, recvWarning.angleWithVeh,
                recvWarning.WarningDetail, recvWarning.WarningFlag,
                recvWarning.Warningtype);
        res = mysql_query(&my_connecyion, insert_rv);


// 远车 test_millog
 sprintf(insert_rv,
                "INSERT INTO "
                "test_millog(timestamp,lat,lon,heading,speed,acc,carid,"
                "carlight,gear,angle,hrdistance,zdistance,hdistance,hrangle,"
                "ttc,warnif,warntype) VALUES "
                "(%lld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d);",
                recvWarning.timeStamp, recvWarning.rvData.lat,
                recvWarning.rvData.lon, recvWarning.rvData.heading,
                recvWarning.rvData.speed, recvWarning.rvData.accSet.accLon,
                recvWarning.rvData.id, recvWarning.rvData.light,
                recvWarning.rvData.transmission, recvWarning.rvData.steerAngle,
                recvWarning.distance, recvWarning.distance_y,
                recvWarning.distance_x, recvWarning.angleWithVeh,
                recvWarning.WarningDetail, recvWarning.WarningFlag,
                recvWarning.Warningtype);
        res = mysql_query(&my_connecyion, insert_rv);


// test_message
	sprintf(insert_message,
                  "INSERT INTO "
                  "test_message(infonum,timestamp,messagetype,messagelength,x,"
                  "y,ttc,hvs,rvs,hrdistance,hvttc,rvttc,refttc,refdistance) "
                  "VALUES (%d,%lld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d);",
                  recvCount, recvWarning.timeStamp, recvWarning.Warningtype,
                  ret, recvWarning.distance_x, recvWarning.distance_y,
                  recvWarning.WarningDetail, recvWarning.hvSpeed,
                  recvWarning.rvSpeed, recvWarning.distance,
                  recvWarning.arriveTimeHv, recvWarning.arriveTimeRv,
                  recvWarning.absTTC, recvWarning.absDis);  //上机检所用
  
          res = mysql_query(&my_connecyion, insert_message);
```



## hil RecvWarn 

```c++
// test_message         
	sprintf(insert,
                  "INSERT INTO "
                  "test_message(infonum,timestamp,messagetype,messagelength,x,"
                  "y,ttc,hvs,rvs,hrdistance,hvttc,rvttc,refttc,refdistance) "
                  "VALUES (%d,%lld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d);",
                  recvCount, recvWarning.timeStamp, recvWarning.Warningtype,
                  ret, recvWarning.distance_x, recvWarning.distance_y,
                  recvWarning.WarningDetail, recvWarning.hvSpeed,
                  recvWarning.rvSpeed, recvWarning.distance,
                  recvWarning.arriveTimeHv, recvWarning.arriveTimeRv,
                  recvWarning.absTTC, recvWarning.absDis);  //上机检所用
       
          // //华人所用
          res = mysql_query(&my_connecyion, insert);
```



# 注入

## mil

```c++
// test_vtdlog
sprintf(insert,
            "INSERT INTO test_vtdlog "
            "(timestamp,lon,lat,heading,speed,acc,carid,carlight,gear,angle) "
            "VALUES (%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld);",
            getTimestamp(), car.Long, car.lat, car.heading, car.speed,
            car.acceleration, car.id, car.carlight, car.transmission,
            car.angle);
    mysql_query(&my_connecyion, insert);
```

## hil



