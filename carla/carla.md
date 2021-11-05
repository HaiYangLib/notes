# RayCastSemanticLidar

## 数据产生

>  RayCastSemanticLidar的数据由插件在UE4中产生

* **设置激光雷达各线垂直角**   

  >**void ARayCastSemanticLidar::CreateLasers()**
  >
  >carla会根据Description.UpperFovLimit 和Description.LowerFovLimit以及Description.Channels线数，在垂直方向上均分各个射线
  >
  >```c++
  >const float VerticalAngle =
  >        Description.UpperFovLimit - static_cast<float>(i) * DeltaAngle;
  >    LaserAngles.Emplace(VerticalAngle);
  >```

* **计算两次Tick时间内，激光雷达各个射线的碰撞点**

  > **void ARayCastSemanticLidar::SimulateLidar(const float DeltaTime) **
  >
  > 首先计算这个时间段内，每个射线扫描点的数量 
  >
  > ```c++
  > const uint32 PointsToScanWithOneLaser = FMath::RoundHalfFromZero(
  >     Description.PointsPerSecond * DeltaTime / float(ChannelCount));
  > ```
  >
  > 然后计算水平角间隔
  >
  > ```c++
  > const float AngleDistanceOfTick =
  >     Description.RotationFrequency * Description.HorizontalFov * DeltaTime;
  > const float AngleDistanceOfLaserMeasure =
  >     AngleDistanceOfTick / PointsToScanWithOneLaser;
  > 
  > ```
  >
  > 通过计算得到的水平角和垂直角 发射射线，只有ShootLaser函数返回为真的时候才执行 WritePointAsync函数
  >
  > ```c++
  > const float VertAngle = LaserAngles[idxChannel];
  > const float HorizAngle =
  >     std::fmod(CurrentHorizontalAngle +
  >                   AngleDistanceOfLaserMeasure * idxPtsOneLaser,
  >               Description.HorizontalFov) -
  >     Description.HorizontalFov / 2;
  > const bool PreprocessResult =
  >     RayPreprocessCondition[idxChannel][idxPtsOneLaser];
  > 
  > if (PreprocessResult &&
  >     ShootLaser(VertAngle, HorizAngle, HitResult, TraceParams)) {
  >   WritePointAsync(idxChannel, HitResult);
  > } 
  > ```



## 数据发送

> UE4插件产生的数据需要输出给客户端

* **写入数据**

  > **void ARayCastSemanticLidar::WritePointAsync(uint32_t channel,FHitResult &detection)**
  >
  > 激光雷达数据被写入到RecordedHits中
  >
  > ```c++
  > RecordedHits[channel].emplace_back(detection);
  > ```
  >
  > 在PostPhysTick结束前执行
  >
  > ```c++
  > DataStream.Send(*this, SemanticLidarData, DataStream.PopBufferFromPool());
  > ```
  >
  > ```c++
  > void ARayCastSemanticLidar::PostPhysTick(UWorld *World, ELevelTick TickType,
  >                                          float DeltaTime) {
  >   TRACE_CPUPROFILER_EVENT_SCOPE(ARayCastSemanticLidar::PostPhysTick);
  >   SimulateLidar(DeltaTime);
  > 
  >   {
  >     TRACE_CPUPROFILER_EVENT_SCOPE_STR("Send Stream");
  >     auto DataStream = GetDataStream(*this);
  >     DataStream.Send(*this, SemanticLidarData, DataStream.PopBufferFromPool());
  >   }
  > }
  > ```
  >
  > Sensor成员函数GetDataStream 返回**FAsyncDataStreamTmpl <carla::streaming::Stream >;**
  >
  > 所以ARayCastSemanticLidar::PostPhysTick函数中的 DataStream类型为FAsyncDataStreamTmpl <carla::streaming::Stream >
  >
  > ```c++
  > using FAsyncDataStream = FAsyncDataStreamTmpl<carla::streaming::Stream>;
  > using FDataStream = FDataStreamTmpl<carla::streaming::Stream>;
  > 
  > FDataStream Stream; 
  > 
  > template <typename SensorT>
  >   FAsyncDataStream GetDataStream(const SensorT &Self) {
  >     return Stream.MakeAsyncDataStream(Self, 					                   
  >                                       GetEpisode().GetElapsedGameTime());
  >   }
  > 
  > ```
  >
  > 在FAsyncDataStreamTmpl <carla::streaming::Stream >;中Stream为carla::streaming::Stream
  >
  > 所以DataStream.Send 执行的是**carla::streaming::Stream.Write( std::move(Header),
  >       carla::sensor::SensorRegistry::Serialize(Sensor, std::forward<ArgsT>(Args)...));**
  >
  > ```c++
  > template <typename SensorT, typename... ArgsT>
  > inline void FAsyncDataStreamTmpl<T>::Send(SensorT &Sensor, ArgsT &&... Args)
  > {
  >   Stream.Write(
  >       std::move(Header),
  >       carla::sensor::SensorRegistry::Serialize(Sensor, std::forward<ArgsT>(Args)...));
  > }
  > 
  > ```
  >
  > 在 LibCarla/source/carla/streaming/Stream.h中，carla::streaming::Stream实际上是detail::Stream< detail::MultiStreamState>;
  >
  > 顺藤摸瓜，DataStream.Send执行的最终是detail::Stream<detail::MultiStreamState >.Write
  >
  > 所以只需分析detail::Stream<detail::MultiStreamState >.Write 函数即明白激光雷达数据是如何发送给客户端的。

* **实际发送**

  > **激光雷达数据会通过MultiStreamState中的Write函数发送出去**
  >
  > ```c++
  >   template <typename... Buffers>
  >     void Write(Buffers &&... buffers) {
  >       auto message = Session::MakeMessage(std::move(buffers)...);
  > 
  >       // try write single stream
  >       auto session = _session.load();
  >       if (session != nullptr) {
  >         session->Write(std::move(message));
  >         // Return here, _session is only valid if we have a 
  >         // single session.
  >         return; 
  >       }
  > 
  >       // try write multiple stream
  >       std::lock_guard<std::mutex> lock(_mutex);
  >       for (auto &s : _sessions) {
  >         if (s != nullptr) {
  >           s->Write(message);
  >         }
  >       }
  >     }
  > 
  > ```
  >
  > 无论是single stream还是multiple stream，detail::Stream<detail::MultiStreamState >.Write 函数都会使用AtomicSharedPtr<Session> _session;来发送数据
  >
  > 在LibCarla/source/carla/streaming/detail/Session.h可以发现，Session实际上是tcp::ServerSession;
  >
  > ```c++
  > using Session = tcp::ServerSession;
  > ```
  >
  > 下面看一下tcp::ServerSession是个什么东西
  >
  > 在LibCarla/source/carla/streaming/detail/tcp/ServerSession.h中给出了ServerSession的声明
  >
  > 在**void ServerSession::Write(std::shared_ptr<const Message> message) **中
  >
  > ```c++
  > using socket_type = boost::asio::ip::tcp::socket;
  > socket_type _socket;
  > 
  > 
  > auto handle_sent = [this, self, message](const boost::system::error_code &ec, size_t DEBUG_ONLY(bytes)) {
  >         _is_writing = false;
  >         if (ec) {
  >           log_info("session", _session_id, ": error sending data :", ec.message());
  >           CloseNow();
  >         } else {
  >           DEBUG_ONLY(log_debug("session", _session_id, ": successfully sent", bytes, "bytes"));
  >           DEBUG_ASSERT_EQ(bytes, sizeof(message_size_type) + message->size());
  >         }
  >       };
  > 
  > _deadline.expires_from_now(_timeout);
  >       boost::asio::async_write(
  >           _socket,
  >           message->GetBufferSequence(),
  >           handle_sent);
  > 
  > //_deadline实际上boost::asio::deadline_timer
  >  boost::asio::deadline_timer _deadline;
  > ```



  

  

