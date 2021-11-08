* 在文件中Examples/CppClient/main.cpp 有如下代码：

```c++
 auto world = client.LoadWorld(town_name);
```

LibCarla/source/carla/client/Client.h

```c++
 World LoadWorld(
        std::string map_name,
        bool reset_settings = true,
        rpc::MapLayer map_layers = rpc::MapLayer::All) const {
      return World{_simulator->LoadEpisode(std::move(map_name), reset_settings, map_layers)};
    }
```

LibCarla/source/carla/client/detail/Simulator.cpp

```c++
 EpisodeProxy Simulator::LoadEpisode(std::string map_name, bool reset_settings, rpc::MapLayer map_layers) {
    const auto id = GetCurrentEpisode().GetId();
    _client.LoadEpisode(std::move(map_name), reset_settings, map_layers);
    size_t number_of_attempts = _client.GetTimeout().milliseconds() / 50u;
    for (auto i = 0u; i < number_of_attempts; ++i) {
      using namespace std::literals::chrono_literals;
      if (_client.GetEpisodeSettings().synchronous_mode)
        _client.SendTickCue();
      _episode->WaitForState(50ms);
      auto episode = GetCurrentEpisode();
      if (episode.GetId() != id) {
        return episode;
      }
    }
    throw_exception(std::runtime_error("failed to connect to newly created map"));
  }

```

LibCarla/source/carla/client/detail/Client.cpp

```c++
 void Client::LoadEpisode(std::string map_name, bool reset_settings, rpc::MapLayer map_layer) {
    // Await response, we need to be sure in this one.
    _pimpl->CallAndWait<void>("load_new_episode", std::move(map_name), reset_settings, map_layer);
  }
```



* Carla 加载地图LoadWorld函数，由RPC实现

```c++
void Client::LoadEpisode(std::string map_name, bool reset_settings, rpc::MapLayer map_layer) {
    // Await response, we need to be sure in this one.
    _pimpl->CallAndWait<void>("load_new_episode", std::move(map_name), reset_settings, map_layer);
}
```

最终的代码实现为**return rpc_client.call(function, std::forward<Args>(args) ...);**

```c++
 template <typename ... Args>
    auto RawCall(const std::string &function, Args && ... args) {
      try {
        return rpc_client.call(function, std::forward<Args>(args) ...);
      } catch (const ::rpc::timeout &) {
        throw_exception(TimeoutException(endpoint, GetTimeout()));
      }
    }

template <typename T, typename ... Args>
    auto CallAndWait(const std::string &function, Args && ... args) {
      auto object = RawCall(function, std::forward<Args>(args) ...);
      using R = typename carla::rpc::Response<T>;
      auto response = object.template as<R>();
      if (response.HasError()) {
        throw_exception(std::runtime_error(response.GetError().What()));
      }
      return Get(response);
    }
```



* **class World **中只有一个成员变量**detail::EpisodeProxy _episode;**

Carla客户端向发起一个**"load_new_episode"**的请求，并包含了地图的名字

  _pimpl->CallAndWait<void>("load_new_episode", std::move(map_name), reset_settings, map_layer);

_episode等待50ms，_episode->WaitForState(50ms)。然后创建一个episode，auto episode = GetCurrentEpisode()。在World LoadWorld函数中，episode被当作参数，参与World类的构造， return World{_simulator->LoadEpisode(std::move(map_name), reset_settings, map_layers)};

**episode**实际上是**EpisodeProxyImpl**

```c++
struct EpisodeProxyPointerType {
    using Shared = std::shared_ptr<Simulator>;
    using Strong = AtomicSharedPtr<Simulator>;
    using Weak = std::weak_ptr<Simulator>;
}; 
using EpisodeProxy = EpisodeProxyImpl<EpisodeProxyPointerType::Strong>;
```

**episode**由**GetCurrentEpisode()**函数返回

```c++
 EpisodeProxy Simulator::GetCurrentEpisode() {
    if (_episode == nullptr) {
      ValidateVersions(_client);
      _episode = std::make_shared<Episode>(_client);
      _episode->Listen();
      if (!GetEpisodeSettings().synchronous_mode) {
        WaitForTick(_client.GetTimeout());
      }
      _light_manager->SetEpisode(EpisodeProxy{shared_from_this()});
    }
    return EpisodeProxy{shared_from_this()};
  }
```



**class EpisodeProxyImpl**

```C++
 template <typename PointerT>
  class EpisodeProxyImpl {
  public:
..............................................
    EpisodeProxyImpl(SharedPtrType simulator);
    
   	auto GetId() const noexcept {
      return _episode_id;
    }
..............................................
    uint64_t _episode_id;
    PointerT _simulator;
  };

```

```C++
  template <typename T>
  EpisodeProxyImpl<T>::EpisodeProxyImpl(SharedPtrType simulator)
    : _episode_id(simulator != nullptr ? simulator->GetCurrentEpisodeId() : 0u),
      _simulator(std::move(simulator)) {}
```



**class World ** 仅有一个成员变量detail::EpisodeProxy _episode;

```c++
explicit World(detail::EpisodeProxy episode) : _episode(std::move(episode)) {}

private:
    detail::EpisodeProxy _episode;
```

