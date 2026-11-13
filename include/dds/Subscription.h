#ifndef _UT_ROBOT_SUBSCRIPTION_H_
#define _UT_ROBOT_SUBSCRIPTION_H_

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <functional>
#include <mutex>
#include <thread>
#include <spdlog/spdlog.h>

namespace unitree
{
namespace robot
{

template <typename MessageType>
class SubscriptionBase
{
public:
  using MsgType = MessageType;
  using SharedPtr = std::shared_ptr<SubscriptionBase<MsgType>>;
  
  // `seed` is what latest() returns until a sample arrives. `accept` filters samples before
  // they are stored: a rejected one is counted and leaves the newest accepted sample and its
  // arrival time in place, so it can never displace a good one.
  SubscriptionBase(const std::string& topic, MessageType seed = MessageType(),
                   std::function<bool(const MessageType&)> accept = nullptr)
    : msg_(std::move(seed)), accept_(std::move(accept))
  {
    last_update_time_ = std::chrono::steady_clock::now() - std::chrono::milliseconds(timeout_ms_);
    sub_ = std::make_shared<unitree::robot::ChannelSubscriber<MessageType>>(topic);
    sub_->InitChannel([this](const void *msg){
      const auto& sample = *(const MessageType*)msg;
      std::lock_guard<std::mutex> lock(mutex_);
      if (accept_ && !accept_(sample)) {
        ++rejected_;
        return;
      }
      last_update_time_ = std::chrono::steady_clock::now();
      accepted_any_ = true;
      msg_ = sample;
      post_communication();
    });
  }

  void set_timeout_ms(uint32_t timeout_ms) { timeout_ms_ = timeout_ms; }

  // The newest accepted sample, when it arrived (time_point::min() for the seed) and how many
  // were rejected so far, read together under the lock the callback writes them under:
  // reading msg_ directly races it.
  MessageType latest(std::chrono::steady_clock::time_point* arrived, uint64_t* rejected = nullptr) {
    std::lock_guard<std::mutex> lock(mutex_);
    *arrived = accepted_any_ ? last_update_time_ : std::chrono::steady_clock::time_point::min();
    if (rejected) *rejected = rejected_;
    return msg_;
  }

  bool isTimeout() {
    auto now = std::chrono::steady_clock::now();
    auto elasped_time = now - last_update_time_;
    return elasped_time > std::chrono::milliseconds(timeout_ms_);
  }

  void wait_for_connection() {
    auto t0 = std::chrono::steady_clock::now();
    bool warn_info = false;
    while(isTimeout()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (!warn_info && std::chrono::steady_clock::now() - t0 > std::chrono::seconds(2)) {
        warn_info = true;
        spdlog::warn("Waiting for connection {}", sub_->GetChannelName());
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // wait for stable communicaiton
    if (warn_info) {
      spdlog::info("Connected {}", sub_->GetChannelName());
    }
  }


  MessageType msg_;
protected:
  virtual void post_communication() {} // something after receiving message

  uint32_t timeout_ms_{1000};
  std::mutex mutex_;
  std::chrono::steady_clock::time_point last_update_time_;
  std::function<bool(const MessageType&)> accept_;
  uint64_t rejected_ = 0;
  bool accepted_any_ = false;
  // Declared last so it is destroyed first: its callback reads every member above.
  unitree::robot::ChannelSubscriberPtr<MessageType> sub_;
};


}; // namespace robot
}; // namespace unitree

#endif // _UT_ROBOT_SUBSCRIPTION_H_