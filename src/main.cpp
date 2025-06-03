//
// Created by think on 2025/5/19.
//
#include <boost/sml.hpp>
#include <iostream>
#include <thread>
#include <zmq.hpp>

#include "sea.pb.h"
#include "sea_control.hpp"

#include <memory>

using namespace rocos;

namespace {

namespace sml = boost::sml;
SeaControl* seaControl = nullptr;

// 状态定义
class DISABLED {};
class STOPPED {};
class RUNNING {};
class ERROR {};
class STARTING {}; // 过渡状态
class STOPPING {}; // 过渡状态

// 事件定义
struct EventInit_REQ {};
struct EventStart_REQ {};
struct EventSuccess {};
struct EventReset_REQ {};
struct EventErrorOccurred {};
struct EventStop_REQ {};

// 动作定义
const auto action_run = []() {
  spdlog::info("Running.....");

};

const auto action_stop = []() {
  spdlog::info("Stopping.....");

};

const auto action_start = []() {
  spdlog::info("Starting.....");
};


// 状态机定义
struct StateMachine {
  auto operator()() const noexcept {
    using namespace sml;

    return make_transition_table(
        // 初始状态
        *state<class DISABLED> + event<EventInit_REQ> = state<class STOPPED>,
        // 状态切换
        state<class STOPPED> + event<EventStart_REQ> = state<class STARTING>,
        state<class STARTING> + sml::on_entry<_> / action_start,
        state<class STARTING> + event<EventSuccess> = state<class RUNNING>,
        state<class RUNNING> + sml::on_entry<_> / action_run,

        state<class RUNNING> + event<EventStop_REQ> = state<class STOPPING>,
        state<class STOPPING> + sml::on_entry<_> / action_stop,
        state<class STOPPING> + event<EventSuccess> = state<class STOPPED>,

        state<class ERROR> + event<EventReset_REQ> = state<class DISABLED>,

        state<class RUNNING> + event<EventErrorOccurred> = state<class ERROR>,
        state<class STOPPED> + event<EventErrorOccurred> = state<class ERROR>,
        state<class DISABLED> + event<EventErrorOccurred> = state<class ERROR>,
        state<class STARTING> + event<EventErrorOccurred> = state<class ERROR>,
        state<class STOPPING> + event<EventErrorOccurred> = state<class ERROR>
    );
  }
};

sml::sm<StateMachine> sm{}; //状态机

}  // anonymous namespace

int main() {

  seaControl = new SeaControl("config/control.yml");







//  std::cout << "Hello, World!" << std::endl;
//
//  zmq::context_t context(1);
//  zmq::socket_t socket(context, zmq::socket_type::pub);
//  socket.bind("tcp://*:6060");
//
//  while (true) {
//    sea::ControlFeedback feedback;
//
//    sea::StatusFeedback* statusFeedback = feedback.mutable_status();
//
//    statusFeedback->set_run_state(sea::RUNNING);
//    statusFeedback->set_work_mode(sea::IMPEDANCE);
//    statusFeedback->set_stiffness(10);
//    statusFeedback->set_damping(10);
//    statusFeedback->set_current_position(0.5);
//
//    std::string serialized;
//    if (!feedback.SerializeToString(&serialized)) {
//      std::cerr << "Failed to serialize statusFeedback." << std::endl;
//      continue;
//    }
//
//    zmq::message_t msg(serialized.size());
//    memcpy(msg.data(), serialized.data(), serialized.size());
//
//    socket.send(msg, zmq::send_flags::none);
//
//    std::this_thread::sleep_for(std::chrono::seconds(1));
//  }
//
//  return 0;
}