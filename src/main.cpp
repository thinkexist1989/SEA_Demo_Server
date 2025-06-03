//
// Created by think on 2025/5/19.
//
#include <iostream>
#include <thread>
#include <zmq.hpp>

#include "sea.pb.h"
#include "sea_control.hpp"

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