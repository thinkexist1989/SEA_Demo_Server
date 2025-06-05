//
// Created by think on 6/5/25.
//
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <zmq.hpp>
#include "sea.pb.h"

TEST_CASE("client") {
        // 测试客户端连接
        zmq::context_t context(1);
        zmq::socket_t socket(context, zmq::socket_type::req);
        socket.connect("tcp://localhost:6060");

        // 发送请求
        sea::ControlCommand command;
        command.mutable_set_work_mode()->set_work_mode(sea::VELOCITY);

        std::string serialized;
        if (!command.SerializeToString(&serialized)) {
          std::cerr << "Failed to serialize statusFeedback." << std::endl;
          return;
        }
        zmq::message_t msg(serialized.size());
        memcpy(msg.data(), serialized.data(), serialized.size());
        socket.send(msg, zmq::send_flags::none);

        // 接收响应

        zmq::message_t response;
        socket.recv(response, zmq::recv_flags::none);

        sea::ControlFeedback feedback;
        if (!feedback.ParseFromArray(response.data(), response.size())) {
            std::cerr << "Failed to parse ControlFeedback." << std::endl;
            return;
        }


        // 发送请求
        {
          sea::ControlCommand command;
          command.mutable_set_velocity()->set_velocity(1.0);

          if (!command.SerializeToString(&serialized)) {
            std::cerr << "Failed to serialize statusFeedback." << std::endl;
            return;
          }
          zmq::message_t msg(serialized.size());
          memcpy(msg.data(), serialized.data(), serialized.size());
          socket.send(msg, zmq::send_flags::none);

          // 接收响应

          zmq::message_t response;
          socket.recv(response, zmq::recv_flags::none);

          sea::ControlFeedback feedback;
          if (!feedback.ParseFromArray(response.data(), response.size())) {
            std::cerr << "Failed to parse ControlFeedback." << std::endl;
            return;
          }
        }
        

}