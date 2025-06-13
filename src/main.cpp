//
// Created by think on 2025/5/19.
//
#include <iostream>
#include <thread>
#include <zmq.hpp>

#include "ethercat.hpp"
#include "sea.pb.h"
#include "sea_control.hpp"

bool isRunning = true;

void signalHandler(int signum) {
  std::cout << "Interrupt signal (" << signum << ") received.\n";
  isRunning = false;
}

  // namespace

using namespace rocos;

namespace rocos {

std::ostream& operator<<(std::ostream& os, const Statusword& statusword) {
  using std::setfill;
  using std::setw;
  std::string driveStateString = statusword.getDriveStateString();
  int gapSize2 = driveStateString.size() + 1;
  if (gapSize2 < 6) {
    gapSize2 = 6;
  }
  os << std::left << std::boolalpha << setw(gapSize2 + 27) << setfill('-')
     << "|"
     << "|\n"
     << setw(gapSize2 + 27) << setfill(' ') << "| Statusword"
     << "|\n"
     << setw(gapSize2 + 27) << setfill('-') << "|"
     << "|\n"
     << setw(25) << setfill(' ') << "| Name of Bit" << setw(gapSize2 + 2)
     << "| Value"
     << "|\n"
     << setw(25) << setfill('-') << "|" << setw(gapSize2 + 2) << "+"
     << "|\n"
     << setfill(' ') <<

      setw(25) << "| Ready to switch on:"
     << "| " << setw(gapSize2) << statusword.readyToSwitchOn_ << "|\n"
     << setw(25) << "| Switched on:"
     << "| " << setw(gapSize2) << statusword.switchedOn_ << "|\n"
     << setw(25) << "| Operation enabled:"
     << "| " << setw(gapSize2) << statusword.operationEnabled_ << "|\n"
     << setw(25) << "| Fault:"
     << "| " << setw(gapSize2) << statusword.fault_ << "|\n"
     << setw(25) << "| Voltage enabled:"
     << "| " << setw(gapSize2) << statusword.voltageEnabled_ << "|\n"
     << setw(25) << "| Quick stop:"
     << "| " << setw(gapSize2) << statusword.quickStop_ << "|\n"
     << setw(25) << "| Switch on disabled:"
     << "| " << setw(gapSize2) << statusword.switchOnDisabled_ << "|\n"
     << setw(25) << "| Warning:"
     << "| " << setw(gapSize2) << statusword.warning_ << "|\n"
     << setw(25) << "| Target reached:"
     << "| " << setw(gapSize2) << statusword.targetReached_ << "|\n"
     << setw(25) << "| Internal limit active:"
     << "| " << setw(gapSize2) << statusword.internalLimitActive_ << "|\n"
     <<
      // setw(25)<<"| Following error:"<<"|
      // "<<setw(gapSize2)<<statusword.followingError_<<"| \n"<< // mode of
      // operation specific
      setw(25) << setfill('-') << "|" << setw(gapSize2 + 2) << "+"
     << "|\n"
     << setfill(' ') << setw(25) << "| Resulting Drive State:"
     << "| " << setw(gapSize2) << driveStateString << "|\n"
     << setw(25) << setfill('-') << "|" << setw(gapSize2 + 2) << "+"
     << "|" <<

      std::noboolalpha << std::right << setfill(' ');

  return os;
}

std::ostream& operator<<(std::ostream& os, const DriveState& driveState) {
  switch (driveState) {
    case rocos::DriveState::NotReadyToSwitchOn:
      os << "NotReadyToSwitchOn";
      break;
    case rocos::DriveState::SwitchOnDisabled:
      os << "SwitchOnDisabled";
      break;
    case rocos::DriveState::ReadyToSwitchOn:
      os << "ReadyToSwitchOn";
      break;
    case rocos::DriveState::SwitchedOn:
      os << "SwitchedOn";
      break;
    case rocos::DriveState::OperationEnabled:
      os << "OperationEnabled";
      break;
    case rocos::DriveState::QuickStopActive:
      os << "QuickStopActive";
      break;
    case rocos::DriveState::FaultReactionActive:
      os << "FaultReactionActive";
      break;
    case rocos::DriveState::Fault:
      os << "Fault";
      break;
    case rocos::DriveState::NA:
      os << "NA";
      break;
  }
  return os;
}

std::ostream& operator<<(std::ostream& os, const Controlword& controlword) {
  using std::setfill;
  using std::setw;

  os << std::left << std::boolalpha << setw(40) << setfill('-') << "|"
     << "|\n"
     << setw(40) << setfill(' ') << "| Controlword"
     << "|\n"
     << setw(25) << setfill('-') << "|" << setw(8) << "+" << setw(7) << "+"
     << "|"
     << "\n"
     << setw(25) << setfill(' ') << "| Name"
     << "| Value | Mode |"
     << "\n"
     << setw(25) << setfill('-') << "|" << setw(8) << "+" << setw(7) << "+"
     << "|"
     << "\n"
     << setw(25) << setfill(' ') << "| switch on:"
     << "| " << setw(6) << controlword.switchOn_ << "|" << setw(6) << " all"
     << "|\n"
     << setw(25) << setfill(' ') << "| enable voltage:"
     << "| " << setw(6) << controlword.enableVoltage_ << "|" << setw(6)
     << " all"
     << "|\n"
     << setw(25) << setfill(' ') << "| quick stop:"
     << "| " << setw(6) << controlword.quickStop_ << "|" << setw(6) << " all"
     << "|\n"
     << setw(25) << setfill(' ') << "| enable operation:"
     << "| " << setw(6) << controlword.enableOperation_ << "|" << setw(6)
     << " all"
     << "|\n"
     << setw(25) << setfill(' ') << "| new set point:"
     << "| " << setw(6) << controlword.newSetPoint_ << "|" << setw(6) << " pp"
     << "|\n"
     << setw(25) << setfill(' ') << "| start homing:"
     << "| " << setw(6) << controlword.homingOperationStart_ << "|" << setw(6)
     << " hm"
     << "|\n"
     << setw(25) << setfill(' ') << "| change set:"
     << "| " << setw(6) << controlword.changeSetImmediately_ << "|" << setw(6)
     << " pp"
     << "|\n"
     << setw(25) << setfill(' ') << "| relative_:"
     << "| " << setw(6) << controlword.relative_ << "|" << setw(6) << " pp "
     << "|\n"
     << setw(25) << setfill(' ') << "| fault_ reset:"
     << "| " << setw(6) << controlword.faultReset_ << "|" << setw(6) << " all"
     << "|\n"
     << setw(25) << setfill(' ') << "| halt_:"
     << "| " << setw(6) << controlword.halt_ << "|" << setw(6) << " all"
     << "|\n"
     <<

      setw(25) << setfill('-') << "|" << setw(8) << "+" << setw(7) << "+"
     << "|" << std::right << std::noboolalpha;
  return os;
}

}  // namespace rocos

void test_velocity() {
  auto* seaControl = new SeaControl("sea_config.yml");

  std::cout << "State1: " << seaControl->GetState() << std::endl;

  std::cout << "State2: " << seaControl->GetState() << std::endl;

  usleep(1000000);  // 等待1秒钟

  std::cout << "State3: " << seaControl->GetState() << std::endl;

//  seaControl->Init();

  usleep(1000000);  // 等待1秒钟

  // seaControl->Reset();

  //   usleep(1000000);  // 等待1秒钟

  std::cout << "State4: " << seaControl->GetState() << std::endl;

  seaControl->SetWorkMode(WORK_MODE_VELOCITY);

  seaControl->Run();

  std::cout << "State6: " << seaControl->GetState() << std::endl;

  seaControl->SetVelocity(4.0);  // 设置速度为10.0 rpm

  std::this_thread::sleep_for(std::chrono::seconds(5));  // 运行5秒钟

  seaControl->SetVelocity(0.0);  // 设置速度为10.0 rpm

  seaControl->Stop();

  while (1) {
    usleep(1000000);  // 等待1秒钟
  }
}

void test_zmq() {
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

void zmq_server(SeaControl* seaControl) {
  zmq::context_t context(1);
  zmq::socket_t socket(context, zmq::socket_type::rep);
  socket.bind("tcp://*:6060");

  while (isRunning) {
    zmq::message_t request;
    socket.recv(request, zmq::recv_flags::none);

    sea::ControlCommand control_command;
    if (!control_command.ParseFromArray(request.data(), request.size())) {
      std::cerr << "Failed to parse ControlCommand." << std::endl;
      continue;
    }

    // 解析接收到的消息
    sea::ControlFeedback feedback;

    if (control_command.has_stop()) {

      seaControl->Stop();

      feedback.mutable_stop()->set_stop(true);

    } else if (control_command.has_get_config()) {
      auto* config = feedback.mutable_config();
      config->set_encoder1_resolution(seaControl->hw_.high_encoder_ppr);
      config->set_encoder2_resolution(seaControl->hw_.low_encoder_ppr);
      config->set_spring_stiffness(seaControl->hw_.spring_stiffness);

    } else if (control_command.has_get_status()) {
      auto* status = feedback.mutable_status();
      status->set_run_state(
          static_cast<sea::RunState>(seaControl->GetRunState()));
      status->set_work_mode(
          static_cast<sea::WorkMode>(seaControl->GetWorkMode()));
      status->set_stiffness(seaControl->imp_.stiffness);
      status->set_damping(seaControl->imp_.damping);
      status->set_current_position(seaControl->GetCurrentPosition());
      status->set_current_velocity(seaControl->GetCurrentVelocity());
      status->set_encoder1_feedback(seaControl->GetEncoder1Feedback());
      status->set_encoder2_feedback(seaControl->GetEncoder2Feedback());
      status->set_spring_angle(seaControl->GetSpringAngle());
      status->set_external_force(seaControl->GetExternalForce());


    } else if (control_command.has_set_velocity()) {
      
      double velocity = control_command.set_velocity().vel();

      spdlog::info("Setting velocity to: {}", velocity);

      seaControl->SetVelocity(velocity);

      feedback.mutable_set_velocity()->set_vel(velocity);

    } else if (control_command.has_set_position()) {
      double position = control_command.set_position().pos();
      double max_vel = control_command.set_position().max_vel();
      double max_acc = control_command.set_position().max_acc();

      spdlog::info("Setting position to: {}, max_vel: {}, max_acc: {}",
                   position, max_vel, max_acc);

      seaControl->SetPosition(position, max_vel, max_acc);

      feedback.mutable_set_position()->set_pos(position);
      feedback.mutable_set_position()->set_max_vel(max_vel);
      feedback.mutable_set_position()->set_max_acc(max_acc);

    } else if (control_command.has_set_damping()) {
      double damping = control_command.set_damping().damping();
      spdlog::info("Setting damping to: {}", damping);

      seaControl->SetDamping(damping);

      feedback.mutable_set_damping()->set_damping(damping);

    } else if (control_command.has_set_stiffness()) {
      double stiffness = control_command.set_stiffness().stiffness();
      spdlog::info("Setting stiffness to: {}", stiffness);

      seaControl->SetStiffness(stiffness);

      feedback.mutable_set_stiffness()->set_stiffness(stiffness);

    } else if (control_command.has_set_work_mode()) {
      seaControl->Stop();
      usleep(200000);  // 等待200毫秒以确保停止完成

      auto work_mode = control_command.set_work_mode().work_mode();
      spdlog::info("Setting work mode to: {}", (int)work_mode);

      seaControl->SetWorkMode(work_mode);

      seaControl->Run();

    } else if (control_command.has_reset()) {
      spdlog::info("Resetting....");

      seaControl->Reset();

      feedback.mutable_reset();

    } else {
      std::cerr << "Unknown command received." << std::endl;
      continue;
    }

    std::string serialized;
    if (!feedback.SerializeToString(&serialized)) {
      std::cerr << "Failed to serialize statusFeedback." << std::endl;
      continue;
    }

    zmq::message_t msg(serialized.size());
    memcpy(msg.data(), serialized.data(), serialized.size());

    socket.send(msg, zmq::send_flags::none);


  }
}

int main() {
  // 注册信号处理函数
  signal(SIGINT, signalHandler);

  std::cout << "ROCOS SEA Application" << std::endl;

  auto* seaControl = new SeaControl("sea_config.yml");

  seaControl->Init();  // 到Disabled状态


  // 启动ZMQ服务器线程
  std::thread zmqThread(zmq_server, seaControl);

  // 监测处理键盘输入
  while (isRunning) {
    std::string input;
    std::cout << "Enter command (v, p, i, z): ";
    std::getline(std::cin, input);

    if (input == "v") {
    } else if (input == "p") {
    } else if (input == "i") {
    } else if (input == "z") {
    } else {
      std::cout << "Unknown command: " << input << std::endl;
    }
  }
}
