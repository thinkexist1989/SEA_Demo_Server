//
// Created by think on 5/30/25.
//

#include "sea_control.hpp"

#include <boost/sml.hpp>

using namespace rocos;

namespace {

namespace sml = boost::sml;
SeaControl* seaControl = nullptr;

// 状态定义
class DISABLED {};
class STOPPED {};
class RUNNING {};
class ERROR {};
class STARTING {};  // 过渡状态
class STOPPING {};  // 过渡状态

// 事件定义
struct EventInit_REQ {};
struct EventStart_REQ {};
struct EventSuccess {};
struct EventReset_REQ {};
struct EventErrorOccurred {};
struct EventStop_REQ {};

// 动作定义
const auto action_run = []() { spdlog::info("Running....."); };

const auto action_stop = []() { spdlog::info("Stopping....."); };

const auto action_start = []() { spdlog::info("Starting....."); };

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
        state<class STOPPING> + event<EventErrorOccurred> = state<class ERROR>);
  }
};

sml::sm<StateMachine> sm{};  // 状态机

}  // anonymous namespace

SeaControl::SeaControl(const std::string& config_file) {
  seaControl = this;  // 设置全局实例指针
  loadConfig(config_file);
}

void SeaControl::loadConfig(const std::string& config_file) {
  try {
    config_ = YAML::LoadFile(config_file);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("Failed to load configuration: " +
                             std::string(e.what()));
  }

  if (!config_["ethercat"]) {
    spdlog::error("Can not find [ethercat] TAG. ");
  }

  auto ecat = config_["ethercat"];
  ecat_.position_actual_value.name =
      ecat["position_actual_value"].as<std::string>("Position actual value");
  ecat_.velocity_actual_value.name =
      ecat["velocity_actual_value"].as<std::string>("Velocity actual value");
  ecat_.torque_actual_value.name =
      ecat["torque_actual_value"].as<std::string>("Torque actual value");
  ecat_.secondary_position_value.name =
      ecat["secondary_position_value"].as<std::string>(
          "Secondary position value");
  ecat_.secondary_velocity_value.name =
      ecat["secondary_velocity_value"].as<std::string>(
          "Secondary velocity value");
  ecat_.status_word.name = ecat["status_word"].as<std::string>("Statusword");
  ecat_.control_word.name = ecat["control_word"].as<std::string>("Controlword");
  ecat_.mode_of_operation.name =
      ecat["mode_of_operation"].as<std::string>("Mode of operation");
  ecat_.target_position.name =
      ecat["target_position"].as<std::string>("Target Position");
  ecat_.target_velocity.name =
      ecat["target_velocity"].as<std::string>("Target Velocity");
  ecat_.target_torque.name =
      ecat["target_torque"].as<std::string>("Target Torque");

  process_pdo_mapping();  // 处理PDO和共享内存变量关联

  if (!config_["hardware"]) {
    spdlog::error("Can not find [hardware] TAG. ");
  }

  auto hw = config_["hardware"];
  hw_.high_encoder_ppr = hw["high_encoder_ppr"].as<int>(262144);
  hw_.low_encoder_ppr = hw["low_encoder_ppr"].as<int>(65536);
  hw_.ratio = hw["ratio"].as<double>(100.0);
  hw_.spring_stiffness = hw["spring_stiffness"].as<double>(126.0507);

  if (!config_["impedance"]) {
    spdlog::error("Can not find [impedance] TAG. ");
  }

  auto imp = config_["impedance"];
  imp_.stiffness = imp["stiffness"].as<double>(7.0);
  imp_.damping = imp["damping"].as<double>(0.5);
}

void SeaControl::saveConfig(const std::string& config_file) const {
  try {
    YAML::Emitter out;
    out << config_;
    std::ofstream fout(config_file);
    fout << out.c_str();
    fout.close();
  } catch (const YAML::Exception& e) {
    spdlog::error("Failed to save configuration: {}", e.what());
  }
}

void SeaControl::process_pdo_mapping() {
  ecat_cfg_ = EcatConfig::getInstance(0);  // 获取EcatConfig实例

  if (ecat_cfg_->getSlaveNum() < 1) {
    spdlog::error("No slave found in EtherCAT bus. Program Exit. ");
    exit(-1);
  }

  ecat_.position_actual_value.p =
      ecat_cfg_->findSlaveInputVarPtrByName<int32_t>(
          0, ecat_.position_actual_value.name);
  ecat_.velocity_actual_value.p =
      ecat_cfg_->findSlaveInputVarPtrByName<int32_t>(
          0, ecat_.velocity_actual_value.name);
  ecat_.torque_actual_value.p =
      ecat_cfg_->findSlaveInputVarPtrByName<int16_t>(
          0, ecat_.torque_actual_value.name);
  ecat_.secondary_position_value.p =
      ecat_cfg_->findSlaveInputVarPtrByName<int32_t>(
          0, ecat_.secondary_position_value.name);
  ecat_.secondary_velocity_value.p =
      ecat_cfg_->findSlaveInputVarPtrByName<int32_t>(
          0, ecat_.secondary_velocity_value.name);
  ecat_.status_word.p = ecat_cfg_->findSlaveInputVarPtrByName<int16_t>(
      0, ecat_.status_word.name);

  ecat_.mode_of_operation.p = ecat_cfg_->findSlaveOutputVarPtrByName<int8_t>(
      0, ecat_.mode_of_operation.name);
  ecat_.control_word.p = ecat_cfg_->findSlaveOutputVarPtrByName<uint16_t>(
      0, ecat_.control_word.name);
  ecat_.target_position.p = ecat_cfg_->findSlaveOutputVarPtrByName<int32_t>(
      0, ecat_.target_position.name);
  ecat_.target_velocity.p = ecat_cfg_->findSlaveOutputVarPtrByName<int32_t>(
      0, ecat_.target_velocity.name);
  ecat_.target_torque.p = ecat_cfg_->findSlaveOutputVarPtrByName<int16_t>(
      0, ecat_.target_torque.name);
}

void SeaControl::start() {

}

void SeaControl::run() {

}

void SeaControl::stop() {

}

void SeaControl::impedance_handler() {

}

void SeaControl::velocity_handler() {

}

void SeaControl::position_handler() {

}

void SeaControl::zero_force_handler() {

}
