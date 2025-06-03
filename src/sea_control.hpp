//
// Created by think on 5/30/25.
//

#ifndef SEA_DEMO_SERVER_SEA_CONTROL_HPP
#define SEA_DEMO_SERVER_SEA_CONTROL_HPP

#include <rocos_ecm/ecat_config.h>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <thread>

namespace rocos {

template <typename T>
struct PdoEntry {
  std::string name;
  T* p;
};

struct Ecat {
  PdoEntry<int32_t> position_actual_value;
  PdoEntry<int32_t> velocity_actual_value;
  PdoEntry<int16_t> torque_actual_value;
  PdoEntry<int32_t> secondary_position_value;
  PdoEntry<int32_t> secondary_velocity_value;
  PdoEntry<int16_t> status_word;
  PdoEntry<int8_t> mode_of_operation;
  PdoEntry<uint16_t> control_word;
  PdoEntry<int32_t> target_position;
  PdoEntry<int32_t> target_velocity;
  PdoEntry<int16_t> target_torque;
};

struct Hardware {
  int high_encoder_ppr{262144};
  int low_encoder_ppr{65536};
  double ratio{100.0};
  double spring_stiffness{126.0507};
};

struct Impedance {
  double stiffness{7.0};
  double damping{0.5};
};

class SeaControl {
 public:
  explicit SeaControl(const std::string& config_file) {
    loadConfig(config_file);
  }

  void loadConfig(const std::string& config_file) {
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
    ecat_.control_word.name =
        ecat["control_word"].as<std::string>("Controlword");
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

  void saveConfig(const std::string& config_file) const {
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

  







 private:
  YAML::Node config_;
  Ecat ecat_;
  Hardware hw_;
  Impedance imp_;

  EcatConfig* ecat_cfg_;

  bool is_running_ {false};  // 是否正在运行

  void process_pdo_mapping() {
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
};

}  // namespace rocos

#endif  // SEA_DEMO_SERVER_SEA_CONTROL_HPP
