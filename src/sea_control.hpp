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
  explicit SeaControl(const std::string& config_file);

  void loadConfig(const std::string& config_file);

  void saveConfig(const std::string& config_file) const;

  void start();

  void run();

  void stop();

 private:
  YAML::Node config_;
  Ecat ecat_;
  Hardware hw_;
  Impedance imp_;

  EcatConfig* ecat_cfg_;

  void process_pdo_mapping();

  void impedance_handler();

  void velocity_handler();

  void position_handler();

  void zero_force_handler();
};

}  // namespace rocos

#endif  // SEA_DEMO_SERVER_SEA_CONTROL_HPP
