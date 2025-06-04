//
// Created by think on 5/30/25.
//

#ifndef SEA_DEMO_SERVER_SEA_CONTROL_HPP
#define SEA_DEMO_SERVER_SEA_CONTROL_HPP

#include <rocos_ecm/ecat_config.h>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include "ethercat.hpp"

#include <fstream>
#include <thread>

#define WORK_MODE_IMPEDANCE 0
#define WORK_MODE_ZERO_FORCE 1
#define WORK_MODE_POSITION 2
#define WORK_MODE_VELOCITY 3



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

  void Init();
  void Run();
  void Stop();
  void Reset();

  void SetWorkMode(int mode) {
    work_mode_ = mode;
  }

  void SetVelocity(double vel) { // 低速端 rpm
    int32_t order = vel * hw_.ratio; // Synapticon Target Velocity是高速端rpm
    setTargetVelocityRaw(0, order);
  }

  void loadConfig(const std::string& config_file);

  void saveConfig(const std::string& config_file) const;

  void start();

  void run();

  void stop();

  void init();

  void reset();

 private:
  YAML::Node config_;
  Ecat ecat_;
  Hardware hw_;
  Impedance imp_;

  EcatConfig* ecat_cfg_;

  int work_mode_ {WORK_MODE_IMPEDANCE}; // 工作模式

  std::string name_ {"sea_joint"}; // 关节名称
  Statusword statusword_{}; // 状态字
  Controlword controlword_{}; // 控制字
  DriveState current_drive_state_{DriveState::NA}; // 驱动器当前状态
  DriveState target_drive_state_{DriveState::NA}; // 驱动器目标状态

  ModeOfOperation mode_{ModeOfOperation::CyclicSynchronousPositionMode};

  bool conduct_state_change_{false}; //是否启动状态机 默认不启动
  std::atomic<bool> state_change_successful_{false}; //当前状态切换是否成功

  std::chrono::system_clock::time_point drive_state_change_time_point_;

  uint16_t num_of_successful_target_state_readings_{0};

  std::shared_ptr<std::thread> guard_thread_{nullptr}; // 保护线程
  std::shared_ptr<std::thread> run_thread_{nullptr}; // 处理线程

  std::atomic<bool> is_guard_thread_running_{false}; // 线程运行标志

 private:

  void process_pdo_mapping();

  void impedance_handler();

  void velocity_handler();

  void position_handler();

  void zero_force_handler();

  void setTargetPositionRaw(int id, int32_t pos);

  void setTargetVelocityRaw(int id, int32_t vel);

  void setTargetTorqueRaw(int id, int16_t tor);

  int32_t getActualPositionRaw(int id);

  int32_t getActualVelocityRaw(int id);

  int16_t getActualTorqueRaw(int id);

  int32_t getSecondaryPositionRaw(int id);

  int32_t getSecondaryVelocityRaw(int id);

  uint16_t getStatuswordRaw(int id);

  void setControlwordRaw(int id, uint16_t ctrlwd);

  void setModeOfOperationRaw(int id, int8_t mode);

  void waitForSignal(int id);

  DriveState getDriverState(int id);

  /// \brief 获取下一个状态控制字
  /// \param requestedDriveState 请求的驱动器状态
  /// \param currentDriveState  当前的驱动器状态
  /// \return 返回控制字
  Controlword getNextStateTransitionControlword(const DriveState &requestedDriveState,
                                                       const DriveState &currentDriveState);


  /// \brief 处理状态机，在DriveGuard线程中循环调用
  void engageStateMachine();

  bool setDriverState(const DriveState &driveState, bool waitForState = true);

};

}  // namespace rocos

#endif  // SEA_DEMO_SERVER_SEA_CONTROL_HPP
