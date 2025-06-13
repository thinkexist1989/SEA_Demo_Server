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

enum RunState {
  DISABLED = 0,
  STOPPED = 1,
  RUNNING = 2,
  UNKNOWN = 3,
  ERROR = 4,
  STARTING = 5,
  STOPPING = 6
};

enum WorkMode {
  IMPEDANCE = 0,
  ZERO_FORCE = 1,
  POSITION = 2,
  VELOCITY = 3,
};

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
  int high_sign {1};
  int low_sign {1};
};

struct Impedance {
  double stiffness{7.0};
  double damping{0.5};
};

class SeaControl {
 public:
  explicit SeaControl(const std::string& config_file);

  void Init();   // 初始化到Disable
  void Run();    // 上使能并运行
  void Stop();   // 停止并下使能
  void Reset();

//  bool SetEnable(bool enable);

  void SetWorkMode(int mode) {
    work_mode_ = mode;
  }

  std::string GetState() const;

  RunState GetRunState() const;
  WorkMode GetWorkMode() const {
    return static_cast<WorkMode>(work_mode_);
  }


  void SetVelocity(double vel) { // 低速端 rad/s
    int32_t order = vel * rad2rpm * hw_.ratio; // Synapticon Target Velocity是高速端rpm
    setTargetVelocityRaw(0, order);
  }

  void SetPosition(double pos, double max_vel = 1.0, double max_acc = 10.0) { // 高速端 rad
    int32_t order = pos * cnt_per_rad_high_;

    // 送入规划器
//    setTargetPositionRaw(0, order);
  }


  void SetStiffness(double stiffness) {
    imp_.stiffness = stiffness;
    config_["impedance"]["stiffness"] = stiffness;
  }

  void SetDamping(double damping) {
    imp_.damping = damping;
    config_["impedance"]["damping"] = damping;
  }



  double GetCurrentPosition();
  double GetCurrentVelocity();

  double GetEncoder1Feedback() { return GetCurrentPosition(); };
  double GetEncoder2Feedback() {
    return getSecondaryPositionRaw(0) / cnt_per_rad_low_;
  }

  double GetSpringAngle() {
    cur_pos_high_ = getActualPositionRaw(0); // 高速端 pos(cnt)
    cur_pos_low_ = getSecondaryPositionRaw(0); // 低速端 pos(cnt)

    inner_rad_ = hw_.high_sign * (cur_pos_high_ - offset_cnt_high_) / cnt_per_rad_high_; // 扭簧内圈弧度
    outer_rad_ = hw_.low_sign * (cur_pos_low_ - offset_cnt_low_) / cnt_per_rad_low_;    // 扭簧外圈弧度

    delta_rad_ = outer_rad_ - inner_rad_;      // 内外圈弧度差值

    return delta_rad_;
  }

  double GetExternalForce() {
    return -GetSpringAngle() * hw_.spring_stiffness; // 返回外力矩
  }


  /// \brief 对编码器进行零点对齐
  void alignEncoderZero() {
    // 对编码器进行零点对齐
    cur_pos_high_ = getActualPositionRaw(0); // 高速端 pos(cnt)

    cur_pos_low_ = getSecondaryPositionRaw(0); // 低速端 pos(cnt)
    cur_vel_low_ = getSecondaryVelocityRaw(0); // 低速端 vel(rpm)

    // 首次进入初始化
    offset_cnt_high_ = cur_pos_high_; // 以刚启动的位置作为初始位置
    offset_cnt_low_ = cur_pos_low_;

  }



  void loadConfig(const std::string& config_file);

  void saveConfig(const std::string& config_file) const;

  void start();

  void run();

  void stop();

  void init();

  void reset();

 public:
  const double rad2rpm = 60 / (2 * M_PI); // rad/s to rpm

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

  double cnt_per_rad_high_ {0.0}; // 高速端cnt/rad
  double cnt_per_rad_low_ {0.0}; // 低速端cnt/rad


  int cur_pos_high_ = 0.0;//高速端 pos(cnt)
  int cur_vel_high_ = 0.0;//高速端 vel(rpm?)
  int cur_tor_high_ = 0.0;//高速端 torque(1/1000 * 57)

  int cur_pos_low_ = 0.0;// 低速端 pos(cnt)
  int cur_vel_low_ = 0.0;// 低速端 vel(rpm)

  int offset_cnt_high_{0}; // 高速端cnt offset
  int offset_cnt_low_{0}; // 低速端cnt offset

  double inner_rad_ = 0.0;
  double outer_rad_ = 0.0;
  double delta_rad_ = 0.0; // 内外圈弧度差值

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
