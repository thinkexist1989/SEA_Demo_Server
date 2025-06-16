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
class UNKNOWN {};   // 未知状态

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
  seaControl->run();
};

const auto action_stop = []() {
  spdlog::info("Stopping.....");
  seaControl->stop();
};

const auto action_start = []() {
  spdlog::info("Starting.....");
  seaControl->start();
};

const auto action_init = []() {
  spdlog::info("Initializing.....");
  seaControl->init();
};

const auto action_reset = []() {
  spdlog::info("Resetting.....");
  seaControl->reset();
};

// 状态机定义
struct StateMachine {
  auto operator()() const noexcept {
    using namespace sml;

    return make_transition_table(
        // 初始状态
        *state<class UNKNOWN> + event<EventInit_REQ> = state<class DISABLED>,
        state<class DISABLED> / action_init = state<class STOPPED>,
        // state<class STOPPED> + sml::on_entry<_> / action_init,
        // 状态切换
        state<class STOPPED> + event<EventStart_REQ> = state<class STARTING>,
        state<class STARTING> + sml::on_entry<_> / action_start,
        state<class STARTING> + event<EventSuccess> = state<class RUNNING>,
        state<class RUNNING> + sml::on_entry<_> / action_run,

        state<class RUNNING> + event<EventStop_REQ> = state<class STOPPING>,
        state<class STOPPING> + sml::on_entry<_> / action_stop,
        state<class STOPPING> + event<EventSuccess> = state<class STOPPED>,

        state<class ERROR> + event<EventReset_REQ> / action_reset =
            state<class DISABLED>,

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

  seaControl->alignEncoderZero();
}

void SeaControl::loadConfig(const std::string& config_file) {
  try {
    auto temp = YAML::LoadFile(config_file);
    config_ = temp["sea"];
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
      ecat["mode_of_operation"].as<std::string>("Modes of operation");
  ecat_.target_position.name =
      ecat["target_position"].as<std::string>("Target position");
  ecat_.target_velocity.name =
      ecat["target_velocity"].as<std::string>("Target velocity");
  ecat_.target_torque.name =
      ecat["target_torque"].as<std::string>("Target torque");

  process_pdo_mapping();  // 处理PDO和共享内存变量关联

  if (!config_["hardware"]) {
    spdlog::error("Can not find [hardware] TAG. ");
  }

  auto hw = config_["hardware"];
  hw_.high_encoder_ppr = hw["high_encoder_ppr"].as<int>(262144);
  hw_.low_encoder_ppr = hw["low_encoder_ppr"].as<int>(65536);
  hw_.ratio = hw["ratio"].as<double>(100.0);
  hw_.spring_stiffness = hw["spring_stiffness"].as<double>(126.0507);

  cnt_per_rad_high_ = hw_.high_encoder_ppr * hw_.ratio / (2 * M_PI);
  cnt_per_rad_low_ = hw_.low_encoder_ppr / (2 * M_PI);

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
    YAML::Node node;
    node["sea"] = config_;
    out << node;
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
  ecat_.torque_actual_value.p = ecat_cfg_->findSlaveInputVarPtrByName<int16_t>(
      0, ecat_.torque_actual_value.name);
  ecat_.secondary_position_value.p =
      ecat_cfg_->findSlaveInputVarPtrByName<int32_t>(
          0, ecat_.secondary_position_value.name);
  ecat_.secondary_velocity_value.p =
      ecat_cfg_->findSlaveInputVarPtrByName<int32_t>(
          0, ecat_.secondary_velocity_value.name);
  ecat_.status_word.p =
      ecat_cfg_->findSlaveInputVarPtrByName<int16_t>(0, ecat_.status_word.name);

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
  // 这是主要是使能关节

  switch (work_mode_) {
    case WORK_MODE_POSITION:
      spdlog::info("Set mode to Position Mode.");
      mode_ = ModeOfOperation::
          CyclicSynchronousPositionMode;  // 设置工作模式为位置模式
      setTargetPositionRaw(
          0, getActualPositionRaw(0));  // 设置目标位置为当前实际位置
      break;
    case WORK_MODE_VELOCITY:
      spdlog::info("Set mode to Velocity Mode.");
      mode_ = ModeOfOperation::
          CyclicSynchronousVelocityMode;  // 设置工作模式为速度模式
      setTargetVelocityRaw(0, 0);         // 设置目标速度为0
      break;
    case WORK_MODE_ZERO_FORCE:
    case WORK_MODE_IMPEDANCE:
      spdlog::info("Set mode to Torque Mode.");
      mode_ = ModeOfOperation::
          CyclicSynchronousTorqueMode;  // 设置工作模式为力矩模式
      setTargetTorqueRaw(0, 0);         // 设置目标力矩为0
      break;
    default:
      break;
  }

  // 设置工作模式
  setModeOfOperationRaw(0, static_cast<int8_t>(mode_));  // 设置工作模式

  setDriverState(DriveState::OperationEnabled);  //
}

void SeaControl::run() {
  if (run_thread_ && run_thread_->joinable()) {
    spdlog::warn("Run thread is already running, stopping it first.");

    run_thread_->join();  // 等待当前线程结束
    // run_thread_.reset();  // 终止当前线程
  }

  // 这里是主循环，处理不同的工作模式
  switch (work_mode_) {
    case WORK_MODE_IMPEDANCE:
      spdlog::info("Starting Impedance Handler.");
      run_thread_ =
          std::make_shared<std::thread>(&SeaControl::impedance_handler, this);
      break;
    case WORK_MODE_ZERO_FORCE:

      run_thread_ =
          std::make_shared<std::thread>(&SeaControl::zero_force_handler, this);
      break;
    case WORK_MODE_POSITION:
      spdlog::info("Starting Position Handler.");
      run_thread_ =
          std::make_shared<std::thread>(&SeaControl::position_handler, this);
      break;
    case WORK_MODE_VELOCITY:
      spdlog::info("Starting Velocity Handler.");
      run_thread_ =
          std::make_shared<std::thread>(&SeaControl::velocity_handler, this);
      break;
    default:
      spdlog::error("Unknown work mode: {}", work_mode_);
      break;
  }
}

void SeaControl::stop() {
  // 这里是停止运动，下使能
  if (run_thread_ && run_thread_->joinable()) {
    spdlog::warn("Stopping...wait for run thread to finish.");

    run_thread_->join();  // 等待当前线程结束
  }

  setDriverState(DriveState::SwitchOnDisabled);  // 下使能
}

void SeaControl::init() {
  if (guard_thread_ && guard_thread_->joinable()) {
    spdlog::warn("Guard thread is already running, stopping it first.");
    is_guard_thread_running_ = false;
    guard_thread_->join();
  }

  is_guard_thread_running_ = true;

  guard_thread_ = std::make_shared<std::thread>([=]() {
    std::cout << "Drive Guard is running on thread "
              << std::this_thread::get_id() << std::endl;
    while (is_guard_thread_running_) {
      current_drive_state_ = getDriverState(0);

      if (current_drive_state_ == DriveState::Fault) {
        sm.process_event(EventErrorOccurred{});
        //        is_guard_thread_running_ = false;
        //        return;
      }

      if (conduct_state_change_) engageStateMachine();

      usleep(10000);
    }
    std::cout << "Drive Guard thread is terminated." << std::endl;
  });
}

void SeaControl::reset() {
  setTargetPositionRaw(0,
                       getActualPositionRaw(0));  // 设置目标位置为当前实际位置
  setTargetVelocityRaw(0, 0);                     // 设置目标速度为0
  setTargetTorqueRaw(0, 0);                       // 设置目标力矩为0
  setDriverState(DriveState::SwitchOnDisabled);
}

void SeaControl::impedance_handler() {
  int offset_cnt_high;  // 以刚启动的位置作为初始位置

  double target_torque = 0.0;

  int is_first = 0;  // 阻抗模式使用
  double pos = 0.0;
  double last_pos = 0.0;
  double vel = 0.0;
  double offset_torque = 0.0;       // 阻抗模式使用
  double last_offset_torque = 0.0;  // 阻抗模式使用
  double vel_torque = 0.0;          // 阻抗模式使用

  double command_torque = 0.0;  // 阻抗模式使用

  while (sm.is(sml::state<class RUNNING>)) {
    // 更新所需数据
    int cur_pos_high = getActualPositionRaw(0);  // 高速端 pos(cnt)
    int cur_vel_high = getActualVelocityRaw(0);  // 高速端 vel(rpm?)
    int cur_tor_high = getActualTorqueRaw(0);    // 高速端 torque(1/1000 * 57)

    int cur_pos_low = getSecondaryPositionRaw(0);  // 低速端 pos(cnt)
    int cur_vel_low = getSecondaryVelocityRaw(0);  // 低速端 vel(rpm)

    if (is_first < 5) {
      // 首次进入初始化
      offset_cnt_high = cur_pos_high;  // 以刚启动的位置作为初始位置
                                       //      offset_cnt_low = cur_pos_low;

      is_first++;  // 取消进入
    }

    double inner_rad = hw_.high_sign * (cur_pos_high - offset_cnt_high) /
                       cnt_per_rad_high_;  // 扭簧内圈弧度

    double sensor_torque = GetExternalForce();  // 检测到的外力矩

    pos = inner_rad;  // 以外圈编码器作为最终位置
    vel = (pos - last_pos) / CYCLE_TIME;

    target_torque = -(pos * imp_.stiffness + vel * imp_.damping);  // 目标力矩

    offset_torque = target_torque - sensor_torque;
    vel_torque = (offset_torque - last_offset_torque);

    command_torque = target_torque + (15 * offset_torque + 3.5 * vel_torque);

    if (std::abs(command_torque) > 57) {
      command_torque = command_torque > 0 ? 57 : -57;
    }

    int send_torque_value = command_torque / 57.0 * 1000;

    setTargetTorqueRaw(0, send_torque_value);

    last_pos = pos;
    last_offset_torque = offset_torque;

    waitForSignal(0);  // 等待信号
  }

  spdlog::info("Exit impedance control loop.");
}

void SeaControl::velocity_handler() {
  while (sm.is(sml::state<class RUNNING>)) {
    waitForSignal(0);  // 等待信号
  }

  spdlog::info("Exit velocity control loop.");
}

void SeaControl::position_handler() {

  input.current_position = {GetCurrentPosition()};  // 当前实际位置（低速端）
  input.max_velocity = {1.0};                  //
  input.max_acceleration = {10.0};              // 最大加速度
  input.max_jerk = {100.0};                     // 最大加加速度
  input.target_position = {input.current_position};   // 目标位置（低速端）
  input.target_velocity = {0.0};                    // 目标速度（低速端）
  input.target_acceleration = {0.0};                // 目标加速度（低速端）
  input.control_interface = ruckig::ControlInterface::Position;  // 控制接口为位置模式

  while (sm.is(sml::state<class RUNNING>)) {
    if(!is_target_pos_updated_) {
      continue;
    }

    auto result = ruckig.update(input, output);  // 更新Ruckig状态
    if (result == ruckig::Result::Working) {     // 正在运动
      setTargetPositionRaw(0, output.new_position[0] * cnt_per_rad_high_);
      output.pass_to_input(input);

    } else if (result == ruckig::Result::Finished) {  // 运动完成
      is_target_pos_updated_ = false;
      input.current_position[0] = GetCurrentPosition();  // 当前实际位置（低速端）
      input.target_position = input.current_position;

    } else if (result <= ruckig::Result::Error) {  // 发生错误

      input.target_position[0] = GetCurrentPosition();  // 更新目标位置为当前实际位置


      switch (result) {
        case ruckig::Result::ErrorInvalidInput:
          spdlog::error("Ruckig Error: Invalid input parameters.");
          break;
        case ruckig::Result::ErrorTrajectoryDuration:
          spdlog::error("Ruckig Error: Trajectory duration exceeds limits.");
          break;
        case ruckig::Result::ErrorPositionalLimits:
          spdlog::error("Ruckig Error: Trajectory exceeds positional limits.");
          break;
        case ruckig::Result::ErrorZeroLimits:
          spdlog::error("Ruckig Error: Zero limits conflict.");
          break;
        case ruckig::Result::ErrorExecutionTimeCalculation:
          spdlog::error("Ruckig Error: Execution time calculation error.");
          break;
        case ruckig::Result::ErrorSynchronizationCalculation:
          spdlog::error("Ruckig Error: Synchronization calculation error.");
          break;
        default:
          spdlog::error("Ruckig Error: Unclassified error.");
          break;
      }
    }

    waitForSignal(0);  // 等待信号
  }


  input.control_interface = ruckig::ControlInterface::Velocity;  // 控制接口为位置模式
  input.target_velocity = {0.0};
  input.target_acceleration = {0.0};
  while(ruckig.update(input, output) == ruckig::Result::Working) {  // 正在运动
    setTargetPositionRaw(0, output.new_position[0] * hw_.ratio * cnt_per_rad_high_);
    output.pass_to_input(input);
    waitForSignal(0);  // 等待信号
  }

  spdlog::info("Exit position control loop.");
}

void SeaControl::zero_force_handler() {
  while (sm.is(sml::state<class RUNNING>)) {
    waitForSignal(0);  // 等待信号
  }

  spdlog::info("Exit zero force control loop.");
}

void SeaControl::setTargetPositionRaw(int id, int32_t pos) {
  if (ecat_.target_position.p) *ecat_.target_position.p = pos;
}

void SeaControl::setTargetVelocityRaw(int id, int32_t vel) {
  if (ecat_.target_velocity.p) *ecat_.target_velocity.p = vel;
}

void SeaControl::setTargetTorqueRaw(int id, int16_t tor) {
  if (ecat_.target_torque.p) *ecat_.target_torque.p = tor;
}

int32_t SeaControl::getActualPositionRaw(int id) {
  return ecat_.position_actual_value.p ? *ecat_.position_actual_value.p : 0;
}

int32_t SeaControl::getActualVelocityRaw(int id) {
  return ecat_.velocity_actual_value.p ? *ecat_.velocity_actual_value.p : 0;
}

int16_t SeaControl::getActualTorqueRaw(int id) {
  return ecat_.torque_actual_value.p ? *ecat_.torque_actual_value.p : 0;
}

int32_t SeaControl::getSecondaryPositionRaw(int id) {
  return ecat_.secondary_position_value.p ? *ecat_.secondary_position_value.p
                                          : 0;
}

int32_t SeaControl::getSecondaryVelocityRaw(int id) {
  return ecat_.secondary_velocity_value.p ? *ecat_.secondary_velocity_value.p
                                          : 0;
}

uint16_t SeaControl::getStatuswordRaw(int id) {
  return ecat_.status_word.p ? *ecat_.status_word.p : 0;
}

void SeaControl::setControlwordRaw(int id, uint16_t ctrlwd) {
  if (ecat_.control_word.p) *ecat_.control_word.p = ctrlwd;
}

void SeaControl::setModeOfOperationRaw(int id, int8_t mode) {
  if (ecat_.mode_of_operation.p) *ecat_.mode_of_operation.p = mode;
}

void SeaControl::waitForSignal(int id) { ecat_cfg_->waitForSignal(id); }

Controlword SeaControl::getNextStateTransitionControlword(
    const DriveState& requestedDriveState,
    const DriveState& currentDriveState) {
  Controlword controlword;
  controlword.setAllFalse();
  switch (requestedDriveState) {
    case DriveState::SwitchOnDisabled:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                    << "drive state has already been reached" << name_ << "'"
                    << std::endl;
          //                        MELO_ERROR_STREAM("[elmo_ethercat_sdk:Elmo::getNextStateTransitionControlword]
          //                        "
          //                                                  << "drive state
          //                                                  has already been
          //                                                  reached for '"
          //                                                  << name_ << "'");
          //                        addErrorToReading(ErrorType::PdoStateTransitionError);
          break;
        case DriveState::ReadyToSwitchOn:
          controlword.setStateTransition7();
          break;
        case DriveState::SwitchedOn:
          controlword.setStateTransition10();
          break;
        case DriveState::OperationEnabled:
          controlword.setStateTransition9();
          break;
        case DriveState::QuickStopActive:
          controlword.setStateTransition12();
          break;
        case DriveState::Fault:
          controlword.setStateTransition15();
          break;
        default:
          std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                    << "PDO state transition not implemented for '" << name_
                    << "'" << std::endl;
      }
      break;

    case DriveState::ReadyToSwitchOn:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          controlword.setStateTransition2();
          break;
        case DriveState::ReadyToSwitchOn:
          std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                    << "drive state has already been reached for '" << name_
                    << "'" << std::endl;
          break;
        case DriveState::SwitchedOn:
          controlword.setStateTransition6();
          break;
        case DriveState::OperationEnabled:
          controlword.setStateTransition8();
          break;
        case DriveState::QuickStopActive:
          controlword.setStateTransition12();
          break;
        case DriveState::Fault:
          controlword.setStateTransition15();
          break;
        default:
          std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                    << "PDO state transition not implemented for '" << name_
                    << "'" << std::endl;
      }
      break;

    case DriveState::SwitchedOn:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          controlword.setStateTransition2();
          break;
        case DriveState::ReadyToSwitchOn:
          controlword.setStateTransition3();
          break;
        case DriveState::SwitchedOn:
          std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                    << "drive state has already been reached for '" << name_
                    << "'" << std::endl;
          break;
        case DriveState::OperationEnabled:
          controlword.setStateTransition5();
          break;
        case DriveState::QuickStopActive:
          controlword.setStateTransition12();
          break;
        case DriveState::Fault:
          controlword.setStateTransition15();
          break;
        default:
          std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                    << "PDO state transition not implemented for '" << name_
                    << "'" << std::endl;
      }
      break;

    case DriveState::OperationEnabled:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          controlword.setStateTransition2();
          break;
        case DriveState::ReadyToSwitchOn:
          controlword.setStateTransition3();
          break;
        case DriveState::SwitchedOn:
          controlword.setStateTransition4();
          break;
        case DriveState::OperationEnabled:
          std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                    << "drive state has already been reached for '" << name_
                    << "'" << std::endl;
          break;
        case DriveState::QuickStopActive:
          controlword.setStateTransition12();
          break;
        case DriveState::Fault:
          controlword.setStateTransition15();
          break;
        default:
          std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                    << "PDO state transition not implemented for '" << name_
                    << "'" << std::endl;
      }
      break;

    case DriveState::QuickStopActive:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          controlword.setStateTransition2();
          break;
        case DriveState::ReadyToSwitchOn:
          controlword.setStateTransition3();
          break;
        case DriveState::SwitchedOn:
          controlword.setStateTransition4();
          break;
        case DriveState::OperationEnabled:
          controlword.setStateTransition11();
          break;
        case DriveState::QuickStopActive:
          std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                    << "drive state has already been reached for '" << name_
                    << "'" << std::endl;
          break;
        case DriveState::Fault:
          controlword.setStateTransition15();
          break;
        default:
          std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                    << "PDO state transition not implemented for '" << name_
                    << "'" << std::endl;
      }
      break;

    default:
      std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                << "PDO state cannot be reached for '" << name_ << "'"
                << std::endl;
  }

  return controlword;
}

void SeaControl::engageStateMachine() {
  // elapsed time since the last new controlword
  auto microsecondsSinceChange =
      (std::chrono::duration_cast<std::chrono::microseconds>(
           std::chrono::system_clock::now() - drive_state_change_time_point_))
          .count();

  // get the current state
  // since we wait until "hasRead" is true, this is guaranteed to be a newly
  // read value
  //        const DriveState currentDriveState = reading_.getDriveState();

  // check if the state change already was successful:
  if (current_drive_state_ == target_drive_state_) {
    num_of_successful_target_state_readings_++;
    //            if (numberOfSuccessfulTargetStateReadings_ >=
    //            configuration_.minNumberOfSuccessfulTargetStateReadings) {
    if (num_of_successful_target_state_readings_ >= 3) {
      // disable the state machine
      conduct_state_change_ = false;
      num_of_successful_target_state_readings_ = 0;
      state_change_successful_ = true;

      sm.process_event(EventSuccess{});

      return;
    }
  } else if (microsecondsSinceChange > 20000) {
    // get the next controlword from the state machine
    controlword_ = getNextStateTransitionControlword(target_drive_state_,
                                                     current_drive_state_);
    drive_state_change_time_point_ = std::chrono::system_clock::now();
    setControlwordRaw(0, controlword_.getRawControlword());  // set control word
  }
}

bool SeaControl::setDriverState(const DriveState& driveState,
                                bool waitForState) {
  bool success = false;
  /*
   ** locking the mutex_
   ** This is not done with a lock_guard here because during the waiting time
   *the
   ** mutex_ must be unlocked periodically such that PDO writing (and thus state
   ** changes) may occur at all!
   */
  //        mutex_.lock();

  // reset the "stateChangeSuccessful_" flag to false such that a new successful
  // state change can be detected
  state_change_successful_ = false;

  // make the state machine realize that a state change will have to happen
  conduct_state_change_ = true;

  // overwrite the target drive state
  target_drive_state_ = driveState;

  // set the hasRead flag to false such that at least one new reading will be
  // available when starting the state change
  //        hasRead_ = false;

  // set the time point of the last pdo change to now
  drive_state_change_time_point_ = std::chrono::system_clock::now();

  // set a temporary time point to prevent getting caught in an infinite loop
  auto driveStateChangeStartTimePoint = std::chrono::system_clock::now();

  // return if no waiting is requested
  if (!waitForState) {
    // unlock the mutex
    //            mutex_.unlock();
    // return true if no waiting is requested
    return true;
  }

  // Wait for the state change to be successful
  // during the waiting time the mutex MUST be unlocked!

  while (true) {
    // break loop as soon as the state change was successful
    if (state_change_successful_) {
      success = true;
      break;
    }

    // break the loop if the state change takes too long
    // this prevents a freezing of the end user's program if the hardware is not
    // able to change it's state.
    auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now() - driveStateChangeStartTimePoint);
    if (duration_us.count() >
        150000) {  // wait for 100ms  TODO:
                   // configuration_.driveStateChangeMaxTimeout
      std::cout << "It takes too long (" << duration_us.count() / 1000.0
                << " ms) to switch state!" << std::endl;
      break;
    }
    // unlock the mutex during sleep time
    //            mutex_.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    // lock the mutex to be able to check the success flag
    //            mutex_.lock();
  }
  // unlock the mutex one last time
  //        mutex_.unlock();
  return success;
}

DriveState SeaControl::getDriverState(int id) {
  Statusword status;
  status.setFromRawStatusword(getStatuswordRaw(id));
  return status.getDriveState();
}
void SeaControl::Init() {
  sm.process_event(EventInit_REQ{});  // 触发初始化事件
}
void SeaControl::Run() { sm.process_event(EventStart_REQ{}); }
void SeaControl::Stop() { sm.process_event(EventStop_REQ{}); }
void SeaControl::Reset() { sm.process_event(EventReset_REQ{}); }
std::string SeaControl::GetState() const {
  if (sm.is(sml::state<class DISABLED>)) {
    return "DISABLED";
  } else if (sm.is(sml::state<class STOPPED>)) {
    return "STOPPED";
  } else if (sm.is(sml::state<class RUNNING>)) {
    return "RUNNING";
  } else if (sm.is(sml::state<class ERROR>)) {
    return "ERROR";
  } else if (sm.is(sml::state<class STARTING>)) {
    return "STARTING";
  } else if (sm.is(sml::state<class STOPPING>)) {
    return "STOPPING";
  }
  return "UNKNOWN";
}
RunState SeaControl::GetRunState() const {
  if (sm.is(sml::state<class DISABLED>)) {
    return DISABLED;
  } else if (sm.is(sml::state<class STOPPED>)) {
    return STOPPED;
  } else if (sm.is(sml::state<class RUNNING>)) {
    return RUNNING;
  } else if (sm.is(sml::state<class ERROR>)) {
    return ERROR;
  } else if (sm.is(sml::state<class STARTING>)) {
    return STARTING;
  } else if (sm.is(sml::state<class STOPPING>)) {
    return STOPPING;
  } else {
    return UNKNOWN;
  }
}
double SeaControl::GetCurrentPosition() {
  return getActualPositionRaw(0) / cnt_per_rad_high_;
}

double SeaControl::GetCurrentVelocity() {  // rad/s
  return getActualVelocityRaw(0) / hw_.ratio / rad2rpm;
}

// bool SeaControl::SetEnable(bool enable) {
//   if (enable)
//     return setDriverState(DriveState::OperationEnabled);
//   else
//     return setDriverState(DriveState::SwitchOnDisabled);
// }