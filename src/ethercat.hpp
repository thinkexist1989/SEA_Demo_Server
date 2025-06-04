//
// Created by think on 6/3/25.
//

#pragma once

#include <cstdint>
#include <iomanip>
#include <iostream>

namespace rocos {

enum class DriveState : uint8_t {
  NotReadyToSwitchOn,
  SwitchOnDisabled,
  ReadyToSwitchOn,
  SwitchedOn,
  OperationEnabled,
  QuickStopActive,
  FaultReactionActive,
  Fault,
  NA
};

enum class StateTransition : uint8_t {
  _2,
  _3,
  _4,
  _5,
  _6,
  _7,
  _8,
  _9,
  _10,
  _11,
  _12,
  _15
};

enum class ModeOfOperation : int8_t {
  NA = 0,
  ProfiledPositionMode = 1,
  ProfiledVelocityMode = 3,
  ProfiledTorqueMode = 4,
  HomingMode = 6,
  CyclicSynchronousPositionMode = 8,
  CyclicSynchronousVelocityMode = 9,
  CyclicSynchronousTorqueMode = 10
};

class Statusword {
 private:
  bool readyToSwitchOn_{false};      // bit 0
  bool switchedOn_{false};           // bit 1
  bool operationEnabled_{false};     // bit 2
  bool fault_{false};                // bit 3
  bool voltageEnabled_{false};       // bit 4
  bool quickStop_{false};            // bit 5
  bool switchOnDisabled_{false};     // bit 6
  bool warning_{false};              // bit 7
  bool targetReached_{false};        // bit 10
  bool internalLimitActive_{false};  // bit 11
  bool followingError_{false};       // bit 13, CSV mode

  // the raw statusword
  uint16_t rawStatusword_{0};

 public:
  friend std::ostream& operator<<(std::ostream& os,
                                  const Statusword& statusword);

  void setFromRawStatusword(uint16_t status) {
    readyToSwitchOn_ = static_cast<bool>(status & 1 << (0));
    switchedOn_ = static_cast<bool>(status & 1 << (1));
    operationEnabled_ = static_cast<bool>(status & 1 << (2));
    fault_ = static_cast<bool>(status & 1 << (3));
    voltageEnabled_ = static_cast<bool>(status & 1 << (4));
    quickStop_ = static_cast<bool>(status & 1 << (5));
    switchOnDisabled_ = static_cast<bool>(status & 1 << (6));
    warning_ = static_cast<bool>(status & 1 << (7));
    targetReached_ = static_cast<bool>(status & 1 << (10));
    internalLimitActive_ = static_cast<bool>(status & 1 << (11));
    followingError_ = static_cast<bool>(status & 1 << (13));

    rawStatusword_ = status;
  }

  uint16_t getRawStatusword() { return rawStatusword_; }

  DriveState getDriveState() const {
    DriveState driveState = DriveState::NA;

    // MAN-G-DS402 manual page 47
    if ((rawStatusword_ & 0b0000000001001111) == 0b0000000000000000) {
      driveState = DriveState::NotReadyToSwitchOn;
    } else if ((rawStatusword_ & 0b0000000001001111) == 0b0000000001000000) {
      driveState = DriveState::SwitchOnDisabled;
    } else if ((rawStatusword_ & 0b0000000001101111) == 0b0000000000100001) {
      driveState = DriveState::ReadyToSwitchOn;
    } else if ((rawStatusword_ & 0b0000000001101111) == 0b0000000000100011) {
      driveState = DriveState::SwitchedOn;
    } else if ((rawStatusword_ & 0b0000000001101111) == 0b0000000000100111) {
      driveState = DriveState::OperationEnabled;
    } else if ((rawStatusword_ & 0b0000000001101111) == 0b0000000000000111) {
      driveState = DriveState::QuickStopActive;
    } else if ((rawStatusword_ & 0b0000000001001111) == 0b0000000000001111) {
      driveState = DriveState::FaultReactionActive;
    } else if ((rawStatusword_ & 0b0000000001001111) == 0b0000000000001000) {
      driveState = DriveState::Fault;
    } else {
      driveState = DriveState::Fault;
    }

    return driveState;
  }

  std::string getDriveStateString() const {
    DriveState driveState = getDriveState();
    switch (driveState) {
      case DriveState::SwitchOnDisabled:
        return "switch on disabled";
        break;
      case DriveState::ReadyToSwitchOn:
        return "ready to switch on";
        break;
      case DriveState::SwitchedOn:
        return "switched on";
        break;
      case DriveState::OperationEnabled:
        return "operation enabled";
        break;
      case DriveState::QuickStopActive:
        return "quick stop active";
        break;
      case DriveState::Fault:
        return "fault_";
        break;
      case DriveState::FaultReactionActive:
        return "fault_ reaction active";
      case DriveState::NotReadyToSwitchOn:
        return "not ready to switch on";
      default:
        return "N/A";
    }
  }
};

struct Controlword {
  bool switchOn_{false};              // bit 0
  bool enableVoltage_{false};         // bit 1
  bool quickStop_{false};             // bit 2
  bool enableOperation_{false};       // bit 3
  bool newSetPoint_{false};           // bit 4 profiled position mode
  bool homingOperationStart_{false};  // bit 4 homing mode
  bool changeSetImmediately_{false};  // bit 5 profiled position mode
  bool relative_{false};              // bit 6 profiled position mode
  bool faultReset_{false};            // bit 7
  bool halt_{false};                  // bit 8

  void setFromRawControlword(uint16_t ctrlwd) {
    switchOn_ = (ctrlwd >> 1) & 0x01;
    enableVoltage_ = (ctrlwd >> 1) & 0x01;
    quickStop_ = (ctrlwd >> 2) & 0x01;
    enableOperation_ = (ctrlwd >> 3) & 0x01;
    // 4, 5, 6 homing,pp specific = false
    faultReset_ = (ctrlwd >> 7) & 0x01;
    halt_ = (ctrlwd >> 8) & 0x01;
  }
  /*!
   * get the control word as a 16 bit unsigned integer
   * THIS DOES NOT RESPECT THE MODE SPECIFIC OPTIONS!
   * The usually used cyclic modes do not need mode specific options.
   * @return	the raw controlword
   */
  uint16_t getRawControlword() {
    uint16_t rawControlword = 0;

    if (switchOn_) {
      rawControlword |= (1 << 0);
    }
    if (enableVoltage_) {
      rawControlword |= (1 << 1);
    }
    if (quickStop_) {
      rawControlword |= (1 << 2);
    }
    if (enableOperation_) {
      rawControlword |= (1 << 3);
    }
    if (faultReset_) {
      rawControlword |= (1 << 7);
    }
    if (halt_) {
      rawControlword |= (1 << 8);
    }

    return rawControlword;
  }

  /*!
   * State transition 2
   * SWITCH ON DISABLED -> READY TO SWITCH ON
   * This corresponds to a "shutdown" Controlword
   */
  void setStateTransition2() {
    setAllFalse();
    enableVoltage_ = true;
    quickStop_ = true;
  }

  /*!
   * State transition 3
   * READY TO SWITCH ON -> SWITCHED ON
   * This corresponds to a "switch on" Controlword
   */
  void setStateTransition3() {
    setAllFalse();
    switchOn_ = true;
    enableVoltage_ = true;
    quickStop_ = true;
  }

  /*!
   * State transition 4
   * SWITCHED ON -> ENABLE OPERATION
   */
  void setStateTransition4() {
    setAllFalse();
    switchOn_ = true;
    enableVoltage_ = true;
    quickStop_ = true;
    enableOperation_ = true;
  }

  /*!
   * State transition 5
   * OPERATION ENABLED -> SWITCHED ON
   * This corresponds to a "disable operation" Controlword
   */
  void setStateTransition5() {
    setAllFalse();
    switchOn_ = true;
    enableVoltage_ = true;
    quickStop_ = true;
  }

  /*!
   * State transition 6
   * SWITCHED ON -> READY TO SWITCH ON
   */
  void setStateTransition6() {
    setAllFalse();
    enableVoltage_ = true;
    quickStop_ = true;
  }

  /*!
   * State transition 7
   * READY TO SWITCH ON -> SWITCH ON DISABLED
   */
  void setStateTransition7() { setAllFalse(); }

  /*!
   * State transition 8
   * OPERATION ENABLED -> READY TO SWITCH ON
   */
  void setStateTransition8() {
    setAllFalse();
    enableVoltage_ = true;
    quickStop_ = true;
  }

  /*!
   * State transition 9
   * OPERATION ENABLED -> SWITCH ON DISABLED
   * This resets the elmo to the same state as on hardware startup
   * 0x0000
   */
  void setStateTransition9() { setAllFalse(); }

  /*!
   * State transition 10
   * SWITCHED ON -> SWITCH ON DISABLED
   * This Statusword is 0x0000
   */
  void setStateTransition10() { setAllFalse(); }

  /*!
   * State transition 11
   * OPERATION ENABLED -> QUICK STOP ACTIVE
   */
  void setStateTransition11() {
    setAllFalse();
    enableVoltage_ = true;
  }

  /*!
   * State transition 12
   * QUICK STOP ACTIVE -> SWITCH ON DISABLED
   */
  void setStateTransition12() { setAllFalse(); }

  /*!
   * State transition 15
   * FAULT -> SWITCH ON DISABLED
   */
  void setStateTransition15() {
    setAllFalse();
    faultReset_ = true;
  }

  /*!
   * Sets all bools of this struct to false
   */
  void setAllFalse() {
    switchOn_ = false;
    enableVoltage_ = false;
    quickStop_ = false;
    enableOperation_ = false;
    newSetPoint_ = false;
    homingOperationStart_ = false;
    changeSetImmediately_ = false;
    relative_ = false;
    faultReset_ = false;
    halt_ = false;
  }

  /*!
   * goes to the init state
   * Alias for state transition 2
   */
  void setInit() { setStateTransition2(); }

  friend std::ostream& operator<<(std::ostream& os,
                                  const Controlword& controlword);
};

}  // namespace rocos
