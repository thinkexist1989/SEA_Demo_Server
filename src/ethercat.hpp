//
// Created by think on 6/3/25.
//

#pragma once

#include <iostream>
#include <cstdint>
#include <iomanip>

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

enum class StateTransition : uint8_t { _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _15 };

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
  friend std::ostream &operator<<(std::ostream &os, const Statusword &statusword);

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
    }else {
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

std::ostream& operator<<(std::ostream& os, const Statusword& statusword) {
  using std::setfill;
  using std::setw;
  std::string driveStateString = statusword.getDriveStateString();
  int gapSize2 = driveStateString.size() + 1;
  if (gapSize2 < 6) {
    gapSize2 = 6;
  }
  os << std::left << std::boolalpha << setw(gapSize2 + 27) << setfill('-') << "|"
     << "|\n"
     << setw(gapSize2 + 27) << setfill(' ') << "| Statusword"
     << "|\n"
     << setw(gapSize2 + 27) << setfill('-') << "|"
     << "|\n"
     << setw(25) << setfill(' ') << "| Name of Bit" << setw(gapSize2 + 2) << "| Value"
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


}

