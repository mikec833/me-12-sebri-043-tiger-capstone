#ifndef TIGERBALL_LATCH_CONTROLLER_H
#define TIGERBALL_LATCH_CONTROLLER_H

#include <stdint.h>

// Hardware-independent pulse state. The sketch applies this state to the relay.
class LatchController {
 public:
  LatchController(uint32_t pulseTimeMs, uint32_t cooldownTimeMs)
      : pulseTimeMs_(pulseTimeMs),
        cooldownTimeMs_(cooldownTimeMs),
        relayOn_(false),
        pulseCompleted_(false),
        relayOffTime_(0),
        lastPulseFinishedTime_(0) {}

  bool requestPulse(uint32_t currentTime) {
    if (relayOn_) {
      return false;
    }

    if (pulseCompleted_ &&
        static_cast<uint32_t>(currentTime - lastPulseFinishedTime_) <
            cooldownTimeMs_) {
      return false;
    }

    relayOn_ = true;
    relayOffTime_ = currentTime + pulseTimeMs_;
    return true;
  }

  // Returns true only when this call changes ON to OFF.
  bool update(uint32_t currentTime) {
    if (relayOn_ && deadlineReached(currentTime, relayOffTime_)) {
      forceOff(currentTime);
      return true;
    }
    return false;
  }

  void forceOff(uint32_t currentTime) {
    if (relayOn_) {
      lastPulseFinishedTime_ = currentTime;
      pulseCompleted_ = true;
    }
    relayOn_ = false;
  }

  bool isOn() const { return relayOn_; }

 private:
  static bool deadlineReached(uint32_t currentTime, uint32_t deadline) {
    return static_cast<int32_t>(currentTime - deadline) >= 0;
  }

  const uint32_t pulseTimeMs_;
  const uint32_t cooldownTimeMs_;
  bool relayOn_;
  bool pulseCompleted_;
  uint32_t relayOffTime_;
  uint32_t lastPulseFinishedTime_;
};

#endif
