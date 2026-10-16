#include <cassert>
#include <cstdint>
#include <iostream>

#include "../latch_controller.h"

int main() {
  LatchController latch(500, 3000);

  // First valid command starts one 500 ms pulse.
  assert(latch.requestPulse(1000));
  assert(latch.isOn());
  assert(!latch.update(1499));
  assert(latch.isOn());

  // A repeated command cannot extend an active pulse.
  assert(!latch.requestPulse(1200));
  assert(latch.update(1500));
  assert(!latch.isOn());

  // Cooldown starts when the relay turns off.
  assert(!latch.requestPulse(4499));
  assert(latch.requestPulse(4500));
  assert(latch.isOn());

  // A failure/disconnect request immediately returns to OFF.
  latch.forceOff(4600);
  assert(!latch.isOn());

  // Deadline comparison remains correct through millis() rollover.
  LatchController rolloverLatch(500, 3000);
  assert(rolloverLatch.requestPulse(UINT32_MAX - 249));
  assert(!rolloverLatch.update(249));
  assert(rolloverLatch.update(250));
  assert(!rolloverLatch.isOn());

  std::cout << "PASS: latch controller timeout, cooldown, force-off and rollover\n";
  return 0;
}
