#include "../../latch_controller.h"

void setup() {
  LatchController latch(500, 3000);
  if (!latch.requestPulse(1000)) __builtin_trap();
  if (!latch.isOn()) __builtin_trap();
  if (latch.update(1499)) __builtin_trap();
  if (!latch.update(1500)) __builtin_trap();
  if (latch.isOn()) __builtin_trap();
  if (latch.requestPulse(4499)) __builtin_trap();
  if (!latch.requestPulse(4500)) __builtin_trap();
  latch.forceOff(4600);
  if (latch.isOn()) __builtin_trap();
}

void loop() {}
