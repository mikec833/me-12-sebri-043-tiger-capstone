#!/usr/bin/env bash
# Host-side simulation of the v4.7 command arbitration (bench matrix F2 to F13
# plus extra cases). Compiles the real sketch against the mocks in mock/ and
# steps loop() on a simulated 1 ms clock. Needs only a C++17 compiler.
#   xiao/firmware_sim/run.sh
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
c++ -std=c++17 -Wall -Wno-unused-function -Imock -x c++ sim.cpp -o /tmp/tigerball_fw_sim
/tmp/tigerball_fw_sim
