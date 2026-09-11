# The Tiger Challenge

Melbourne Zoo Sumatran tiger enrichment capstone — design context as of 11 September 2026.

## Purpose and concept

The project aims to develop a robotic enrichment system for Hutan, a Sumatran tiger at Melbourne Zoo. A mobile spherical robot is intended to encourage investigation, stalking, movement and play, then guide the tiger towards a separate feeder box that provides a food reward. Animal engagement and welfare benefits are intended outcomes, not yet demonstrated results.

The current concept uses two motor-driven internal wheels acting against the inside of a spherical shell. Differential wheel motion provides propulsion and steering. The ball carries its control electronics, sensors and battery; the food-release mechanism remains in the stationary feeder box. A speaker and amplifier are also planned for sound cues.

## Prototype and demonstration goals

A small, approximately 20 cm diameter ball is being used for early development. A larger ball is planned for the Endeavour exhibition; approximately 50 cm has been discussed, but final dimensions and hardware remain subject to mechanical development.

The intended demonstration is a robot moving within a defined space and triggering a simplified feeder box when it reaches the designated feeder zone. The box releases food or a substitute reward. This is a supervised prototype demonstration with no live tiger interaction, rather than evidence of readiness for enclosure deployment.

## System architecture

```text
Operator GUI / controller <-- wireless --> Raspberry Pi 3
                                              |
                                    UART commands / status
                                              |
                                            Teensy <-- motor encoders
                                              |
                                      Sabertooth 2x12
                                              |
                                    Two DC motors / wheels
                                              |
                                         Spherical shell

Fixed UWB anchors <--> onboard UWB tag --> localisation input to Pi
IMU / battery sensing -----------------> control and monitoring
Pi arrival logic ----------------------> separate feeder-box mechanism
Pi audio ------------------------------> amplifier --> speaker
```

| Component | Intended responsibility |
|---|---|
| Operator interface | Manual forward/reverse/turn/stop and variable speed; emergency stop; display mode, position, battery and faults. Waypoint selection and mode controls support later autonomy. |
| Raspberry Pi 3 | High-level state machine, operator communication, localisation, geofencing, navigation, feeder coordination and logging. Python is the proposed application language. |
| Teensy | Predictable motor-control timing, encoder acquisition, left/right speed targets, closed-loop speed regulation, command-timeout stopping and motor fault handling. Firmware is expected to use C/C++. |
| Sabertooth 2x12 | Converts controller commands into motor power. The exact input mode and signalling must be confirmed before wiring and implementation; earlier notes use “PWM/direction” as shorthand. |
| Power system | Planned rechargeable battery with protected motor supply and regulated electronics supplies. Current measurements will inform battery and regulator sizing. |

The proposed software uses **timed control loops plus asynchronous sensor updates**. The Pi runs a supervisory loop; callbacks or background tasks update timestamped sensor and command data. The Teensy runs a fixed-rate motor loop, with interrupts for encoder pulses. Illustrative starting rates are 10–30 Hz for supervision and 50–100 Hz for motor control, to be validated experimentally.

Only the Pi's supervisory logic selects the final requested motion; the Teensy applies motor control and local safety overrides. Sensor callbacks do not independently command motion. Suggested states are idle, manual control, autonomous navigation, geofence avoidance, fault and emergency stop.

## Sensors and interfaces

- **Ultra-wideband (UWB):** fixed anchors and one mobile tag provide position for navigation, exclusion zones and feeder proximity. Qorvo modules/development kits have been considered; hardware, anchor layout and the Pi interface still need confirmation. Accuracy must be measured with the shell, obstructions and intended environment.
- **Inertial measurement unit (IMU):** the BNO085 breakout has been discussed for orientation, angular rate, rocking measurement and possible interaction detection. Its controller connection and role in heading estimation remain to be finalised.
- **Motor encoders:** provide left/right speed feedback to the Teensy. Wheel rotation alone does not establish ball displacement because the wheels can slip against the shell.
- **Battery voltage sensing:** supports telemetry and low-voltage stopping. Motor-current sensing is a possible addition for power characterisation and fault detection.
- **Optional obstacle sensors:** time-of-flight, lidar and ultrasonic options have been explored. Mounting and sensing through a rotating shell remain unresolved.

The planned Pi–Teensy link is UART, carrying speed/turn/stop commands and returning encoder speeds, status and faults. Operator communication is expected to use Wi-Fi; alternatives remain under evaluation. Sensor fusion, including a possible Kalman filter, is an enhancement rather than an established implementation. UWB position alone does not supply heading.

## Feeder-box interaction

The longer-term sequence is: attract engagement → detect distinct tiger interactions → navigate to the selected feeder → stop in its arrival zone → request food release. One feeder is planned for prototyping, with multiple feeders a possible extension.

Draft requirements propose **three interactions separated by 30 seconds** and a **0.5 m feeder-arrival threshold**. These are provisional values to validate; older notes also mention five interactions. Detection must distinguish animal contact from driving, terrain disturbances and the robot's own rocking.

For the exhibition, reaching the feeder zone is the essential trigger; reliable tiger-interaction recognition is not a prerequisite. The feeder communication link, actuator, release confirmation and protection against repeated releases still require detailed design.

## Safety approach

The intended behaviour is stopped on startup/reset, emergency-stop priority over all movement commands, and a Teensy watchdog that stops the motors if fresh Pi commands cease. Operator-link loss must also propagate to a stop. Invalid or stale localisation prevents autonomous movement.

The Pi applies speed/acceleration limits and exclusion zones for the pond, enclosure boundary and high-risk terrain. Margins must account for localisation uncertainty and stopping distance. Battery monitoring and stall/fault detection support safe stopping; encoder-only stall detection cannot detect every slipping or trapped condition.

Emergency return is a separate proposed navigation feature and must never override an emergency stop. Mechanical containment, electrical protection and operating limits require validation before any animal trial. Exhibition operation is low-speed, supervised and confined to a controlled area, with no public handling.

## MVP and stretch scope

| Priority | Scope |
|---|---|
| Core MVP | Reliable manual driving, motor control, encoder feedback, UWB position input, basic geofencing, emergency stop, communication-loss stop and useful data logging. |
| Exhibition integration | Demonstrate movement in a defined area and repeatable arrival-triggered release from one feeder box. |
| Next-stage goals | Closed-loop performance tuning, basic waypoint navigation, safe manual/autonomous mode transitions and validated IMU-based interaction detection. |
| Stretch goals | Dynamic obstacle avoidance/path planning, richer sensor fusion, multiple feeders, camera display, emergency return and more elaborate enrichment behaviour. |

Earlier notes classify some features inconsistently, particularly mode switching and stall detection. This table expresses implementation priorities; it does not remove the need to address faults relevant to the chosen demonstration.

## Testing approach

1. **Bench tests:** verify motor direction and stopping, encoder sign/scaling, command-to-speed response and current draw. Start with open-loop characterisation before tuning feedback control.
2. **Small-ball tests:** measure traction, slip, steering and forward/backward rocking under load. Log commands, encoder speeds, IMU data and supply conditions; use these to identify the relevant dynamics and evaluate speed control and rate damping.
3. **Module and fault tests:** exercise localisation validity, geofence decisions, startup/reset, emergency stop, dropped commands, sensor loss and battery/fault handling.
4. **Integrated demonstration:** repeat travel and feeder arrival/release trials, including noisy boundary readings and repeated zone entry. Record arrival error, stopping behaviour and unintended releases.
5. **Larger prototype:** repeat identification, power measurements and controller tuning. The development method may transfer from the small ball; numerical models and gains must be revalidated.

Log timestamps, mode/state, targets, motor commands, measured speeds, UWB/IMU data, battery voltage and faults. Set measurable acceptance limits before final testing; current notes do not establish validated performance figures.

## Current development status

The project is in detailed design and early hardware/control development. A small prototype exists and noticeable forward/backward swaying has been reported. The latest discussions concern Raspberry Pi setup, motor-driver testing, current measurement, low-level controller design and integrating sensors through the two-controller architecture. Michael is leading control/software work.

There is no confirmed end-to-end demonstration, validated safety performance or completed autonomous system in the available context. Immediate work is to establish repeatable motor/encoder tests, validate power requirements, tune motion on the small ball, and integrate localisation, stopping behaviour and the feeder link in stages.

This workspace currently contains reference material and this README, not a runnable software release. Files under `sources/` are read-only synced references. This summary follows the project conversation, prioritising recent decisions and marking unresolved details rather than treating earlier suggestions as completed work.
