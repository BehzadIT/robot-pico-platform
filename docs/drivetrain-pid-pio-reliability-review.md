# Drivetrain PID / PIO Reliability Review

## Purpose
This document reviews the current robot movement control stack on the Pico and proposes a more reliable and more reactive direction for the drivetrain controller.

This is specifically about:
- dual-track / dual-motor robot movement
- encoder-based drivetrain feedback
- MicroPython runtime behavior on Pico / Pico 2
- RP2040 / RP2350 PIO usage for quadrature encoders
- motion-control PID behavior and lifecycle

This is not a generic PID document for every future control loop on the robot.

## Scope And Steering Policy
The primary steering model assumed in this document is `inner-track slowdown`.

That means:
- straight driving: both tracks target the same speed
- turning: the outer track stays at the requested speed
- the inner track is scaled down toward zero as turn angle approaches `+/-90`

This is the right baseline for the current project because it is easier to stabilize, easier to reason about with a skid-steer chassis, and safer to bring up than pivot turning.

The alternative `pivot / one-track-reverse` mode is still documented later in this review, but it is treated as a future option rather than the primary recommendation.

## Executive Summary
The current drivetrain stack is not unreliable mainly because of the PID gains. The bigger problems are lifecycle, timing, and hardware ownership.

The most important current risks are:
- the control loop is started and stopped as threads, but the thread lifecycle is not synchronized
- encoder PIO state machines are allocated at module import time and again during control-loop startup
- the current RPM measurement blocks for `50 ms` inside each control step, which makes the loop much less reactive than it appears
- steering angle exists in the PID layer but is not wired through the current WebSocket / HTTP movement command path
- PID controller state is reused between runs instead of being reset intentionally
- the local PID helper has correctness defects that can silently produce wrong behavior

The result is a system that can:
- become stale after stop/start cycles
- race its own hardware cleanup
- show sluggish control response because sensing is blocking
- crash or enter inconsistent hardware state when PIO ownership is duplicated
- make tuning look unstable even if the real problem is loop timing and state management

The core recommendation is:
1. Make the drivetrain controller long-lived and state-driven, not create/kill-driven.
2. Make encoder ownership single-owner and persistent.
3. Make velocity estimation non-blocking inside a fixed-rate loop.
4. Reset PID state deliberately on stop, restart, and direction transitions.
5. Only retune after lifecycle and timing are cleaned up.

## Encoder Facts That Should Drive The Design
The encoder details already documented locally are important enough that they should directly shape the runtime architecture instead of being treated as incidental notes.

Documented local facts:
- motor family: Pololu 37D metal gearmotor with integrated encoder
- gear ratio: `30:1`
- encoder type: integrated dual-channel Hall-effect encoder
- encoder output: two `90 degree` phase-shifted square waves, channels `A` and `B`
- encoder resolution: `64 CPR` at the motor shaft using full quadrature decoding
- derived output-shaft resolution: `64 * 30 = 1920 counts/output-shaft revolution`

What those facts mean for software:
- the current code should treat `64` as already fully decoded motor-shaft counts, not multiply by `4` again
- low-speed velocity estimation can be precise enough for closed-loop drivetrain control if counts are sampled regularly and interpreted consistently
- sign conventions matter: direction, encoder count polarity, and RPM sign must all be defined in one place
- the encoder subsystem should expose both raw counts and derived velocity because both are useful for debugging and validation
- the encoder supply requirement still needs bench verification because the local note says `3.5 V - 20 V` with `5 V` recommended, while other project notes have conflicts

Design consequence:
- a single persistent encoder subsystem is the right direction
- the subsystem should own the PIO state machines, count polarity, conversion constants, and health/fault state
- drivetrain code should consume encoder data from that subsystem instead of instantiating encoder objects itself

## Current System Review

## Layer 1: Command Ingress And Navigation State
Current code path:
- Android sends WebSocket drive and stop commands to the Pico.
- Pico route handlers update navigation state through `driverController`.
- The PID loop reads that state and drives the motors.

Relevant code:
- [navigation.py](../server/routes/navigation.py)
- [robot_drive_controller.py](../services/robot_drive_controller.py)
- [NavigationWebSocketApi.kt](../../robot-android-app/app/src/main/java/com/bizi/roboapp/core/transport/NavigationWebSocketApi.kt)

Observed behavior:
- the Android app currently sends signed throttle / turn intent over WebSocket
- the Pico request model currently parses `k`, `t`, `r`, `q`, and `w`
- the Pico transport derives RPM / direction / angle from the intent payload before the worker loop runs
- `RobotNavigationController.accept_drive_request()` updates RPM, direction, and angle together
- the PID loop reads `target_angle`, and live drive traffic now sets it through the derived turn command

Effect:
- the cross-coupled steering logic is present in the PID layer but is mostly dormant in live operation
- today the drivetrain is effectively a straight-line speed controller plus stop handling

Reliability impact:
- medium risk functionally
- high risk for tuning workflow, because steering can look "broken" while the real problem is missing transport plumbing

Reactiveness impact:
- medium
- steering can never become responsive if the input is not actually reaching the controller

## Single Persistent Encoder Subsystem
The preferred design is one persistent subsystem that owns both drive encoders for the full lifetime of the drivetrain firmware.

Required properties of that subsystem:
- initialize both PIO state machines once during drivetrain bring-up
- own all encoder pins, PIO IDs, count polarity, and conversion constants
- expose persistent raw counts, delta counts, motor RPM, and output-shaft RPM
- support explicit `reset_counts()` and explicit `restart()` behavior
- surface health/fault information so higher-level drivetrain code can decide whether to stop, recover, or enter a faulted state
- never be created implicitly at import time
- never be recreated on every drive command or every PID loop start

Recommended interface shape for the future implementation:
- `snapshot()` returning both encoder counts plus timestamp/dt
- `velocity_snapshot()` returning per-track velocity derived from count deltas
- `reset_counts()` for deliberate zeroing
- `health()` or equivalent fault/status query
- `shutdown()` for explicit cleanup at firmware shutdown, not routine motion control

Reliability reason:
- the encoder subsystem is where PIO ownership, polarity, conversion math, and stale state need to be centralized
- if this logic stays spread across the PID loop and test/demo modules, lifecycle bugs will keep reappearing

## Layer 2: Control-Loop Lifetime
Relevant code:
- [robot_cross_coupled_pid.py](../services/robot_cross_coupled_pid.py)

Observed behavior:
- `robot_pid` is a singleton object
- `start()` launches a new `_thread`
- `terminate_thread()` sets a flag and immediately stops motors
- `is_running()` is inferred only from `terminate_flag`
- cleanup happens later inside the worker thread

This creates a race:
- stop can set `terminate_flag = True`
- a new start can happen before the old thread reaches encoder cleanup
- both threads share `self.enc_left`, `self.enc_right`, and motor output objects
- the old thread can deinit encoder objects that were just created by the new thread

Effect:
- stale state
- hardware cleanup races
- intermittent failures after repeated stop/start cycles
- possible "sometimes unstable" behavior that looks random

Reliability impact:
- critical

Reactiveness impact:
- medium
- even if the control law is good, thread churn adds avoidable latency and state hazards

## Exceptions, Faults, And Logging
For this drivetrain stack, fault handling is part of the design, not an afterthought.

The updated implementation direction should assume these classes of failure are possible:
- PIO state-machine allocation failure
- invalid or non-consecutive encoder pin configuration
- duplicate state-machine use
- encoder read failure or stale FIFO/program state
- implausible count jumps or sign inversions
- PID computation fault
- motor output fault
- stop-path fault during emergency stop or cleanup
- recovery/restart fault after a previous failure

Recommended policy:
- any fault that makes encoder data untrustworthy should stop the drivetrain immediately
- any fault that makes PWM output uncertain should stop the drivetrain immediately
- recoverable startup/configuration errors should be logged with enough context to diagnose them remotely
- normal runtime logs should be sampled or rate-limited so the hot path stays usable
- fault logs should never be rate-limited to the point that root-cause context is lost

Minimum log context for encoder/drivetrain faults:
- subsystem tag such as `encoder`, `drivetrain`, or `pid`
- state-machine ID
- pin mapping
- raw count sample or last known count
- `dt` or loop timestamp
- target RPM and measured RPM when relevant
- current controller state such as `starting`, `running`, `stopping`, or `faulted`
- exception type and message

Recommended fault-handling boundaries:
- low-level encoder/PWM modules should catch hardware-facing exceptions, log them with hardware context, and surface a fault result upward
- the drivetrain controller should be the component that decides to stop motors and transition into `faulted`
- transport and route layers should not try to recover hardware faults directly

Documentation rule:
- every fault path should be described in code comments or docstrings where the safety behavior would otherwise be unclear

## Layer 3: PID Controller Logic
Relevant code:
- [robot_cross_coupled_pid.py](../services/robot_cross_coupled_pid.py)
- [PID.py](../services/simple_pid/PID.py)

Observed behavior:
- two independent PID instances are used, one per track
- turning behavior is implemented by reducing the inner-track setpoint
- sign is applied after the PID output, so both tracks share forward or reverse direction
- no intentional pivot-turn mode exists

This part is conceptually reasonable for inner-track slowdown, but there are important implementation risks:
- PID state is not reset on each control run
- PID state is not reset on stop
- PID state is not reset on direction reversal
- the helper PID implementation has bugs:
  - `integral_limits` property returns output limits instead of integral limits
  - `set_auto_mode()` references `self.output_limitsoutput_limits`, which is a typo

Effect:
- integral bias can leak from one motion segment into the next
- stop/start behavior can feel inconsistent
- future tuning changes could expose helper-library bugs more clearly

Reliability impact:
- high

Reactiveness impact:
- medium
- stale integral state can make the robot overshoot or feel sticky after restart

## Layer 4: Measurement And Timing
Relevant code:
- [robot_cross_coupled_pid.py](../services/robot_cross_coupled_pid.py)
- [dual_rpm_pio_test.py](../services/dual_rpm_pio_test.py)

Observed behavior:
- `read_rpms()` captures encoder count, sleeps `0.05 s`, reads again, computes RPM from delta
- the main loop then sleeps another `0.01 s`

So the control loop is not truly a `10 ms` loop. It is roughly:
- `50 ms` blocking measurement window
- plus computation
- plus `10 ms` extra sleep

That means:
- loop cadence is closer to `~60 ms+`
- velocity is measured using a blocking observation window
- the control loop cannot react faster than the measurement window
- command changes can only affect motor output after that blocking read completes

Effect:
- sluggish response
- extra delay in stop-to-output and command-to-output transitions
- harder PID tuning because control lag is built into the measurement method

Reliability impact:
- medium

Reactiveness impact:
- critical

## Layer 5: PIO Encoder Ownership
Relevant code:
- [dual_rpm_pio_test.py](../services/dual_rpm_pio_test.py)
- [robot_cross_coupled_pid.py](../services/robot_cross_coupled_pid.py)

Observed behavior:
- the encoder module creates global `PIOQuadratureCounter` instances at import time
- the runtime control loop creates fresh encoder counters again on the same state-machine IDs
- deinitialization removes the program from the PIO instance

This is dangerous because:
- PIO state machines are scarce hardware resources
- current code has multiple ownership paths for the same resource
- import-time hardware allocation makes program behavior depend on import order
- repeated create/deinit cycles are harder to reason about than a persistent owner model

Effect:
- stale hardware state
- state-machine conflicts
- fragile restart behavior
- crashes that appear "sometimes" after repeated lifecycle changes

Reliability impact:
- critical

Reactiveness impact:
- medium

## Layer 6: PWM Output Handling
Relevant code:
- [robot_cross_coupled_pid.py](../services/robot_cross_coupled_pid.py)

Observed behavior:
- PWM objects are created once, which is good
- PWM frequency is set inside `set_motor_output()` on every output update

This is not ideal because:
- PWM frequency is a configuration concern, not a per-cycle control action
- on RP2040/RP2350, PWM hardware is organized into slices, and frequency changes can affect shared generator behavior
- repeated `freq()` calls in the fast path add avoidable work and potential side effects

Effect:
- unnecessary work in the hot loop
- more moving pieces in the most time-sensitive part of the controller

Reliability impact:
- low to medium

Reactiveness impact:
- medium

## External Research Findings

## General DIY Drivetrain PID Practice
The most consistent guidance across practical PID references is:
- use a regular control interval
- keep sensing and actuation timing deterministic
- limit output and integral state deliberately
- reset or reinitialize controller state during mode transitions
- tune only after timing and sensing behavior are stable

Brett Beauregard's well-known PID series is still a useful practical reference because it focuses on the issues that turn "works pretty well" code into robust control code: sample time, derivative kick, windup, auto/manual transitions, and bumpless startup. That maps directly onto the current project's needs. [Article: Brett Beauregard](https://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/)

The `simple-pid` projects in Python and MicroPython also emphasize:
- regular update intervals
- output limiting
- anti-windup
- explicit controller state management
- ability to observe PID components during tuning

References:
- [Repo: m-lundberg/simple-pid](https://github.com/m-lundberg/simple-pid)
- [Repo: gastmaier/micropython-simple-pid](https://github.com/gastmaier/micropython-simple-pid)

Implication for this project:
- the drivetrain should not be tuned around blocking measurement and unstable lifecycle behavior
- the controller should be made stateful and deterministic first

## MicroPython Runtime Guidance
The official MicroPython `_thread` documentation explicitly says the module is highly experimental and its API is not yet fully settled. That is a strong signal to keep thread usage simple, bounded, and easy to reason about. [Official: `_thread`](https://docs.micropython.org/en/latest/library/_thread.html)

The official MicroPython performance guidance says RAM allocation and garbage collection can cost milliseconds, and that performance-sensitive code should minimize repeated allocation and reuse objects. [Official: speed guide](https://docs.micropython.org/en/latest/reference/speed_python.html)

The ISR guidance also reinforces the same principle:
- keep critical code short
- avoid allocation
- use preallocated buffers
- schedule deferred work when needed

References:
- [Official: speed guide](https://docs.micropython.org/en/latest/reference/speed_python.html)
- [Official: ISR rules](https://docs.micropython.org/en/latest/reference/isr_rules.html)
- [Official: micropython.schedule](https://docs.micropython.org/en/latest/library/micropython.html)

Implication for this project:
- do not rely on frequent thread creation / destruction as a normal control primitive
- avoid import-time hardware side effects
- keep the control loop allocation-free as much as practical
- keep logging volume in the hot loop under control

## RP2040 / RP2350 PIO Guidance
The official MicroPython `rp2.StateMachine` docs are clear that state machines are numbered hardware resources, and that programs are loaded into PIO instruction memory. `StateMachine.restart()` exists specifically to clear internal state, while `active()` starts and stops the machine. [Official: `rp2.StateMachine`](https://docs.micropython.org/en/latest/library/rp2.StateMachine.html)

The official Raspberry Pi quadrature encoder example is especially relevant. It treats the encoder as a persistent absolute counter and, in the main control loop, computes delta from the last sample:
- keep the hardware counting continuously
- read absolute position
- subtract previous sample to get velocity information

That pattern is more robust than rebuilding encoder objects or sleeping inside the read function. [Official example: pico-examples quadrature encoder](https://github.com/raspberrypi/pico-examples/blob/master/pio/quadrature_encoder/quadrature_encoder.c)

The RP2040 datasheet also matters here:
- the PWM block has 8 slices, each slice can drive two outputs
- PWM is based on a free-running counter
- PIO state machines can be started and stopped at any time, but they are explicit hardware resources

References:
- [Official: RP2040 datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)
- [Official: MicroPython `rp2.StateMachine`](https://docs.micropython.org/en/latest/library/rp2.StateMachine.html)
- [Official example: Raspberry Pi quadrature encoder](https://github.com/raspberrypi/pico-examples/blob/master/pio/quadrature_encoder/quadrature_encoder.c)

Implication for this project:
- one owner should create and manage encoder state machines
- those state machines should stay alive across normal driving
- if reinitialization is needed, it should be explicit and centralized
- velocity should be derived from absolute-count deltas in the fixed-rate control loop

## Reference Repo Patterns
Useful patterns seen in reference implementations:
- persistent encoder objects that expose position and velocity
- update methods that take sampling time explicitly
- automatic or centralized state-machine allocation instead of duplicate ownership
- keeping control and measurement cadence aligned

References:
- [Repo: matteomeneghetti/pico-quadrature-encoder](https://github.com/matteomeneghetti/pico-quadrature-encoder)
- [Repo: 1988kramer/motor_control](https://github.com/1988kramer/motor_control)
- [Repo: peterhinch/micropython-samples](https://github.com/peterhinch/micropython-samples)

These are not one-to-one templates for this robot, but they reinforce the same design direction:
- one owner per hardware resource
- fixed-rate update loop
- separate sensing lifetime from control-command lifetime

## Platform Maturity Notes
The current robot is on MicroPython and Pico-class hardware where some advanced or edge-case behavior is still evolving.

Examples from upstream:
- `_thread` remains officially experimental
- RP2350 / Pico 2 PIO compatibility issues have been reported in upstream MicroPython

References:
- [Official: `_thread`](https://docs.micropython.org/en/latest/library/_thread.html)
- [Issue: RP2350 PIO scripts fail on Pico 2](https://github.com/micropython/micropython/issues/17047)
- [Issue: Pico 2 W thread interaction problem](https://github.com/micropython/micropython/issues/16779)

This does not mean the platform is unusable. It means the project should prefer simpler ownership and lifecycle patterns instead of relying on behavior that is hard to debug when multiple subsystems are active.

## Current System vs Best Practice

| Area | Current system | Better practice | Main risk |
|---|---|---|---|
| Controller lifetime | Start/stop thread per motion session | One long-lived control task or tightly controlled single worker | thread race and stale cleanup |
| Encoder ownership | import-time globals plus runtime recreation | single persistent encoder subsystem | duplicated PIO ownership |
| Velocity estimation | blocking `sleep(0.05)` inside read | delta from persistent absolute counts each fixed tick | sluggish response |
| PID lifecycle | state carries across runs | explicit reset on stop/start/direction change | biased restart behavior |
| Steering input | angle exists only inside PID layer | wire angle through transport and nav state | dormant steering logic |
| PWM fast path | set frequency during output update | set frequency once during init | avoidable hot-path overhead |
| Error handling | fragmented lifecycle and cleanup | explicit state machine for running/stopping/faulted | stale state after faults |

## Why Inner-Track Slowdown Is The Right Baseline
For this project, inner-track slowdown is the better default because:
- it is simpler to stabilize than pivot turning
- it keeps both tracks in the same motion direction
- it fits a cautious bring-up strategy
- it reduces the number of abrupt sign changes in the drivetrain
- it is easier to reason about when encoder polarity and wiring still need verification

For `angle = +/-90`, inner-track slowdown means:
- outer track remains at commanded speed
- inner track trends toward zero
- the robot performs its tightest skid turn without reversing one side

This is a good reliability-first choice.

## Alternative: Pivot / One-Track-Reverse Steering
Pivot turning is still a valid alternative, but it should be treated as a different mode, not a trivial extension.

Pivot mode would require:
- setpoints that can go negative for one side while the other remains positive
- more careful handling of low-speed sign changes
- explicit control of transitions between skid-turn and pivot-turn behavior
- stronger validation of motor direction polarity and encoder sign correctness
- likely retuning because plant behavior changes significantly near zero and during reversals

Advantages:
- tighter turns
- better in-place rotation capability
- potentially better heading correction at very low linear speed

Disadvantages:
- higher implementation complexity
- more abrupt drivetrain dynamics
- greater chance of oscillation or jerk during sign reversals
- more difficult bring-up on a system that is already showing stale-state and lifecycle problems

Recommendation:
- keep inner-track slowdown as the main path
- document pivot as a future extension once the base control stack is stable

## Recommended Solution Direction

## Suggested MicroPython Module Layout
If this review is converted into implementation work, the current module split should be cleaned up. The current production logic is mixed with test/demo behavior in a way that makes ownership unclear.

Runtime choice for the first stabilization phase:
- prefer one persistent worker thread over per-command thread creation
- keep timer-driven control documented as a future alternative rather than the first migration step

Why timer-driven control is not the phase-1 default:
- MicroPython timer callbacks run under tighter constraints than normal thread code
- the current drivetrain code still needs cleaner fault boundaries and ownership before moving to a more constrained scheduling model
- a persistent worker thread is the lower-risk step for removing stale state first

Recommended structure:
- `services/encoder_pio.py`
  - low-level PIO quadrature counter implementation
  - pin validation
  - state-machine setup, restart, and deinit behavior
  - no demo/test motor-driving code
- `services/encoder_subsystem.py`
  - persistent owner for both encoders
  - count snapshots, velocity estimation, reset behavior, and fault/health reporting
  - the single place where encoder CPR, gear ratio, and sign conventions are applied
- `services/drivetrain_controller.py`
  - long-lived drivetrain state machine
  - fixed-rate loop
  - stop/fault handling
  - interaction with PID and encoder subsystem
- `services/pid_controller.py` or equivalent wrapper
  - PID setup
  - PID reset policy
  - access to PID components for diagnostics
- `services/encoder_pio_test.py` or similar
  - only test/demo utilities
  - no production ownership of encoder hardware

Recommended rename:
- `dual_rpm_pio_test.py` should not remain the production-facing runtime module name because it currently mixes reusable encoder logic with test/demo behavior
- the review should explicitly recommend splitting it and renaming the production portion to something lifecycle-oriented such as `encoder_pio.py` or `encoder_subsystem.py`

Reliability reason:
- naming and file boundaries affect design quality here because unclear ownership is one of the direct causes of stale hardware state

## Code Documentation Requirements
Code documentation is important in this part of the system because the failure modes are lifecycle and hardware-ownership problems, not just algorithm mistakes.

Minimum documentation expectations for the future implementation:
- module docstring for the persistent encoder subsystem explaining ownership and lifetime
- docstring for any method that starts, stops, resets, or recovers hardware
- comment where encoder sign conventions are defined
- comment where RPM conversion constants are applied
- comment where PIO state-machine IDs and pin assumptions are bound
- comment or docstring explaining the stop/fault state machine
- comment where logging is intentionally rate-limited in the control path

The goal is not to comment every line. The goal is to document:
- ownership
- safety behavior
- timing assumptions
- recovery behavior
- hardware assumptions

## Phase 1: Remove Stale State And Crash Paths
Goal:
- make repeated start/stop safe
- stop duplicated hardware ownership

Recommended changes:
- replace repeated control-thread creation with one long-lived controller owner
- if a worker thread is kept, ensure it is created once and transitions through explicit states such as `idle`, `running`, `stopping`, and `faulted`
- move all encoder PIO state-machine allocation out of import-time globals
- give one subsystem sole ownership of encoder state machines
- do not recreate encoder state machines on every drive command
- perform explicit reset / restart only inside that owner when needed
- ensure encoder-subsystem initialization, shutdown, and recovery paths all log meaningful exceptions
- ensure startup faults leave the drivetrain in a safe non-driving state

Expected result:
- repeated stop/start no longer races cleanup
- fewer "sometimes unstable" failures

## Phase 2: Make Measurement And Control Timing Deterministic
Goal:
- improve responsiveness without sacrificing reliability

Recommended changes:
- run the control loop at a fixed period such as `50-100 Hz`
- on each tick, read current absolute encoder counts and compute delta from the previous sample
- compute velocity from `delta_counts / dt`
- remove blocking `sleep()` calls from encoder read helpers used by the control loop
- keep the hot path free of avoidable allocation
- initialize PWM frequency once, not on each output update
- keep encoder conversion logic in the persistent subsystem instead of re-deriving it ad hoc in the PID loop

Expected result:
- faster command response
- more stable tuning conditions
- less built-in loop delay

## Phase 3: Fix PID State Management
Goal:
- make controller behavior repeatable between movement segments

Recommended changes:
- reset PID internal state on:
  - controller startup
  - explicit stop
  - sign reversal
  - fault recovery
- either repair the local PID helper or replace it with a well-tested equivalent
- make integral and output clamping behavior explicit and correct
- expose P / I / D components for tuning diagnostics at a limited rate
- log PID exceptions with target state and latest measurement context, then force a safe stop

Expected result:
- less sticky or biased behavior after stop/start
- safer and more explainable tuning

## Phase 4: Complete Steering Plumbing
Goal:
- make the chosen steering policy actually reachable in live operation

Recommended changes:
- add angle to the live movement protocol
- update request parsing and controller state handoff
- keep angle clamped to the intended steering domain
- preserve current stale-command stop semantics
- document the chosen inner-track slowdown policy directly in the command-to-control path code

Expected result:
- the inner-track slowdown controller can actually steer the robot

## Phase 5: Retune Only After The Above
Goal:
- avoid tuning around architectural defects

Recommended changes:
- retune PID gains only after:
  - loop rate is fixed
  - encoder ownership is stable
  - PID reset behavior is correct
  - steering input is wired
- test straight drive first
- test low-speed start/stop second
- test moderate turning third
- test aggressive turning only after base behavior is repeatable

Expected result:
- gains that reflect the real plant, not the current software timing artifacts

## Bench Validation Checklist
Before trusting any tuning changes, verify:
- exact encoder pins on the as-built robot
- exact encoder supply voltage on the as-built robot
- encoder sign for forward and reverse on both tracks
- commanded stop latency
- repeated start/stop for at least tens of cycles
- repeated WebSocket disconnect/reconnect cycles
- repeated direction reversals
- straight-line drift under constant RPM
- turn response for small, medium, and large angles
- long-run soak without stale state or control-thread wedging

Useful quantitative checks:
- actual control-loop period
- loop jitter
- count delta stability at low speed
- count noise at zero speed
- time from command reception to PWM update
- time from stop command to zero PWM

## Recommended Implementation Priorities
If this review is converted into code work, the order should be:
1. encoder / PIO single ownership with exception-safe startup and recovery
2. split and rename the current encoder/test module into production and test-oriented modules
3. control-loop lifetime redesign
4. non-blocking count-delta velocity estimation
5. PID helper repair or replacement
6. PID reset policy
7. steering transport plumbing
8. retuning

This order is important. Reversing it will produce misleading tuning results and waste time.

## Sources

## Current Project Code And Docs
- [Current architecture](../../docs/overview/current-architecture.md)
- [Pico firmware architecture](../../docs/firmware/pico-firmware-architecture.md)
- [Motor control and encoders](../../docs/firmware/motor-control-and-encoders.md)
- [Known issues and open questions](../../docs/verification/known-issues-and-open-questions.md)
- [Pololu_37D_Encoder_Spec_and_RPM_Test_Code (1).txt](../docs/Pololu_37D_Encoder_Spec_and_RPM_Test_Code%20%281%29.txt)
- [navigation.py](../server/routes/navigation.py)
- [robot_drive_controller.py](../services/robot_drive_controller.py)
- [robot_cross_coupled_pid.py](../services/robot_cross_coupled_pid.py)
- [PID.py](../services/simple_pid/PID.py)
- [dual_rpm_pio_test.py](../services/dual_rpm_pio_test.py)
- [NavigationWebSocketApi.kt](../../robot-android-app/app/src/main/java/com/bizi/roboapp/core/transport/NavigationWebSocketApi.kt)

## Official Platform Sources
- [MicroPython `_thread` docs](https://docs.micropython.org/en/latest/library/_thread.html)
- [MicroPython `machine.PWM` docs](https://docs.micropython.org/en/latest/library/machine.PWM.html)
- [MicroPython `rp2.StateMachine` docs](https://docs.micropython.org/en/latest/library/rp2.StateMachine.html)
- [MicroPython ISR rules](https://docs.micropython.org/en/latest/reference/isr_rules.html)
- [MicroPython speed/performance guide](https://docs.micropython.org/en/latest/reference/speed_python.html)
- [MicroPython `micropython.schedule` docs](https://docs.micropython.org/en/latest/library/micropython.html)
- [RP2040 datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)
- [Raspberry Pi Pico quadrature encoder example](https://github.com/raspberrypi/pico-examples/blob/master/pio/quadrature_encoder/quadrature_encoder.c)

## Upstream Issues
- [MicroPython issue: `_thread` remains experimental by documentation context](https://docs.micropython.org/en/latest/library/_thread.html)
- [MicroPython issue: RP2350 PIO scripts fail on Pico 2](https://github.com/micropython/micropython/issues/17047)
- [MicroPython issue: Pico 2 W thread interaction problem](https://github.com/micropython/micropython/issues/16779)

## Reference Repos And Articles
- [Brett Beauregard: Improving the Beginner's PID](https://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/)
- [m-lundberg/simple-pid](https://github.com/m-lundberg/simple-pid)
- [gastmaier/micropython-simple-pid](https://github.com/gastmaier/micropython-simple-pid)
- [matteomeneghetti/pico-quadrature-encoder](https://github.com/matteomeneghetti/pico-quadrature-encoder)
- [1988kramer/motor_control](https://github.com/1988kramer/motor_control)
- [peterhinch/micropython-samples](https://github.com/peterhinch/micropython-samples)
