# Dual-Grating Flexible Synchronization Checklist

## Intended behavior

- [x] Motor 2 / grating 1 is the positive reference coordinate.
- [x] Motor 4 velocity and grating 2 position are converted into that coordinate.
- [x] Both axes use CST torque mode for compliant motion.
- [x] Grating 1 moves toward an absolute post-home target through a rate-limited trajectory.
- [ ] Grating 2 position correction is reserved for a later control stage.
- [x] Initial grating position offset is preserved instead of being corrected immediately.
- [x] Grating 2 position and velocity are telemetry only and do not affect either motor command.
- [x] Motor 2 actual torque from 6077h is negated and sent to motor 4.
- [x] Motor 4 never feeds position or torque information back into motor 2.

## Coordinate and unit checks

- [x] Grating display position remains zero at the homing release edge.
- [x] Every zeroed position starts from `raw encoder value - homing offset`.
- [x] Grating 1 common position is `-1 * (raw1 - offset1)`.
- [x] Grating 2 common position is `-1 * (raw2 - offset2)`.
- [x] TXT `grating1_pos` and `grating2_pos` use those same common positions.
- [x] Axis 2 velocity and torque signs are inverted into the common coordinate.
- [x] Axis 4 velocity and torque signs remain unchanged in the common coordinate.
- [x] A positive grating 1 target produces a negative axis 2 drive command.
- [x] A positive grating 2 following error produces a positive axis 4 drive command.
- [x] Grating 1 CSP debug converts positive common displacement to negative axis 2 displacement.
- [x] Grating 2 CSP debug converts positive common displacement to positive axis 4 displacement.
- [x] Homing leaves the switch through negative axis 2 velocity and positive axis 4 velocity.
- [x] Position gains use torque-permille per grating pulse.
- [x] Damping gains use torque-permille per pulse/second.
- [x] Torque commands are clamped before being sent to 6071h.
- [x] Running torque commands change by at most 4 permille per 5 ms cycle.
- [x] Stop and grating-read failure bypass the slew limiter and clear torque immediately.

## Timing and data-path checks

- [x] The synchronization loop remains at 5 ms / 200 Hz.
- [x] Loop scheduling uses `steady_clock` and absolute deadlines.
- [x] The force-feedback thread owns controller-card calls while synchronization is active.
- [x] The 3 ms UI/recording sampler consumes the control snapshot instead of issuing competing card calls.
- [x] Both gratings are read by one contiguous encoder call in the real-time loop.
- [x] Motor 2 actual torque and motor 4 mirrored command are recorded during this control stage.
- [x] Damping velocity is derived from grating position differences in grating pulse/second.
- [x] UI telemetry is lock-free and does not block the control loop.
- [x] The master integrator is disabled during trajectory motion and only removes static error after stopping.
- [x] The master integral is cleared when the position error changes sign.
- [x] The conservative master-tuning velocity is 100,000 grating pulse/s.
- [x] The UI exposes per-call maximum timing in the force-loop timing tooltip.
- [ ] Confirm the per-cycle 6077h actual-torque read does not break the 5 ms deadline.
- [ ] On hardware, confirm the displayed maximum loop execution time stays below 5000 us.
- [ ] Hover over the force-loop timing value and record all four breakdown values.
- [ ] If the dual 6071 write remains above 2000 us, export the PDO mapping before implementing a raw-PDO write path.
- [ ] On hardware, confirm the loop does not accumulate missed deadlines.

## UI checks

- [x] The lower-left panel describes dual-grating flexible synchronization.
- [x] Master Kp/Kd remain editable; follower position Kp/Kd are visibly reserved and disabled.
- [x] Planned velocity and torque limit are editable.
- [x] The panel displays positions, displacement difference, motor 2 target/actual torque, and motor 4 mirrored torque.
- [x] Starting while already running updates the target and tuning without restarting.
- [x] Stop and total-disable controls still stop synchronization.

## Interaction checks

- [x] Homing retains the original MC_AxisOn/6040 enable sequence proven on hardware.
- [x] Force-feedback startup never calls MC_AxisOn, EnableAxisCiA402, SwitchAxisMode, or writes 6040h.
- [x] Force-feedback startup requires both homing operations first and reuses their enabled CSV state.
- [x] Homing completion only writes zero CSV velocity; it does not disable either motor or clear card state.
- [x] PA25 write/readback warnings do not abort startup; both motors follow the same permissive path.
- [x] Axis 4 no longer copies a possibly stale PA25 value from axis 2.
- [x] PA25=3 torque mode configures and verifies axis 4 object 6080h above zero.
- [x] Axis 4 6072h/6080h copy failures are diagnostic only and do not cancel startup or disable both motors.
- [x] Axis 4 CST startup logs 6061, 6041 operation enabled, PA25 and 6080h without adding a second enable gate.
- [x] The first non-zero axis 4 command logs 6071, 6077 and 6041 Bit12.
- [x] Starting synchronization stops the independent grating debug loops.
- [x] Starting position, velocity, or manual torque mode stops synchronization first.
- [x] Starting either homing operation stops synchronization first.
- [x] Closing the card joins synchronization, homing, sampling, and grating-loop workers.
- [x] Normal stop performs one dual-axis zero-torque update and leaves both axes enabled.
- [x] Only explicit axis-disable and card-close operations disable the drives.
- [x] Grating read failure clears both torque commands for that cycle.
- [x] Master arrival bypasses the slew limiter and writes zero target torque to axes 2 and 4 in the same update.
- [x] The arrival-cleared log is emitted only after both 6071h writes and the combined update succeed.
- [x] Arrival zero torque is latched until the master target changes; the axes remain enabled in CST.
- [x] Card opening waits for `MC_ECatGetInitStep` station 0 and a valid axis-1 6502h read before configuring the slaves.

## First hardware run

- [ ] Click Open Card once and confirm the log shows EtherCAT initialization complete followed by four ready axes.
- [ ] Home both gratings and confirm both displayed values are zero.
- [ ] Without restarting the device, start synchronization and confirm motors 2 and 4 both respond.
- [ ] Jog both tables in the intended common direction and confirm both displayed grating values increase.
- [ ] Confirm raw motor velocity is negative on axis 2 and positive on axis 4.
- [ ] Start with the default 120 permille torque limit and a small target.
- [ ] Confirm motor 2 moves toward the target without a step impact.
- [ ] Confirm motor 4 target equals the sign-inverted motor 2 measured torque plus the relative-position PD correction.
- [ ] Confirm positive `q1-q2` error increases axis 4 forward torque and negative error produces braking/reverse correction.
- [ ] Confirm new TXT files store actual torque in both `motor2_torque` and `motor4_torque`.
- [ ] Confirm new TXT files append `motor2_target_torque` and `motor4_target_torque`.
- [ ] At master arrival, confirm both target-torque displays become zero and stay zero.
- [ ] Increase gains and torque limit only after the sign checks pass.

## Software validation completed

- [x] Visual Studio Release x64 build succeeds.
- [x] Visual Studio Debug x64 build succeeds.
- [x] Qt UIC accepts the updated UI file.
- [x] `git diff --check` reports no whitespace errors.
- [x] Background worker logging is marshalled onto the Qt UI thread.
- [x] Single-axis disable and total disable stop active synchronization before disabling.
- [x] Closing the card or window flushes and closes active data recording.
- [x] Synchronization error uses displacement after synchronization starts, so initial homing offsets are not chased.
- [x] Axis 4 correction does not feed back into the axis 2 master controller.
- [x] Axis 4 torque command has a 4-per-cycle slew limit, while arrival still clears both targets immediately.
- [x] Axis 4 actual 6077h torque is recorded separately from its 6071h target.
- [x] Master integral is active only after trajectory completion and decays while the grating is moving.
- [x] Unchanged 6071h targets are not rewritten every 5 ms, preventing stale torque commands from accumulating.
- [x] Changed axis 2 and axis 4 torque targets use separate single-axis `MC_Update` calls.
- [x] Master arrival uses a realizable 500-pulse/2000-pulse-per-second settle window and still latches both targets at zero.
