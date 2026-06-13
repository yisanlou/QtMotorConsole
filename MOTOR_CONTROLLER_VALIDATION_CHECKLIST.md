# Motor Controller Validation Checklist

## Build Validation

- [x] Debug x64 builds successfully with MSBuild.
- [x] Release x64 builds successfully with MSBuild using temporary `x64\ReleaseCheck\` output because `x64\Release\QtMotorConsole.exe` was locked by a running process.
- [ ] Visual Studio loads the project with all source files under `src/` and all headers under `include/`.

Known build note:

- `include/MultiCardCPP.h` still emits MSVC `C4828` encoding warnings under `/utf-8`. This existed in the vendor SDK header path and was not changed by this refactor.

## Startup And Shutdown

- [ ] Launch the application.
- [ ] Confirm the log text browser receives startup/runtime messages.
- [ ] Click open card and confirm `MC_Open`, `MC_ECatInit`, slave count detection, PDO load, and servo config complete.
- [ ] Confirm active axis count is detected correctly.
- [ ] Click close card and confirm force feedback/grating-zero workers stop, axes are disabled, cache is reset, and card closes.
- [ ] Re-open after close and confirm the card can initialize again.

## Sampling And Telemetry

- [ ] Confirm position values update for axes 1-4.
- [ ] Confirm velocity values update for axes 1-4.
- [ ] Confirm torque values update for axes 1-4.
- [ ] Confirm grating 1 value updates.
- [ ] Confirm grating 2 value updates or auto-probes the correct encoder index.
- [ ] Confirm the UI refresh loop remains responsive during sampling.
- [ ] Confirm force-feedback max execution time telemetry updates and resets after consumption.

## Axis Motion Commands

- [ ] Position mode: select each available axis, command a safe target position and speed, and confirm motion/log result.
- [ ] Velocity mode: select each available axis, command a safe velocity, and confirm motion/log result.
- [ ] Torque mode: select each available axis, command a safe torque, and confirm output/log result.
- [ ] Disable selected axes individually and confirm motion stops.
- [ ] Disable all axes and confirm all axes stop safely.

## MIT Force Feedback

- [ ] Enter a safe grating 1 target position in pulse units and start force feedback.
- [ ] Confirm grating 2 force-feedback target input is hidden and ignored.
- [ ] Confirm only axes 2 and 4 are prepared/enabled by force feedback.
- [ ] Confirm axis 2 switches to torque mode and receives torque from the grating 1 position loop.
- [ ] Confirm axis 4 switches to the configured MIT/follower mode.
- [ ] Confirm the control loop runs at 5 ms and the performance panel reports execution time below 5000 us.
- [ ] Confirm grating 1 target error computes the axis 2 torque command.
- [ ] Confirm force feedback reads grating 1 position, axis 2 actual velocity, and axis 2 actual torque every cycle.
- [ ] Confirm axis 4 receives grating 1 position, axis 2 actual velocity, axis 2 actual torque, Kp, and Kd every cycle.
- [ ] Confirm axes 1 and 3 remain uninvolved.
- [ ] Stop force feedback and confirm axes 2 and 4 receive zero/clear commands and are disabled.
- [ ] Confirm starting another motion mode stops force feedback first.
- [ ] Confirm starting force feedback while grating zero is running is rejected.

## Grating Closed-Loop Debug

- [ ] Enter a safe grating 1 debug target and start grating 1 closed-loop debug.
- [ ] Confirm only motor 2 is enabled and controlled.
- [ ] Enter a safe grating 2 debug target and start grating 2 closed-loop debug.
- [ ] Confirm only motor 4 is enabled and controlled.
- [ ] Confirm Kp, Ki, integral limit, friction feed-forward, and torque limit come from the right-side grating debug panel.
- [ ] Confirm grating 1 and grating 2 debug loops can be stopped independently.
- [ ] Confirm starting force feedback stops all grating debug loops first.
- [ ] Confirm starting position/velocity/torque mode stops all grating debug loops first.
- [ ] Confirm starting grating zero stops all grating debug loops first.

## Grating Zero

- [ ] Click grating 1 zero and confirm axis 2 enters velocity mode.
- [ ] Confirm grating 1 zero searches at the fast speed until main-module IO1 changes level.
- [ ] Confirm grating 1 zero reverses at the slow speed until IO1 releases, then displayed grating 1 becomes zero.
- [ ] Click grating 2 zero and confirm axis 4 enters velocity mode.
- [ ] Confirm grating 2 zero searches at the fast speed until main-module IO2 changes level.
- [ ] Confirm grating 2 zero reverses at the slow speed until IO2 releases, then displayed grating 2 becomes zero.
- [ ] Confirm starting a grating-zero operation while one is running logs a busy message and does not start a second worker.
- [ ] Confirm closing the card during grating zero stops and joins the worker thread.

## Regression Checks

- [ ] Public UI calls still go through `MotorController.h`.
- [ ] No `.cpp` or `.h` files remain in the repository root.
- [ ] All `.cpp` files are listed in `QtMotorConsole.vcxproj`.
- [ ] All `.h` files are listed in `QtMotorConsole.vcxproj`.
- [ ] No behavior changes were intentionally made beyond code organization.
