# Grating Closed-Loop Debug Checklist

## Intended Boundary

- This feature is separate from MIT force feedback.
- Grating 1 debug loop controls motor 2 only.
- Grating 2 debug loop controls motor 4 only.
- Each grating loop can be started and stopped independently.
- Starting normal force feedback stops all grating debug loops first.

## Code Self-Review

- [x] Debug loop code lives in `src/GratingClosedLoopController.cpp`.
- [x] Force feedback code lives separately in `src/ForceFeedbackController.cpp`.
- [x] Debug loop has its own thread state: `g_gratingClosedLoopRunning[]` and `g_gratingClosedLoopThread[]`.
- [x] Debug loop does not call `StartForceFeedback` or use `g_bFollowRunning` as its running flag.
- [x] Grating 1 maps to motor 2.
- [x] Grating 2 maps to motor 4.
- [x] Grating 1 reads encoder channel 1 with `g_gratingOffset1`.
- [x] Grating 2 reads `g_grating2EncoderIndex` with `g_gratingOffset2`.
- [x] Debug loop prepares its motor in CST torque mode.
- [x] Debug loop writes torque through 6071h.
- [x] Kp, Ki, integral limit, friction feed-forward, and torque limit are configured from the UI.
- [x] Starting position/velocity/torque mode stops all grating debug loops.
- [x] Starting grating zero stops all grating debug loops.
- [x] Starting force feedback stops all grating debug loops.
- [x] Closing the card stops and joins all grating debug loops.
- [x] Total disable stops all grating debug loops before disabling axes.
- [x] `src/GratingClosedLoopController.cpp` is included in the Visual Studio project and filters.

## Hardware Checks

- [ ] Start grating 1 debug loop with a small target and confirm only motor 2 is enabled.
- [ ] Start grating 2 debug loop with a small target and confirm only motor 4 is enabled.
- [ ] Confirm stopping grating 1 does not stop grating 2 if both are running.
- [ ] Confirm stopping grating 2 does not stop grating 1 if both are running.
- [ ] Confirm torque clears to zero when each debug loop stops.
- [ ] Confirm force feedback still starts through its original button and stops debug loops first.
- [ ] Confirm grating zero still starts through its original buttons and stops debug loops first.
- [ ] Confirm UI force-feedback targets are not used by the grating debug buttons.
- [ ] Confirm debug-loop execution is stable at 5 ms with the selected parameters.

## Known Tuning Risks

- [ ] Kp/Ki/friction/limit values are placeholders until real machine tuning.
- [ ] Positive torque direction must be checked for both grating axes.
- [ ] Grating 2 encoder channel auto-detection must be confirmed before closed-loop tests.
- [ ] Torque limit should be kept low for the first live test.
