# Fixed-wing firmware control notes

This is the working reference for `firmware/twin_wings` and
`firmware/single_wing`. It documents the control/mixing conventions to use, with
PX4, ArduPilot, and Betaflight as references rather than as code to copy.

## Conventions

- Body frame: forward `+X`, right `+Y`, down `+Z` (FRD), matching the rest of
  the repo where practical.
- Roll input positive = right wing down / right roll.
- Pitch input positive = nose up.
- Yaw input positive = nose right.
- Throttle is 0..1.
- Servo outputs are normalized 0..1 with 0.5 neutral until a real PWM driver is
  added.

## Twin Motor Flying Wing (`twin_wings`)

Reference mixing:

```
motor_left  = throttle + yaw * differential_gain
motor_right = throttle - yaw * differential_gain

elevon_left  = neutral + pitch * pitch_gain - roll * roll_gain
elevon_right = neutral + pitch * pitch_gain + roll * roll_gain
```

This matches the differential-thrust-plus-elevon model used by flying-wing
mixers in ArduPilot/PX4: yaw is produced by asymmetric thrust, while elevons
handle pitch and roll. Keep the elevon sign convention consistent with the
servo installation before flight.

## Single Motor Flying Wing (`single_wing`)

Reference mixing:

```
motor = throttle

servo_left  = neutral + pitch * pitch_gain - roll * roll_gain
servo_right = neutral + pitch * pitch_gain + roll * roll_gain
```

Single-motor wings have no differential-thrust yaw authority, so yaw is
produced by bank-and-pull (coordinated turns) rather than a mixer term.

## Next control-layer work

1. Replace `analogWrite` with a proper PWM driver: 50 Hz for servos, an
   ESC-safe rate for motors, with correct neutral pulse widths. *(Done in
   `src/pwm_output.*`.)*
2. Add RC input (CRSF/SBUS/PPM) or autopilot command input. *(Bench serial input
   now exists in `src/control_input.*`; real RC/autopilot input is next.)*
3. Add an attitude/rate controller. *(A proportional-plus-rate-damping-plus-integral
   controller now exists in `src/controller.*`; L1/TECS-lite helpers exist in
   `src/guidance.*`; a GPS course/waypoint navigation module exists in
   `src/navigation.*`. Next, define a waypoint source, call `guidanceToTarget()`
   before the controller, and tune against PX4 `fw_att_control`/
   `fw_pos_control_l1`, ArduPlane's TECS, and Betaflight's fixed-wing PID.)*
4. Add IMU and GPS drivers behind a small sensor interface. *(Interfaces exist in
   `src/sensors.*`; hardware drivers are still TODO.)*
