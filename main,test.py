# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       matteo.del                                                   #
# 	Created:      9/23/2026, 2:50:58 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #


# ============================================================
# MOTOR SETUP: check these against the actual robot.
# ============================================================

# Change to True only after checking the motor configuration.
MOTOR_SETUP_CONFIRMED = False

# Order: left front, left rear, right front, right rear.
# These port numbers are placeholders from the notebook.
MOTOR_PORTS = (1, 2, 3, 4)
MOTOR_REVERSED = (False, False, True, True)

# Internal cartridge: 6 = blue, 18 = green, 36 = red.
MOTOR_CARTRIDGE = 18

# ============================================================
# ROBOT DIMENSIONS AND EXTERNAL GEARS
# ============================================================

WHEEL_DIAMETER_MM = 80.0
TRACK_WIDTH_MM = 235.0

MOTOR_GEAR_TEETH = 36.0
WHEEL_GEAR_TEETH = 60.0

# Adjust these after measuring actual turns on the floor.
TURN_RIGHT_SCALE = 1.0
TURN_LEFT_SCALE = 1.0

# ============================================================
# DRIVER SETTINGS
# ============================================================

# arcade: left stick forward/back + right stick left/right.
# tank: left stick controls left side; right stick controls right.
DRIVE_MODE = "arcade"

DRIVE_MAX_PERCENT = 80.0
SLOW_MAX_PERCENT = 30.0
DEADBAND = 6.0
RAMP_PERCENT_PER_SECOND = 250.0

LOOP_MS = 20
NEUTRAL_MS = 250

# Initial tuning values; adjust using the actual robot.
TURN_MAX_PERCENT = 35.0
TURN_MIN_PERCENT = 8.0
TURN_KP = 0.18
TURN_TOLERANCE_DEG = 5.0  # Motor degrees, not robot degrees.
TURN_SETTLE_MS = 120
TURN_TIMEOUT_MS = 5000

STALL_MS = 650
STALL_PROGRESS_DEG = 3.0
PAIR_MISMATCH_DEG = 45.0

remote_control_code_enabled = False


def setup_error():
    if not MOTOR_SETUP_CONFIRMED:
        return "Verify motor setup"

    if len(MOTOR_PORTS) != 4 or len(set(MOTOR_PORTS)) != 4:
        return "Use 4 unique ports"

    if any(p not in range(1, 22) for p in MOTOR_PORTS):
        return "Ports must be 1-21"

    if MOTOR_CARTRIDGE not in (6, 18, 36):
        return "Check cartridge"

    if len(MOTOR_REVERSED) != 4:
        return "Check motor reverse"

    if DRIVE_MODE not in ("arcade", "tank"):
        return "Check drive mode"

    return None


def positive_number(value):
    return (
        isinstance(value, (int, float))
        and 0 < value < float("inf")
    )


def turn_motor_degrees(angle):
    values = (
        WHEEL_DIAMETER_MM,
        TRACK_WIDTH_MM,
        MOTOR_GEAR_TEETH,
        WHEEL_GEAR_TEETH,
        TURN_RIGHT_SCALE,
        TURN_LEFT_SCALE,
    )

    if not all(positive_number(value) for value in values):
        return None

    scale = TURN_RIGHT_SCALE if angle > 0 else TURN_LEFT_SCALE

    return (
        angle
        * TRACK_WIDTH_MM
        / WHEEL_DIAMETER_MM
        * WHEEL_GEAR_TEETH
        / MOTOR_GEAR_TEETH
        * scale
    )


def shape_axis(value):
    value = max(-100.0, min(100.0, value))

    if abs(value) <= DEADBAND:
        return 0.0

    fraction = (abs(value) - DEADBAND) / (100.0 - DEADBAND)
    shaped = 100.0 * (0.5 * fraction + 0.5 * fraction ** 3)

    return shaped if value > 0 else -shaped


def manual_targets(axis3, steering, slow):
    first = shape_axis(axis3)
    second = shape_axis(steering)

    if DRIVE_MODE == "tank":
        left, right = first, second
    else:
        left, right = first + second, first - second

    divisor = max(100.0, abs(left), abs(right))
    cap = SLOW_MAX_PERCENT if slow else DRIVE_MAX_PERCENT

    return left / divisor * cap, right / divisor * cap


def ramp(previous, target, dt_ms):
    step = (
        RAMP_PERCENT_PER_SECOND
        * min(max(dt_ms, 0), 100)
        / 1000.0
    )

    return max(previous - step, min(previous + step, target))


class DriverGate:
    def __init__(self):
        self.field_seen = False

    def allowed(self, attached, enabled, driver, autonomous):
        self.field_seen = self.field_seen or attached

        if autonomous:
            return False

        if self.field_seen:
            return attached and enabled and driver

        return True


class DriveControl:
    def __init__(self):
        self.state = "WAIT"
        self.status = "Center sticks"
        self.outputs = (0.0, 0.0)
        self.previous_buttons = (False, False, False)
        self.neutral_since = None
        self.last_ms = None
        self.turn = None

    def stop(self, status, latch=True):
        self.turn = None
        self.outputs = (0.0, 0.0)
        self.state = "WAIT" if latch else "MANUAL"
        self.status = status
        self.neutral_since = None

        return self.outputs

    def start_turn(self, angle, positions, now_ms):
        amount = turn_motor_degrees(angle)

        if amount is None:
            self.stop("Set wheel/track mm")
            return

        offsets = (amount, amount, -amount, -amount)

        self.turn = {
            "targets": tuple(
                p + d for p, d in zip(positions, offsets)
            ),
            "origin": tuple(positions),
            "started": now_ms,
            "settled": None,
            "check_positions": list(positions),
            "progress_times": [now_ms] * 4,
        }

        self.state = "TURN"
        self.status = (
            ("Right " if angle > 0 else "Left ")
            + str(abs(angle))
        )
        self.outputs = (0.0, 0.0)

    def update_turn(self, positions, now_ms, dt_ms):
        turn = self.turn

        errors = [
            target - actual
            for target, actual in zip(turn["targets"], positions)
        ]

        if now_ms - turn["started"] >= TURN_TIMEOUT_MS:
            return self.stop("Turn timed out")

        travel = [
            p - start
            for p, start in zip(positions, turn["origin"])
        ]

        if (
            abs(travel[0] - travel[1]) > PAIR_MISMATCH_DEG
            or abs(travel[2] - travel[3]) > PAIR_MISMATCH_DEG
        ):
            return self.stop("Check motor pair")

        for i in range(4):
            if (
                abs(errors[i]) <= TURN_TOLERANCE_DEG
                or abs(
                    positions[i] - turn["check_positions"][i]
                ) >= STALL_PROGRESS_DEG
            ):
                turn["check_positions"][i] = positions[i]
                turn["progress_times"][i] = now_ms

            elif (
                now_ms - turn["progress_times"][i] >= STALL_MS
            ):
                return self.stop("Turn stalled")

        if all(
            abs(error) <= TURN_TOLERANCE_DEG
            for error in errors
        ):
            self.outputs = (0.0, 0.0)

            if turn["settled"] is None:
                turn["settled"] = now_ms

            if now_ms - turn["settled"] >= TURN_SETTLE_MS:
                return self.stop("Turn complete", latch=False)

            return self.outputs

        turn["settled"] = None
        commands = []

        for side in range(2):
            pair = errors[side * 2:side * 2 + 2]

            if all(
                abs(error) <= TURN_TOLERANCE_DEG
                for error in pair
            ):
                commands.append(0.0)
                continue

            error = sum(pair) / 2.0

            if abs(error) <= TURN_TOLERANCE_DEG:
                return self.stop("Check motor pair")

            magnitude = min(
                TURN_MAX_PERCENT,
                max(TURN_MIN_PERCENT, abs(error) * TURN_KP),
            )

            target = magnitude if error > 0 else -magnitude

            commands.append(
                ramp(self.outputs[side], target, dt_ms)
            )

        self.outputs = tuple(commands)
        return self.outputs

    def step(
        self,
        now_ms,
        axis3,
        steering,
        slow,
        buttons,
        stop_button,
        positions,
        enabled=True,
    ):
        dt_ms = (
            LOOP_MS
            if self.last_ms is None
            else now_ms - self.last_ms
        )

        self.last_ms = now_ms

        edges = tuple(
            current and not old
            for current, old in zip(
                buttons, self.previous_buttons
            )
        )

        self.previous_buttons = tuple(buttons)

        neutral = (
            abs(axis3) <= DEADBAND
            and abs(steering) <= DEADBAND
        )

        if not enabled:
            return self.stop("Drive disabled")

        if stop_button:
            return self.stop("B stop - center")

        if self.state == "WAIT":
            if neutral and not any(buttons):
                if self.neutral_since is None:
                    self.neutral_since = now_ms

                if (
                    now_ms - self.neutral_since >= NEUTRAL_MS
                ):
                    self.state = "MANUAL"
                    self.status = "Ready"

            else:
                self.neutral_since = None

            return (0.0, 0.0)

        if self.state == "TURN":
            if not neutral:
                self.stop("Manual override", latch=False)
            else:
                return self.update_turn(
                    positions, now_ms, dt_ms
                )

        if neutral and any(edges):
            if sum(bool(button) for button in buttons) != 1:
                return self.stop("Press one turn key")

            angle = (90, -90, 180)[edges.index(True)]
            self.start_turn(angle, positions, now_ms)

            return self.outputs

        if neutral:
            self.outputs = (0.0, 0.0)
            return self.outputs

        targets = manual_targets(axis3, steering, slow)
        cap = SLOW_MAX_PERCENT if slow else DRIVE_MAX_PERCENT

        self.outputs = tuple(
            max(-cap, min(cap, ramp(old, target, dt_ms)))
            for old, target in zip(self.outputs, targets)
        )

        self.status = "Slow drive" if slow else "Manual drive"

        return self.outputs


def run_robot():
    import vex

    brain = vex.Brain()
    controller = vex.Controller(vex.PRIMARY)

    error = setup_error()

    if error:
        brain.screen.print(error)
        brain.screen.next_row()
        brain.screen.print("Edit main.py motor setup")
        return

    cartridges = {
        6: vex.GearSetting.RATIO_6_1,
        18: vex.GearSetting.RATIO_18_1,
        36: vex.GearSetting.RATIO_36_1,
    }

    motors = tuple(
        vex.Motor(
            getattr(vex.Ports, "PORT" + str(port)),
            cartridges[MOTOR_CARTRIDGE],
            reverse,
        )
        for port, reverse in zip(
            MOTOR_PORTS, MOTOR_REVERSED
        )
    )

    groups = (
        vex.MotorGroup(motors[0], motors[1]),
        vex.MotorGroup(motors[2], motors[3]),
    )

    for group in groups:
        group.set_stopping(vex.BRAKE)
        group.stop()

    def competition_event():
        # The main loop alone controls the motors.
        # Autonomous driving is not implemented.
        pass

    competition = vex.Competition(
        competition_event,
        competition_event,
    )

    gate = DriverGate()
    drive = DriveControl()
    last_status = None

    brain.screen.print("Quantum driver assist")
    brain.screen.next_row()
    brain.screen.print("X right90  Y left90  A right180")
    brain.screen.next_row()
    brain.screen.print("B stop  L1 slow | encoder estimate")

    try:
        while True:
            field = (
                competition.is_field_control()
                or competition.is_competition_switch()
            )

            allowed = gate.allowed(
                field,
                competition.is_enabled(),
                competition.is_driver_control(),
                competition.is_autonomous(),
            )

            steering = (
                controller.axis2.position()
                if DRIVE_MODE == "tank"
                else controller.axis1.position()
            )

            buttons = (
                controller.buttonX.pressing(),
                controller.buttonY.pressing(),
                controller.buttonA.pressing(),
            )

            outputs = drive.step(
                brain.timer.time(vex.MSEC),
                controller.axis3.position(),
                steering,
                controller.buttonL1.pressing(),
                buttons,
                controller.buttonB.pressing(),
                tuple(
                    motor.position(vex.DEGREES)
                    for motor in motors
                ),
                allowed,
            )

            for group, output in zip(groups, outputs):
                if abs(output) < 0.01:
                    group.stop()
                else:
                    direction = (
                        vex.FORWARD
                        if output > 0
                        else vex.REVERSE
                    )

                    group.spin(
                        direction,
                        abs(output),
                        vex.PERCENT,
                    )

            if drive.status != last_status:
                controller.screen.clear_row(1)
                controller.screen.set_cursor(1, 1)
                controller.screen.print(drive.status[:19])
                last_status = drive.status

            vex.wait(LOOP_MS, vex.MSEC)

    finally:
        for group in groups:
            group.stop()


if __name__ == "__main__":
    run_robot()

brain.screen.print("Hello V5")
        
