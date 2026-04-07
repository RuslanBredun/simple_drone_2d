from __future__ import annotations

import math
from dataclasses import dataclass, field

from pygame.math import Vector2

from constants import DRONE_MASS_KG, GRAVITY_MPS2, MAX_PITCH_DEG, MAX_THRUST_N, WORLD_HEIGHT_M, WORLD_WIDTH_M


@dataclass
class CommandState:
    throttle: float
    pitch: float


@dataclass
class DroneState:
    pos_m: Vector2
    vel_mps: Vector2
    angle_deg: float


@dataclass
class TargetState:
    pos_m: Vector2
    tolerance_m: float
    vel_mps: Vector2 = field(default_factory=Vector2)


@dataclass
class ControllerOutput:
    command: CommandState
    status_text: str
    debug_point_m: Vector2 | None = None


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(value, maximum))


def _reflect_coordinate(value: float, velocity: float, minimum: float, maximum: float) -> tuple[float, float]:
    if value < minimum:
        return minimum, abs(velocity)
    if value > maximum:
        return maximum, -abs(velocity)
    return value, velocity


def _predict_bounded_target_point(pos_m: Vector2, vel_mps: Vector2, horizon_s: float, step_s: float = 0.05) -> Vector2:
    predicted_pos = pos_m.copy()
    predicted_vel = vel_mps.copy()
    min_x = 0.5
    max_x = WORLD_WIDTH_M - 0.5
    min_y = 0.5
    max_y = WORLD_HEIGHT_M * 0.4

    t = 0.0
    while t < horizon_s:
        dt = min(step_s, horizon_s - t)
        predicted_pos += predicted_vel * dt
        predicted_pos.x, predicted_vel.x = _reflect_coordinate(predicted_pos.x, predicted_vel.x, min_x, max_x)
        predicted_pos.y, predicted_vel.y = _reflect_coordinate(predicted_pos.y, predicted_vel.y, min_y, max_y)
        t += dt

    return predicted_pos


@dataclass
class PIDController:
    kp: float
    ki: float
    kd: float
    integral_limit: float
    output_limit: float
    integral: float = 0.0
    prev_error: float = 0.0
    initialized: bool = False

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = 0.0
        self.initialized = False

    def update(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0

        self.integral += error * dt
        self.integral = _clamp(self.integral, -self.integral_limit, self.integral_limit)

        derivative = 0.0
        if self.initialized:
            derivative = (error - self.prev_error) / dt
        else:
            self.initialized = True

        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return _clamp(output, -self.output_limit, self.output_limit)


class AutopilotController:
    def __init__(
        self,
        *,
        x_pid: PIDController | None = None,
        y_pid: PIDController | None = None,
        velocity_damping_x: float = 1.6,
        velocity_damping_y: float = 2.0,
        command_smoothing: float = 0.18,
        max_pitch_command: float = 0.7,
        status_prefix: str = "auto pid-smooth",
    ) -> None:
        self.x_pid = x_pid or PIDController(kp=4.8, ki=0.02, kd=0.7, integral_limit=2.0, output_limit=5.0)
        self.y_pid = y_pid or PIDController(kp=1.6, ki=0.05, kd=0.9, integral_limit=2.0, output_limit=7.0)
        self.max_thrust_acc_mps2 = MAX_THRUST_N / DRONE_MASS_KG
        self.neutral_angle_deg = -90.0
        self.velocity_damping_x = velocity_damping_x
        self.velocity_damping_y = velocity_damping_y
        self.command_smoothing = command_smoothing
        self.max_pitch_command = max_pitch_command
        self.status_prefix = status_prefix
        self.prev_command = CommandState(
            throttle=_clamp(GRAVITY_MPS2.y / self.max_thrust_acc_mps2, 0.0, 1.0),
            pitch=0.0,
        )

    def reset(self) -> None:
        self.x_pid.reset()
        self.y_pid.reset()
        self.prev_command = CommandState(
            throttle=_clamp(GRAVITY_MPS2.y / self.max_thrust_acc_mps2, 0.0, 1.0),
            pitch=0.0,
        )

    def compute_command(
        self,
        state: DroneState,
        target: TargetState,
        dt: float,
    ) -> ControllerOutput:
        distance_to_target = state.pos_m.distance_to(target.pos_m)
        if distance_to_target <= target.tolerance_m and state.vel_mps.length() <= 0.2:
            self.reset()
            hover_throttle = _clamp(GRAVITY_MPS2.y / self.max_thrust_acc_mps2, 0.0, 1.0)
            command = CommandState(throttle=hover_throttle, pitch=0.0)
            status = f"auto hold   dist={distance_to_target:.2f} m"
            return ControllerOutput(command=command, status_text=status)

        error_x, error_y, command = self._compute_pid_command(state, target, dt)
        status = (
            f"{self.status_prefix}   ex={error_x:+.2f} m   ey={error_y:+.2f} m   "
            f"thr={command.throttle:.2f}   pitch={command.pitch:+.2f}"
        )
        return ControllerOutput(command=command, status_text=status)

    def _compute_pid_command(
        self,
        state: DroneState,
        target: TargetState,
        dt: float,
    ) -> tuple[float, float, CommandState]:
        error_x = target.pos_m.x - state.pos_m.x
        error_y = target.pos_m.y - state.pos_m.y
        desired_world_acc = Vector2(
            self.x_pid.update(error_x, dt) - self.velocity_damping_x * state.vel_mps.x,
            self.y_pid.update(error_y, dt) - self.velocity_damping_y * state.vel_mps.y,
        )
        command = self._command_from_desired_world_acc(desired_world_acc)
        return error_x, error_y, command

    def _command_from_desired_world_acc(self, desired_world_acc: Vector2) -> CommandState:
        required_thrust_acc = desired_world_acc - GRAVITY_MPS2
        thrust_acc_mag = required_thrust_acc.length()

        if thrust_acc_mag < 1e-6:
            desired_angle_deg = self.neutral_angle_deg
        else:
            desired_angle_deg = math.degrees(math.atan2(required_thrust_acc.y, required_thrust_acc.x))

        raw_pitch_command = _clamp(
            (desired_angle_deg - self.neutral_angle_deg) / MAX_PITCH_DEG,
            -self.max_pitch_command,
            self.max_pitch_command,
        )
        raw_throttle_command = _clamp(thrust_acc_mag / self.max_thrust_acc_mps2, 0.0, 1.0)
        throttle_command = self.prev_command.throttle + (raw_throttle_command - self.prev_command.throttle) * self.command_smoothing
        pitch_command = self.prev_command.pitch + (raw_pitch_command - self.prev_command.pitch) * self.command_smoothing
        command = CommandState(throttle=throttle_command, pitch=pitch_command)
        self.prev_command = command
        return command


class FastAutopilotController(AutopilotController):
    def __init__(self) -> None:
        super().__init__(
            x_pid=PIDController(kp=2.5, ki=0.1, kd=0.4, integral_limit=0.5, output_limit=116.0),
            y_pid=PIDController(kp=2.4, ki=0.01, kd=0.5, integral_limit=10.5, output_limit=54.0),
            velocity_damping_x=0.25,
            velocity_damping_y=0.35,
            command_smoothing=0.18,
            max_pitch_command=1.0,
            status_prefix="auto fast-hit",
        )

    def compute_command(
        self,
        state: DroneState,
        target: TargetState,
        dt: float,
    ) -> ControllerOutput:
        error_x, error_y, command = self._compute_pid_command(state, target, dt)
        status = (
            f"{self.status_prefix}   ex={error_x:+.2f} m   ey={error_y:+.2f} m   "
            f"thr={command.throttle:.2f}   pitch={command.pitch:+.2f}"
        )
        return ControllerOutput(command=command, status_text=status)


class InterceptAutopilotController:
    def __init__(self) -> None:
        self.x_pid = PIDController(kp=5.4, ki=1.0, kd=0.6, integral_limit=1.5, output_limit=7.0)
        self.y_pid = PIDController(kp=2.1, ki=0.0, kd=0.8, integral_limit=1.5, output_limit=8.0)
        self.max_thrust_acc_mps2 = MAX_THRUST_N / DRONE_MASS_KG
        self.neutral_angle_deg = -90.0
        self.relative_velocity_damping_x = 2.0
        self.relative_velocity_damping_y = 2.5
        self.command_smoothing = 0.22
        self.reference_speed_mps = 5.0
        self.min_lead_time_s = 0.15
        self.max_lead_time_s = 2.0
        self.prev_command = CommandState(
            throttle=_clamp(GRAVITY_MPS2.y / self.max_thrust_acc_mps2, 0.0, 1.0),
            pitch=0.0,
        )

    def reset(self) -> None:
        self.x_pid.reset()
        self.y_pid.reset()
        self.prev_command = CommandState(
            throttle=_clamp(GRAVITY_MPS2.y / self.max_thrust_acc_mps2, 0.0, 1.0),
            pitch=0.0,
        )

    def compute_command(
        self,
        state: DroneState,
        target: TargetState,
        dt: float,
    ) -> ControllerOutput:
        relative_pos = target.pos_m - state.pos_m
        relative_vel = target.vel_mps - state.vel_mps
        distance_to_target = relative_pos.length()
        relative_speed = relative_vel.length()

        if distance_to_target <= target.tolerance_m and relative_speed <= 0.35:
            desired_world_acc = Vector2(
                relative_vel.x * self.relative_velocity_damping_x,
                relative_vel.y * self.relative_velocity_damping_y,
            )
            command = self._acc_to_command(desired_world_acc)
            status = f"auto intercept hold   dist={distance_to_target:.2f} m   rel_v={relative_speed:.2f}"
            return ControllerOutput(command=command, status_text=status, debug_point_m=target.pos_m.copy())

        lead_time_s = _clamp(
            distance_to_target / max(self.reference_speed_mps, relative_speed + 1.0),
            self.min_lead_time_s,
            self.max_lead_time_s,
        )
        intercept_point = _predict_bounded_target_point(target.pos_m, target.vel_mps, lead_time_s)
        error = intercept_point - state.pos_m
        desired_world_acc = Vector2(
            self.x_pid.update(error.x, dt) - self.relative_velocity_damping_x * (state.vel_mps.x - target.vel_mps.x),
            self.y_pid.update(error.y, dt) - self.relative_velocity_damping_y * (state.vel_mps.y - target.vel_mps.y),
        )
        command = self._acc_to_command(desired_world_acc)
        status = (
            f"auto intercept   lead={lead_time_s:.2f} s   ex={error.x:+.2f} m   ey={error.y:+.2f} m   "
            f"thr={command.throttle:.2f}   pitch={command.pitch:+.2f}"
        )
        return ControllerOutput(command=command, status_text=status, debug_point_m=intercept_point)

    def _acc_to_command(self, desired_world_acc: Vector2) -> CommandState:
        required_thrust_acc = desired_world_acc - GRAVITY_MPS2
        thrust_acc_mag = required_thrust_acc.length()

        if thrust_acc_mag < 1e-6:
            desired_angle_deg = self.neutral_angle_deg
        else:
            desired_angle_deg = math.degrees(math.atan2(required_thrust_acc.y, required_thrust_acc.x))

        raw_pitch_command = _clamp((desired_angle_deg - self.neutral_angle_deg) / MAX_PITCH_DEG, -0.8, 0.8)
        raw_throttle_command = _clamp(thrust_acc_mag / self.max_thrust_acc_mps2, 0.0, 1.0)
        throttle_command = self.prev_command.throttle + (raw_throttle_command - self.prev_command.throttle) * self.command_smoothing
        pitch_command = self.prev_command.pitch + (raw_pitch_command - self.prev_command.pitch) * self.command_smoothing
        command = CommandState(throttle=throttle_command, pitch=pitch_command)
        self.prev_command = command
        return command
