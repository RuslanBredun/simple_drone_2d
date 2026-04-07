import math
import random
from dataclasses import dataclass
from collections import deque

import pygame
from pygame.math import Vector2
from controller import (
    AutopilotController,
    CommandState,
    ControllerOutput,
    DroneState,
    FastAutopilotController,
    InterceptAutopilotController,
    TargetState,
)

from constants import (
    BG_COLOR,
    DRONE_MASS_KG,
    GRAVITY_MPS2,
    HUD_LINE_SPACING,
    HUD_TEXT_OFFSET,
    MAIN_BALL_COLOR,
    MAIN_BODY_HEIGHT_M,
    MAIN_BODY_WIDTH_M,
    MAX_THRUST_N,
    MAX_PITCH_DEG,
    PIXELS_PER_METER,
    PITCH_INPUT_STEP,
    SIM_RECT,
    TEXT_COLOR,
    THROTTLE_INPUT_STEP,
    UI_PITCH_COLOR,
    TARGET_MAX_DIRECTION_DEG,
    TARGET_RETARGET_MAX_S,
    TARGET_RETARGET_MIN_S,
    TARGET_SMOOTHING_RATE,
    TARGET_SPEED_MAX_MPS,
    TARGET_SPEED_MIN_MPS,
    WIND_ACCEL_MAX_MPS2,
    WIND_ACCEL_MIN_MPS2,
    WIND_MAX_DIRECTION_DEG,
    WIND_RETARGET_MAX_S,
    WIND_RETARGET_MIN_S,
    WIND_SMOOTHING_RATE,
    WORLD_HEIGHT_M,
    WORLD_WIDTH_M,
    WIDTH,
    HEIGHT,
)
from ui import DroneControlPanel
from target_utils import generate_random_target
from drone_body import Drone


pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Main Body Control")
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 30)
small_font = pygame.font.SysFont(None, 22)


@dataclass
class TaskState:
    start_pos_m: Vector2
    target_pos_m: Vector2
    tolerance_m: float
    reached: bool = False


class WindModel:
    def __init__(self) -> None:
        self.current_acc_mps2 = self._random_target_vector()
        self.target_acc_mps2 = self.current_acc_mps2.copy()
        self.time_to_retarget_s = self._random_retarget_time()

    def _random_retarget_time(self) -> float:
        return random.uniform(WIND_RETARGET_MIN_S, WIND_RETARGET_MAX_S)

    def _random_target_vector(self) -> Vector2:
        magnitude = random.uniform(WIND_ACCEL_MIN_MPS2, WIND_ACCEL_MAX_MPS2)
        angle_rad = math.radians(random.uniform(-WIND_MAX_DIRECTION_DEG, WIND_MAX_DIRECTION_DEG))
        horizontal_sign = random.choice((-1.0, 1.0))
        return Vector2(horizontal_sign * math.cos(angle_rad), math.sin(angle_rad)) * magnitude

    def update(self, dt: float) -> Vector2:
        if dt <= 0.0:
            return self.current_acc_mps2.copy()

        self.time_to_retarget_s -= dt
        if self.time_to_retarget_s <= 0.0:
            self.target_acc_mps2 = self._random_target_vector()
            self.time_to_retarget_s = self._random_retarget_time()

        blend = 1.0 - math.exp(-WIND_SMOOTHING_RATE * dt)
        self.current_acc_mps2 += (self.target_acc_mps2 - self.current_acc_mps2) * blend
        return self.current_acc_mps2.copy()


class TargetMotionModel:
    def __init__(self) -> None:
        self.current_vel_mps = self._random_target_velocity()
        self.target_vel_mps = self.current_vel_mps.copy()
        self.time_to_retarget_s = self._random_retarget_time()

    def _random_retarget_time(self) -> float:
        return random.uniform(TARGET_RETARGET_MIN_S, TARGET_RETARGET_MAX_S)

    def _random_target_velocity(self) -> Vector2:
        speed = random.uniform(TARGET_SPEED_MIN_MPS, TARGET_SPEED_MAX_MPS)
        angle_rad = math.radians(random.uniform(-TARGET_MAX_DIRECTION_DEG, TARGET_MAX_DIRECTION_DEG))
        horizontal_sign = random.choice((-1.0, 1.0))
        return Vector2(horizontal_sign * math.cos(angle_rad), math.sin(angle_rad)) * speed

    def reset(self) -> None:
        self.current_vel_mps = self._random_target_velocity()
        self.target_vel_mps = self.current_vel_mps.copy()
        self.time_to_retarget_s = self._random_retarget_time()

    def update(self, target_pos_m: Vector2, dt: float) -> Vector2:
        if dt <= 0.0:
            return self.current_vel_mps.copy()

        self.time_to_retarget_s -= dt
        if self.time_to_retarget_s <= 0.0:
            self.target_vel_mps = self._random_target_velocity()
            self.time_to_retarget_s = self._random_retarget_time()

        blend = 1.0 - math.exp(-TARGET_SMOOTHING_RATE * dt)
        self.current_vel_mps += (self.target_vel_mps - self.current_vel_mps) * blend
        target_pos_m += self.current_vel_mps * dt

        min_x = 0.5
        max_x = WORLD_WIDTH_M - 0.5
        min_y = 0.5
        max_y = WORLD_HEIGHT_M * 0.4

        if target_pos_m.x < min_x:
            target_pos_m.x = min_x
            self.current_vel_mps.x = abs(self.current_vel_mps.x)
            self.target_vel_mps.x = abs(self.target_vel_mps.x)
        elif target_pos_m.x > max_x:
            target_pos_m.x = max_x
            self.current_vel_mps.x = -abs(self.current_vel_mps.x)
            self.target_vel_mps.x = -abs(self.target_vel_mps.x)

        if target_pos_m.y < min_y:
            target_pos_m.y = min_y
            self.current_vel_mps.y = abs(self.current_vel_mps.y)
            self.target_vel_mps.y = abs(self.target_vel_mps.y)
        elif target_pos_m.y > max_y:
            target_pos_m.y = max_y
            self.current_vel_mps.y = -abs(self.current_vel_mps.y)
            self.target_vel_mps.y = -abs(self.target_vel_mps.y)

        return self.current_vel_mps.copy()


def is_reset_event(event: pygame.event.Event) -> bool:
    event_key = getattr(event, "key", None)
    event_char = (getattr(event, "unicode", "") or "").lower()
    return event_key == pygame.K_r or event_char == "r"


def world_to_screen(point_m: Vector2) -> tuple[int, int]:
    return (
        round(SIM_RECT.left + point_m.x * PIXELS_PER_METER),
        round(SIM_RECT.top + point_m.y * PIXELS_PER_METER),
    )


def append_history(history: deque[Vector2], point_m: Vector2, min_spacing_m: float = 0.04) -> None:
    if not history or history[-1].distance_to(point_m) >= min_spacing_m:
        history.append(point_m.copy())


def clear_history(history: deque[Vector2], point_m: Vector2) -> None:
    history.clear()
    history.append(point_m.copy())


def reflect_coordinate(value: float, velocity: float, minimum: float, maximum: float) -> tuple[float, float]:
    if value < minimum:
        return minimum, abs(velocity)
    if value > maximum:
        return maximum, -abs(velocity)
    return value, velocity


def assign_new_target(task: TaskState, body: Drone, target_motion: TargetMotionModel | None = None) -> None:
    extent_x, extent_y = body.axis_extents()
    task.target_pos_m = generate_random_target(extent_x, extent_y, WORLD_WIDTH_M, WORLD_HEIGHT_M)
    task.reached = False
    if target_motion is not None:
        target_motion.reset()


def draw_hud(
    surface: pygame.Surface,
    paused: bool,
    body: Drone,
    task: TaskState,
    command: CommandState,
    mode: str,
    wind_acc_mps2: Vector2,
    wind_enabled: bool,
    controller_output: ControllerOutput | None = None,
) -> None:
    controls = font.render("W/S throttle  A/D pitch  0 manual  1 smooth  2 fast  3 intercept  R reset", True, TEXT_COLOR)
    surface.blit(controls, (SIM_RECT.left + HUD_TEXT_OFFSET, HUD_TEXT_OFFSET))

    state = "PAUSED" if paused else "RUNNING"
    info = font.render(
        f"{state}   mode={mode.upper()}   angle={body.angle_deg:5.1f}°   pos=({body.pos.x:5.2f} m, {body.pos.y:5.2f} m)",
        True,
        TEXT_COLOR,
    )
    surface.blit(info, (SIM_RECT.left + HUD_TEXT_OFFSET, HUD_TEXT_OFFSET + HUD_LINE_SPACING))

    hint = small_font.render(
        f"vel=({body.vel.x:+.2f}, {body.vel.y:+.2f}) m/s   throttle={command.throttle:.2f}   pitch={command.pitch:+.2f}",
        True,
        TEXT_COLOR,
    )
    surface.blit(hint, (SIM_RECT.left + HUD_TEXT_OFFSET, HUD_TEXT_OFFSET + HUD_LINE_SPACING * 2))

    wind_text = small_font.render(
        f"wind={'on' if wind_enabled else 'off'}   ax={wind_acc_mps2.x:+.2f}   ay={wind_acc_mps2.y:+.2f} m/s^2",
        True,
        TEXT_COLOR,
    )
    surface.blit(wind_text, (SIM_RECT.left + HUD_TEXT_OFFSET, HUD_TEXT_OFFSET + HUD_LINE_SPACING * 3))

    distance_to_target = body.pos.distance_to(task.target_pos_m)
    task_text = small_font.render(
        f"target=({task.target_pos_m.x:.2f} m, {task.target_pos_m.y:.2f} m)   dist={distance_to_target:.2f} m   reached={task.reached}",
        True,
        TEXT_COLOR,
    )
    surface.blit(task_text, (SIM_RECT.left + HUD_TEXT_OFFSET, HUD_TEXT_OFFSET + HUD_LINE_SPACING * 4))

    if controller_output is not None:
        controller_text = small_font.render(controller_output.status_text, True, TEXT_COLOR)
        surface.blit(controller_text, (SIM_RECT.left + HUD_TEXT_OFFSET, HUD_TEXT_OFFSET + HUD_LINE_SPACING * 5))


def draw_task_markers(surface: pygame.Surface, task: TaskState, target_vel_mps: Vector2) -> None:
    target_center = world_to_screen(task.target_pos_m)
    tolerance_px = max(4, round(task.tolerance_m * PIXELS_PER_METER))
    pygame.draw.circle(surface, TEXT_COLOR, target_center, tolerance_px, width=1)
    pygame.draw.line(surface, TEXT_COLOR, (target_center[0] - 10, target_center[1]), (target_center[0] + 10, target_center[1]), 2)
    pygame.draw.line(surface, TEXT_COLOR, (target_center[0], target_center[1] - 10), (target_center[0], target_center[1] + 10), 2)

    if target_vel_mps.length() > 1e-6:
        arrow_scale = 24.0
        target_vec = target_vel_mps * arrow_scale
        arrow_end_vec = Vector2(target_center) + target_vec
        arrow_end = (round(arrow_end_vec.x), round(arrow_end_vec.y))
        pygame.draw.line(surface, UI_PITCH_COLOR, target_center, arrow_end, 3)

        direction = target_vec.normalize()
        head_length = 10.0
        left_head = direction.rotate(28.0) * head_length
        right_head = direction.rotate(-28.0) * head_length
        left_point = arrow_end_vec - left_head
        right_point = arrow_end_vec - right_head
        pygame.draw.line(surface, UI_PITCH_COLOR, arrow_end, (round(left_point.x), round(left_point.y)), 3)
        pygame.draw.line(surface, UI_PITCH_COLOR, arrow_end, (round(right_point.x), round(right_point.y)), 3)

    start_center = world_to_screen(task.start_pos_m)
    pygame.draw.circle(surface, UI_PITCH_COLOR, start_center, 6)


def draw_path(surface: pygame.Surface, history: deque[Vector2], color: tuple[int, int, int], width: int = 2) -> None:
    if len(history) < 2:
        return
    pygame.draw.lines(surface, color, False, [world_to_screen(point) for point in history], width)


def draw_future_points(surface: pygame.Surface, points_m: list[Vector2], color: tuple[int, int, int], radius: int = 3) -> None:
    for point in points_m:
        pygame.draw.circle(surface, color, world_to_screen(point), radius)


def draw_intercept_marker(surface: pygame.Surface, intercept_point_m: Vector2, target_pos_m: Vector2) -> None:
    intercept_center = world_to_screen(intercept_point_m)
    target_center = world_to_screen(target_pos_m)
    intercept_color = (120, 255, 180)

    pygame.draw.circle(surface, intercept_color, intercept_center, 8, width=2)
    pygame.draw.line(
        surface,
        intercept_color,
        (intercept_center[0] - 8, intercept_center[1] - 8),
        (intercept_center[0] + 8, intercept_center[1] + 8),
        2,
    )
    pygame.draw.line(
        surface,
        intercept_color,
        (intercept_center[0] - 8, intercept_center[1] + 8),
        (intercept_center[0] + 8, intercept_center[1] - 8),
        2,
    )
    pygame.draw.line(surface, intercept_color, intercept_center, target_center, 1)


def draw_wind_vector(surface: pygame.Surface, wind_acc_mps2: Vector2, enabled: bool) -> None:
    origin = (SIM_RECT.right - 90, SIM_RECT.top + 90)
    frame_rect = pygame.Rect(0, 0, 132, 132)
    frame_rect.center = origin
    pygame.draw.rect(surface, (24, 26, 32), frame_rect, border_radius=14)
    pygame.draw.rect(surface, TEXT_COLOR, frame_rect, width=2, border_radius=14)

    label = small_font.render("Wind", True, TEXT_COLOR)
    surface.blit(label, (frame_rect.left + 12, frame_rect.top + 10))

    center = (frame_rect.centerx, frame_rect.centery + 10)
    pygame.draw.circle(surface, TEXT_COLOR, center, 4)

    if not enabled or wind_acc_mps2.length() < 1e-6:
        off_text = small_font.render("off", True, TEXT_COLOR)
        surface.blit(off_text, (frame_rect.left + 12, frame_rect.bottom - 28))
        return

    scale_px_per_mps2 = 26.0
    arrow_vec = Vector2(wind_acc_mps2.x, wind_acc_mps2.y) * scale_px_per_mps2
    arrow_end_vec = Vector2(center) + arrow_vec
    arrow_end = (round(arrow_end_vec.x), round(arrow_end_vec.y))
    pygame.draw.line(surface, TEXT_COLOR, center, arrow_end, 3)

    direction = arrow_vec.normalize()
    head_length = 12.0
    head_angle_deg = 26.0
    left_head = direction.rotate(head_angle_deg) * head_length
    right_head = direction.rotate(-head_angle_deg) * head_length
    left_point = arrow_end_vec - left_head
    right_point = arrow_end_vec - right_head
    pygame.draw.line(surface, TEXT_COLOR, arrow_end, (round(left_point.x), round(left_point.y)), 3)
    pygame.draw.line(surface, TEXT_COLOR, arrow_end, (round(right_point.x), round(right_point.y)), 3)

    magnitude_text = small_font.render(f"{wind_acc_mps2.length():.2f} m/s^2", True, TEXT_COLOR)
    surface.blit(magnitude_text, (frame_rect.left + 12, frame_rect.bottom - 28))


def predict_drone_future_path(
    body: Drone,
    command: CommandState,
    wind_acc_mps2: Vector2,
    horizon_s: float = 2.0,
    step_s: float = 0.1,
) -> list[Vector2]:
    future_points: list[Vector2] = []
    pos = body.pos.copy()
    vel = body.vel.copy()
    angle_deg = body.base_angle_deg + command.pitch * MAX_PITCH_DEG
    angle_rad = math.radians(angle_deg)
    normal = Vector2(math.cos(angle_rad), math.sin(angle_rad))
    thrust_force = normal * (MAX_THRUST_N * command.throttle)
    thrust_acc = thrust_force / body.mass_kg
    acc = GRAVITY_MPS2 + thrust_acc + wind_acc_mps2

    t = 0.0
    while t < horizon_s:
        vel += acc * step_s
        pos += vel * step_s
        future_points.append(pos.copy())
        t += step_s

    return future_points


def predict_target_future_path(
    target_pos_m: Vector2,
    target_vel_mps: Vector2,
    horizon_s: float = 2.0,
    step_s: float = 0.1,
) -> list[Vector2]:
    future_points: list[Vector2] = []
    pos = target_pos_m.copy()
    vel = target_vel_mps.copy()
    min_x = 0.5
    max_x = WORLD_WIDTH_M - 0.5
    min_y = 0.5
    max_y = WORLD_HEIGHT_M * 0.4

    t = 0.0
    while t < horizon_s:
        pos += vel * step_s
        pos.x, vel.x = reflect_coordinate(pos.x, vel.x, min_x, max_x)
        pos.y, vel.y = reflect_coordinate(pos.y, vel.y, min_y, max_y)
        future_points.append(pos.copy())
        t += step_s

    return future_points


def update_task_state(task: TaskState, body: Drone) -> None:
    task.reached = body.pos.distance_to(task.target_pos_m) <= task.tolerance_m


def get_manual_command(control_panel: DroneControlPanel) -> CommandState:
    throttle, pitch = control_panel.command_values()
    return CommandState(throttle=throttle, pitch=pitch)


def make_drone_state(body: Drone) -> DroneState:
    return DroneState(
        pos_m=body.pos.copy(),
        vel_mps=body.vel.copy(),
        angle_deg=body.angle_deg,
    )


def make_target_state(task: TaskState, target_vel_mps: Vector2) -> TargetState:
    return TargetState(
        pos_m=task.target_pos_m.copy(),
        tolerance_m=task.tolerance_m,
        vel_mps=target_vel_mps.copy(),
    )


def reset_body(
    body: Drone,
    task: TaskState,
    control_panel: DroneControlPanel | None = None,
    autopilots: list[object] | None = None,
    body_history: deque[Vector2] | None = None,
) -> None:
    body.angle_deg = body.base_angle_deg
    body.pos = task.start_pos_m.copy()
    body.vel = Vector2(0, 0)
    task.reached = False
    if body_history is not None:
        clear_history(body_history, body.pos)
    if control_panel is not None:
        control_panel.reset()
    if autopilots is not None:
        for autopilot in autopilots:
            autopilot.reset()


def apply_boundary_rules(
    body: Drone,
    task: TaskState,
    control_panel: DroneControlPanel | None = None,
    autopilots: list[object] | None = None,
    body_history: deque[Vector2] | None = None,
) -> None:
    extent_x, extent_y = body.axis_extents()

    if body.pos.x - extent_x < 0.0:
        reset_body(body, task, control_panel, autopilots, body_history)
        return
    if body.pos.x + extent_x > WORLD_WIDTH_M:
        reset_body(body, task, control_panel, autopilots, body_history)
        return
    if body.pos.y - extent_y < 0.0:
        reset_body(body, task, control_panel, autopilots, body_history)
        return
    if body.pos.y + extent_y > WORLD_HEIGHT_M:
        body.pos.y = WORLD_HEIGHT_M - extent_y
        body.vel.y = min(body.vel.y, 0.0)
        


def main() -> None:
    body = Drone(
        Vector2(WORLD_WIDTH_M * 0.5, WORLD_HEIGHT_M * 0.5),
        MAIN_BODY_WIDTH_M,
        MAIN_BODY_HEIGHT_M,
        DRONE_MASS_KG,
    )
    extent_x, extent_y = body.axis_extents()
    task = TaskState(
        start_pos_m=Vector2(WORLD_WIDTH_M * 0.2, WORLD_HEIGHT_M - extent_y),
        target_pos_m=generate_random_target(extent_x, extent_y, WORLD_WIDTH_M, WORLD_HEIGHT_M),
        tolerance_m=0.25,
    )
    control_panel = DroneControlPanel()
    smooth_autopilot = AutopilotController()
    fast_autopilot = FastAutopilotController()
    intercept_autopilot = InterceptAutopilotController()
    wind_model = WindModel()
    target_motion = TargetMotionModel()
    autopilots = [smooth_autopilot, fast_autopilot, intercept_autopilot]
    body_history: deque[Vector2] = deque(maxlen=240)
    target_history: deque[Vector2] = deque(maxlen=240)
    reset_body(body, task, control_panel, autopilots, body_history)
    clear_history(target_history, task.target_pos_m)

    paused = False
    running = True
    mode = "manual"
    wind_acc_mps2 = Vector2(0, 0)
    target_vel_mps = Vector2(0, 0)

    while running:
        dt = clock.tick(60) / 1000.0
        controller_output: ControllerOutput | None = None
        command = get_manual_command(control_panel) if mode == "manual" else CommandState(throttle=0.0, pitch=0.0)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    paused = not paused
                elif is_reset_event(event):
                    reset_body(body, task, control_panel, autopilots, body_history)
                elif event.key == pygame.K_0:
                    mode = "manual"
                elif event.key == pygame.K_1:
                    mode = "auto_smooth"
                    smooth_autopilot.reset()
                elif event.key == pygame.K_2:
                    mode = "auto_fast"
                    fast_autopilot.reset()
                elif event.key == pygame.K_3:
                    mode = "auto_intercept"
                    intercept_autopilot.reset()
                elif mode == "manual":
                    control_panel.handle_key_press(event, THROTTLE_INPUT_STEP, PITCH_INPUT_STEP)

            control_panel.handle_event(event)

        if not paused:
            target_vel_mps = target_motion.update(task.target_pos_m, dt)
            append_history(target_history, task.target_pos_m)

        if mode == "manual":
            command = get_manual_command(control_panel)
        elif mode == "auto_smooth":
            controller_output = smooth_autopilot.compute_command(
                state=make_drone_state(body),
                target=make_target_state(task, target_vel_mps),
                dt=dt,
            )
            command = controller_output.command
        elif mode == "auto_fast":
            controller_output = fast_autopilot.compute_command(
                state=make_drone_state(body),
                target=make_target_state(task, target_vel_mps),
                dt=dt,
            )
            command = controller_output.command
        else:
            controller_output = intercept_autopilot.compute_command(
                state=make_drone_state(body),
                target=make_target_state(task, target_vel_mps),
                dt=dt,
            )
            command = controller_output.command

        if not paused:
            body.angle_deg = body.base_angle_deg + command.pitch * MAX_PITCH_DEG
            body.angle_deg = (body.angle_deg + 180.0) % 360.0 - 180.0

            thrust_force = body.normal * (MAX_THRUST_N * command.throttle)
            thrust_acc = thrust_force / body.mass_kg
            if control_panel.is_wind_enabled():
                wind_acc_mps2 = wind_model.update(dt)
            else:
                wind_acc_mps2 = Vector2(0, 0)
            acc = GRAVITY_MPS2 + thrust_acc + wind_acc_mps2

            body.vel += acc * dt
            body.pos += body.vel * dt
            append_history(body_history, body.pos)
            apply_boundary_rules(body, task, control_panel, autopilots, body_history)
            update_task_state(task, body)
            if task.reached and control_panel.is_auto_target_enabled():
                assign_new_target(task, body, target_motion)
                clear_history(target_history, task.target_pos_m)
        else:
            if not control_panel.is_wind_enabled():
                wind_acc_mps2 = Vector2(0, 0)

        predicted_target_path = predict_target_future_path(task.target_pos_m, target_vel_mps)
        predicted_drone_path = predict_drone_future_path(body, command, wind_acc_mps2)

        screen.fill(BG_COLOR)
        draw_path(screen, target_history, UI_PITCH_COLOR, width=2)
        draw_path(screen, body_history, (255, 170, 120), width=2)
        draw_future_points(screen, predicted_target_path, (120, 220, 255), radius=3)
        draw_future_points(screen, predicted_drone_path, (255, 210, 120), radius=3)
        draw_task_markers(screen, task, target_vel_mps)
        if controller_output is not None and controller_output.debug_point_m is not None:
            draw_intercept_marker(screen, controller_output.debug_point_m, task.target_pos_m)
        draw_wind_vector(screen, wind_acc_mps2, control_panel.is_wind_enabled())
        body.draw(screen, world_to_screen, MAIN_BALL_COLOR)
        draw_hud(
            screen,
            paused,
            body,
            task,
            command,
            mode,
            wind_acc_mps2,
            control_panel.is_wind_enabled(),
            controller_output,
        )
        control_panel.draw(screen, mode=mode, current_command=command)
        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()
