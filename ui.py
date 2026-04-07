from __future__ import annotations

from dataclasses import dataclass

import pygame
from controller import CommandState

from constants import (
    MAX_THRUST_TO_WEIGHT,
    TEXT_COLOR,
    UI_BORDER_COLOR,
    UI_PANEL_COLOR,
    UI_PITCH_COLOR,
    UI_RECT,
    UI_THROTTLE_COLOR,
    UI_TRACK_COLOR,
)


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(value, maximum))


def _snap_to_step(value: float, minimum: float, maximum: float, step: float) -> float:
    if step <= 0:
        return _clamp(value, minimum, maximum)
    snapped = round((value - minimum) / step) * step + minimum
    return _clamp(snapped, minimum, maximum)


def _event_matches(
    event: pygame.event.Event,
    *,
    keys: tuple[int, ...] = (),
    chars: tuple[str, ...] = (),
) -> bool:
    event_key = getattr(event, "key", None)
    event_char = (getattr(event, "unicode", "") or "").lower()
    return event_key in keys or event_char in chars


@dataclass
class Slider:
    label: str
    rect: pygame.Rect
    minimum: float
    maximum: float
    value: float
    default: float
    accent_color: tuple[int, int, int]
    orientation: str = "vertical"
    dragging: bool = False

    def __post_init__(self) -> None:
        self.value = _clamp(self.value, self.minimum, self.maximum)
        self.default = _clamp(self.default, self.minimum, self.maximum)

    def normalized(self) -> float:
        span = self.maximum - self.minimum
        if span <= 0:
            return 0.0
        return (self.value - self.minimum) / span

    def set_value(self, value: float) -> None:
        self.value = _clamp(value, self.minimum, self.maximum)

    def adjust(self, delta: float) -> None:
        self.set_value(self.value + delta)

    def step_toward(self, direction: int, step: float) -> None:
        if direction == 0:
            return

        next_value = _snap_to_step(self.value, self.minimum, self.maximum, step)
        next_value += direction * step
        next_value = _snap_to_step(next_value, self.minimum, self.maximum, step)

        if abs(next_value - self.default) <= step * 0.5:
            next_value = self.default

        self.set_value(next_value)

    def handle_event(self, event: pygame.event.Event) -> bool:
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 and self.rect.collidepoint(event.pos):
            self.dragging = True
            self._set_from_pos(event.pos)
            return True
        if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            was_dragging = self.dragging
            self.dragging = False
            return was_dragging
        if event.type == pygame.MOUSEMOTION and self.dragging:
            self._set_from_pos(event.pos)
            return True
        return False

    def _set_from_pos(self, pos: tuple[int, int]) -> None:
        if self.orientation == "vertical":
            ratio = 1.0 - ((pos[1] - self.rect.top) / max(self.rect.height, 1))
        else:
            ratio = (pos[0] - self.rect.left) / max(self.rect.width, 1)
        ratio = _clamp(ratio, 0.0, 1.0)
        self.set_value(self.minimum + ratio * (self.maximum - self.minimum))

    def handle_center(self) -> tuple[int, int]:
        ratio = self.normalized()
        return self._handle_center_from_ratio(ratio)

    def handle_center_for_value(self, value: float) -> tuple[int, int]:
        span = self.maximum - self.minimum
        if span <= 0:
            return self.handle_center()
        ratio = _clamp((value - self.minimum) / span, 0.0, 1.0)
        return self._handle_center_from_ratio(ratio)

    def _handle_center_from_ratio(self, ratio: float) -> tuple[int, int]:
        if self.orientation == "vertical":
            x = self.rect.centerx
            y = round(self.rect.bottom - ratio * self.rect.height)
            return x, y
        x = round(self.rect.left + ratio * self.rect.width)
        y = self.rect.centery
        return x, y

    def draw(self, surface: pygame.Surface, font: pygame.font.Font, value_font: pygame.font.Font) -> None:
        label_surface = font.render(self.label, True, TEXT_COLOR)
        label_rect = label_surface.get_rect(midbottom=(self.rect.centerx, self.rect.top - 18))
        surface.blit(label_surface, label_rect)

        pygame.draw.rect(surface, UI_TRACK_COLOR, self.rect, border_radius=10)
        pygame.draw.rect(surface, UI_BORDER_COLOR, self.rect, width=2, border_radius=10)

        if self.minimum < 0 < self.maximum:
            zero_ratio = (0.0 - self.minimum) / (self.maximum - self.minimum)
            zero_x = round(self.rect.left + zero_ratio * self.rect.width)
            pygame.draw.line(
                surface,
                UI_BORDER_COLOR,
                (zero_x, self.rect.top - 6),
                (zero_x, self.rect.bottom + 6),
                2,
            )

        handle_x, handle_y = self.handle_center()
        handle_width = 20 if self.orientation == "vertical" else 16
        handle_height = 16 if self.orientation == "vertical" else 24
        handle_rect = pygame.Rect(0, 0, handle_width, handle_height)
        handle_rect.center = (handle_x, handle_y)
        pygame.draw.rect(surface, self.accent_color, handle_rect, border_radius=8)
        pygame.draw.rect(surface, UI_BORDER_COLOR, handle_rect, width=2, border_radius=8)

        value_surface = value_font.render(f"{self.value:+.2f}" if self.minimum < 0 else f"{self.value:.2f}", True, TEXT_COLOR)
        value_rect = value_surface.get_rect(midtop=(self.rect.centerx, self.rect.bottom + 14))
        surface.blit(value_surface, value_rect)

    def draw_indicator(self, surface: pygame.Surface, value: float, color: tuple[int, int, int]) -> None:
        handle_x, handle_y = self.handle_center_for_value(value)
        radius = 8
        pygame.draw.circle(surface, color, (handle_x, handle_y), radius, width=2)


@dataclass
class Checkbox:
    label: str
    rect: pygame.Rect
    checked: bool = False

    def handle_event(self, event: pygame.event.Event) -> bool:
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 and self.rect.collidepoint(event.pos):
            self.checked = not self.checked
            return True
        return False

    def draw(self, surface: pygame.Surface, font: pygame.font.Font) -> None:
        pygame.draw.rect(surface, UI_TRACK_COLOR, self.rect, border_radius=4)
        pygame.draw.rect(surface, UI_BORDER_COLOR, self.rect, width=2, border_radius=4)
        if self.checked:
            pygame.draw.line(surface, TEXT_COLOR, (self.rect.left + 4, self.rect.centery), (self.rect.centerx, self.rect.bottom - 4), 2)
            pygame.draw.line(surface, TEXT_COLOR, (self.rect.centerx, self.rect.bottom - 4), (self.rect.right - 4, self.rect.top + 4), 2)

        label_surface = font.render(self.label, True, TEXT_COLOR)
        label_rect = label_surface.get_rect(midleft=(self.rect.right + 10, self.rect.centery))
        surface.blit(label_surface, label_rect)


class DroneControlPanel:
    def __init__(self) -> None:
        self.rect = UI_RECT.copy()
        self.title_font = pygame.font.SysFont(None, 34)
        self.label_font = pygame.font.SysFont(None, 28)
        self.value_font = pygame.font.SysFont(None, 24)
        self.help_font = pygame.font.SysFont(None, 22)

        throttle_rect = pygame.Rect(self.rect.left + 58, self.rect.top + 110, 56, self.rect.height - 300)
        pitch_rect = pygame.Rect(self.rect.left + 150, self.rect.top + self.rect.height // 2 - 18, 94, 36)

        self.throttle = Slider(
            label="Throttle",
            rect=throttle_rect,
            minimum=0.0,
            maximum=1.0,
            value=0.0,
            default=0.0,
            accent_color=UI_THROTTLE_COLOR,
            orientation="vertical",
        )
        self.pitch = Slider(
            label="Pitch",
            rect=pitch_rect,
            minimum=-1.0,
            maximum=1.0,
            value=0.0,
            default=0.0,
            accent_color=UI_PITCH_COLOR,
            orientation="horizontal",
        )
        auto_target_rect = pygame.Rect(self.rect.left + 24, self.rect.bottom - 190, 22, 22)
        self.auto_target_checkbox = Checkbox(
            label="New target on reach",
            rect=auto_target_rect,
            checked=True,
        )
        moving_target_rect = pygame.Rect(self.rect.left + 24, self.rect.bottom - 162, 22, 22)
        self.moving_target_checkbox = Checkbox(
            label="Moving target",
            rect=moving_target_rect,
            checked=True,
        )
        wind_rect = pygame.Rect(self.rect.left + 24, self.rect.bottom - 134, 22, 22)
        self.wind_checkbox = Checkbox(
            label="Enable wind",
            rect=wind_rect,
            checked=False,
        )

    def handle_event(self, event: pygame.event.Event) -> bool:
        handled = False
        handled = self.throttle.handle_event(event) or handled
        handled = self.pitch.handle_event(event) or handled
        handled = self.auto_target_checkbox.handle_event(event) or handled
        handled = self.moving_target_checkbox.handle_event(event) or handled
        handled = self.wind_checkbox.handle_event(event) or handled
        return handled

    def handle_key_press(self, event: pygame.event.Event, throttle_step: float, pitch_step: float) -> None:
        throttle_direction = 0
        if _event_matches(event, keys=(pygame.K_w,), chars=("w",)):
            throttle_direction += 1
        elif _event_matches(event, keys=(pygame.K_s,), chars=("s",)):
            throttle_direction -= 1
        self.throttle.step_toward(throttle_direction, throttle_step)

        pitch_direction = 0
        if _event_matches(event, keys=(pygame.K_a,), chars=("a",)):
            pitch_direction -= 1
        elif _event_matches(event, keys=(pygame.K_d,), chars=("d",)):
            pitch_direction += 1
        self.pitch.step_toward(pitch_direction, pitch_step)

    def command_values(self) -> tuple[float, float]:
        return self.throttle.value, self.pitch.value

    def reset(self) -> None:
        self.throttle.set_value(self.throttle.default)
        self.pitch.set_value(self.pitch.default)

    def is_auto_target_enabled(self) -> bool:
        return self.auto_target_checkbox.checked

    def is_target_motion_enabled(self) -> bool:
        return self.moving_target_checkbox.checked

    def is_wind_enabled(self) -> bool:
        return self.wind_checkbox.checked

    def draw(
        self,
        surface: pygame.Surface,
        mode: str = "manual",
        current_command: CommandState | None = None,
    ) -> None:
        pygame.draw.rect(surface, UI_PANEL_COLOR, self.rect)
        pygame.draw.line(surface, UI_BORDER_COLOR, (self.rect.left, self.rect.top), (self.rect.left, self.rect.bottom), 3)

        title = self.title_font.render("Drone Controls", True, TEXT_COLOR)
        surface.blit(title, (self.rect.left + 24, self.rect.top + 28))
        self.auto_target_checkbox.draw(surface, self.help_font)
        self.moving_target_checkbox.draw(surface, self.help_font)
        self.wind_checkbox.draw(surface, self.help_font)

        self.throttle.draw(surface, self.label_font, self.value_font)
        self.pitch.draw(surface, self.label_font, self.value_font)

        if mode.startswith("auto") and current_command is not None:
            self.throttle.draw_indicator(surface, current_command.throttle, TEXT_COLOR)
            self.pitch.draw_indicator(surface, current_command.pitch, TEXT_COLOR)

            auto_text = self.help_font.render(
                f"Auto cmd  thr={current_command.throttle:.2f}  pitch={current_command.pitch:+.2f}",
                True,
                TEXT_COLOR,
            )
            surface.blit(auto_text, (self.rect.left + 24, self.rect.top + 72))

        help_lines = (
            "W/S: throttle per press",
            "A/D: pitch per press",
            "0: manual  1: hold  2: fast  3: intercept",
            "Space: pause    R: reset",
        )
        base_y = self.rect.bottom - 94
        for index, line in enumerate(help_lines):
            text = self.help_font.render(line, True, TEXT_COLOR)
            surface.blit(text, (self.rect.left + 24, base_y + index * 24))
