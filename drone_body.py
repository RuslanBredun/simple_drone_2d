import math

import pygame
from pygame.math import Vector2


class Drone:
    def __init__(
        self,
        pos: Vector2,
        width_m: float,
        height_m: float,
        mass_kg: float,
        base_angle_deg: float = -90.0,
    ) -> None:
        self.pos = pos
        self.vel = Vector2(0, 0)
        self.width_m = width_m
        self.height_m = height_m
        self.mass_kg = mass_kg
        self.base_angle_deg = base_angle_deg
        self.angle_deg = self.base_angle_deg

    @property
    def angle_rad(self) -> float:
        return math.radians(self.angle_deg)

    @property
    def normal(self) -> Vector2:
        return Vector2(math.cos(self.angle_rad), math.sin(self.angle_rad))

    @property
    def tangent(self) -> Vector2:
        n = self.normal
        return Vector2(-n.y, n.x)

    def axis_extents(self) -> tuple[float, float]:
        half_w = self.width_m * 0.5
        half_h = self.height_m * 0.5
        tang = self.tangent
        norm = self.normal
        extent_x = abs(tang.x) * half_w + abs(norm.x) * half_h
        extent_y = abs(tang.y) * half_w + abs(norm.y) * half_h
        return extent_x, extent_y

    def draw(
        self,
        surface: pygame.Surface,
        world_to_screen: callable,
        color: tuple[int, int, int],
    ) -> None:
        half_w = self.width_m * 0.5
        half_h = self.height_m * 0.5
        tang = self.tangent
        norm = self.normal
        corners = [
            self.pos + tang * half_w + norm * half_h,
            self.pos - tang * half_w + norm * half_h,
            self.pos - tang * half_w - norm * half_h,
            self.pos + tang * half_w - norm * half_h,
        ]
        pygame.draw.polygon(surface, color, [world_to_screen(corner) for corner in corners])
