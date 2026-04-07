from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional

import pygame
from pygame.math import Vector2

ForceGenerator = Callable[["PhysicsObject"], Vector2]


@dataclass
class PhysicsObject:
    pos: Vector2
    radius: float
    mass: float = 1.0
    vel: Vector2 = field(default_factory=lambda: Vector2(0, 0))
    linear_damping: float = 0.999
    simulate: bool = True
    accept_global_forces: bool = True
    user_data: dict = field(default_factory=dict)
    local_forces: List[ForceGenerator] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.mass = max(self.mass, 1e-6)
        self.inv_mass = 1.0 / self.mass if self.simulate else 0.0

    def set_simulation_enabled(self, enabled: bool) -> None:
        self.simulate = enabled
        self.inv_mass = 1.0 / self.mass if enabled else 0.0

    def set_accept_global_forces(self, enabled: bool) -> None:
        self.accept_global_forces = enabled

    def add_local_force(self, generator: ForceGenerator) -> None:
        self.local_forces.append(generator)

    def clear_local_forces(self) -> None:
        self.local_forces.clear()


class PhysicsEngine:
    def __init__(
        self,
        bounds: Optional[pygame.Rect] = None,
        restitution: float = 0.7,
        collision_restitution: Optional[float] = None,
    ) -> None:
        self.objects: List[PhysicsObject] = []
        self.bounds = bounds
        self.bound_restitution = restitution
        self.collision_restitution = (
            collision_restitution if collision_restitution is not None else restitution
        )
        self.global_forces: Dict[str, ForceGenerator] = {}

    def set_bounds(self, rect: pygame.Rect, restitution: Optional[float] = None) -> None:
        self.bounds = rect
        if restitution is not None:
            self.bound_restitution = restitution

    def set_global_force(self, name: str, generator: Vector2 | ForceGenerator) -> None:
        self.global_forces[name] = self._coerce_force(generator)

    def remove_global_force(self, name: str) -> None:
        self.global_forces.pop(name, None)

    def add_object(self, obj: PhysicsObject) -> PhysicsObject:
        self.objects.append(obj)
        return obj

    def remove_object(self, obj: PhysicsObject) -> None:
        if obj in self.objects:
            self.objects.remove(obj)

    def step(self, dt: float) -> None:
        if dt <= 0:
            return
        for obj in self.objects:
            if not obj.simulate:
                continue

            acceleration = Vector2(0, 0)
            if obj.accept_global_forces:
                for force in self.global_forces.values():
                    acceleration += force(obj)
            for generator in obj.local_forces:
                acceleration += generator(obj)

            obj.vel += acceleration * dt
            obj.vel *= obj.linear_damping
            obj.pos += obj.vel * dt
            if self.bounds:
                self._resolve_bounds(obj)

        self._resolve_collisions()

    def enforce_bounds(self, obj: PhysicsObject) -> None:
        if self.bounds:
            self._resolve_bounds(obj)

    def _coerce_force(self, generator: Vector2 | ForceGenerator) -> ForceGenerator:
        if callable(generator):
            return generator
        vec = Vector2(generator)

        def wrapper(_: PhysicsObject) -> Vector2:
            return vec

        return wrapper

    def _resolve_bounds(self, obj: PhysicsObject) -> None:
        if not self.bounds:
            return

        bounced = False
        if obj.pos.x - obj.radius < self.bounds.left:
            obj.pos.x = self.bounds.left + obj.radius
            obj.vel.x *= self.bound_restitution
            bounced = True
        if obj.pos.x + obj.radius > self.bounds.right:
            obj.pos.x = self.bounds.right - obj.radius
            obj.vel.x *= self.bound_restitution
            bounced = True
        if obj.pos.y - obj.radius < self.bounds.top:
            obj.pos.y = self.bounds.top + obj.radius
            obj.vel.y *= self.bound_restitution
            bounced = True
        if obj.pos.y + obj.radius > self.bounds.bottom:
            obj.pos.y = self.bounds.bottom - obj.radius
            obj.vel.y *= self.bound_restitution
            bounced = True

        if bounced and not obj.simulate:
            obj.vel.update(0, 0)

    def _resolve_collisions(self) -> None:
        for i in range(len(self.objects)):
            a = self.objects[i]
            for j in range(i + 1, len(self.objects)):
                b = self.objects[j]
                delta = b.pos - a.pos
                dist = delta.length()
                if dist == 0:
                    normal = Vector2(1, 0)
                    dist = 1.0
                else:
                    normal = delta / dist

                min_distance = a.radius + b.radius
                if dist >= min_distance:
                    continue

                penetration = min_distance - dist
                total_inv_mass = a.inv_mass + b.inv_mass
                if total_inv_mass == 0:
                    continue

                correction = normal * (penetration / total_inv_mass)
                if a.simulate:
                    a.pos -= correction * a.inv_mass
                if b.simulate:
                    b.pos += correction * b.inv_mass

                relative_vel = a.vel - b.vel
                vel_along_normal = relative_vel.dot(normal)
                if vel_along_normal > 0:
                    continue

                impulse = -(1 + self.collision_restitution) * vel_along_normal
                impulse /= total_inv_mass
                impulse_vec = normal * impulse

                if a.simulate:
                    a.vel += impulse_vec * a.inv_mass
                if b.simulate:
                    b.vel -= impulse_vec * b.inv_mass
