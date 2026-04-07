import random

from pygame.math import Vector2


def generate_random_target(
    extent_x: float,
    extent_y: float,
    world_width_m: float,
    world_height_m: float,
    upper_fraction: float = 0.4,
    margin_m: float = 0.5,
) -> Vector2:
    min_x = extent_x + margin_m
    max_x = world_width_m - extent_x - margin_m
    min_y = extent_y + margin_m
    max_y = world_height_m * upper_fraction
    return Vector2(
        random.uniform(min_x, max_x),
        random.uniform(min_y, max(max_y, min_y + 0.1)),
    )
