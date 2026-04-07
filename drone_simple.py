import pygame
from pygame.math import Vector2
from controller import AutopilotController, DroneState, FastAutopilotController, TargetState
from target_utils import generate_random_target
from drone_body import Drone


WIDTH, HEIGHT = 1280, 800
PIXELS_PER_METER = 100.0
WORLD_WIDTH_M = WIDTH / PIXELS_PER_METER
WORLD_HEIGHT_M = HEIGHT / PIXELS_PER_METER

BG_COLOR = (18, 18, 22)
TEXT_COLOR = (230, 230, 230)
DRONE_COLOR = (220, 90, 90)
TARGET_COLOR = (120, 220, 255)
START_COLOR = (120, 255, 170)

GRAVITY_MPS2 = Vector2(0, 9.81)
DRONE_MASS_KG = 1.5
MAX_PITCH_DEG = 70.0
MAX_THRUST_N = DRONE_MASS_KG * GRAVITY_MPS2.y * 4.0
BODY_WIDTH_M = 1.0
BODY_HEIGHT_M = 0.2

START_POS_M = Vector2(WORLD_WIDTH_M * 0.2, WORLD_HEIGHT_M - BODY_HEIGHT_M * 0.5)
TARGET_TOLERANCE_M = 0.25

def world_to_screen(point_m: Vector2) -> tuple[int, int]:
    return round(point_m.x * PIXELS_PER_METER), round(point_m.y * PIXELS_PER_METER)


def draw_markers(surface: pygame.Surface, target_pos_m: Vector2) -> None:
    start_center = world_to_screen(START_POS_M)
    pygame.draw.circle(surface, START_COLOR, start_center, 6)

    target_center = world_to_screen(target_pos_m)
    tolerance_px = max(4, round(TARGET_TOLERANCE_M * PIXELS_PER_METER))
    pygame.draw.circle(surface, TARGET_COLOR, target_center, tolerance_px, width=1)
    pygame.draw.line(surface, TARGET_COLOR, (target_center[0] - 10, target_center[1]), (target_center[0] + 10, target_center[1]), 2)
    pygame.draw.line(surface, TARGET_COLOR, (target_center[0], target_center[1] - 10), (target_center[0], target_center[1] + 10), 2)


def main() -> None:
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Drone Simple")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 30)
    small_font = pygame.font.SysFont(None, 22)

    body = Drone(START_POS_M.copy(), BODY_WIDTH_M, BODY_HEIGHT_M, DRONE_MASS_KG)
    body.angle_deg = body.base_angle_deg
    _, extent_y = body.axis_extents()
    body.pos = Vector2(START_POS_M.x, WORLD_HEIGHT_M - extent_y)
    body.vel = Vector2(0, 0)
    extent_x, extent_y = body.axis_extents()
    target_pos_m = generate_random_target(extent_x, extent_y, WORLD_WIDTH_M, WORLD_HEIGHT_M)
    smooth_controller = AutopilotController()
    fast_controller = FastAutopilotController()
    paused = False
    running = True
    mode = "fast"

    while running:
        dt = clock.tick(60) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    paused = not paused
                elif event.key == pygame.K_r:
                    body.angle_deg = body.base_angle_deg
                    _, extent_y = body.axis_extents()
                    body.pos = Vector2(START_POS_M.x, WORLD_HEIGHT_M - extent_y)
                    body.vel = Vector2(0, 0)
                    target_pos_m = generate_random_target(extent_x, extent_y, WORLD_WIDTH_M, WORLD_HEIGHT_M)
                    smooth_controller.reset()
                    fast_controller.reset()
                elif event.key == pygame.K_1:
                    mode = "smooth"
                    smooth_controller.reset()
                elif event.key == pygame.K_2:
                    mode = "fast"
                    fast_controller.reset()

        state = DroneState(pos_m=body.pos.copy(), vel_mps=body.vel.copy(), angle_deg=body.angle_deg)
        target = TargetState(pos_m=target_pos_m.copy(), tolerance_m=TARGET_TOLERANCE_M, vel_mps=Vector2(0, 0))
        if mode == "smooth":
            controller_output = smooth_controller.compute_command(state, target, dt)
        else:
            controller_output = fast_controller.compute_command(state, target, dt)
        command = controller_output.command

        if not paused:
            body.angle_deg = body.base_angle_deg + command.pitch * MAX_PITCH_DEG
            thrust_force = body.normal * (MAX_THRUST_N * command.throttle)
            acc = GRAVITY_MPS2 + thrust_force / body.mass_kg
            body.vel += acc * dt
            body.pos += body.vel * dt

            extent_x, extent_y = body.axis_extents()
            if body.pos.x - extent_x < 0.0 or body.pos.x + extent_x > WORLD_WIDTH_M or body.pos.y - extent_y < 0.0:
                body.angle_deg = body.base_angle_deg
                _, extent_y = body.axis_extents()
                body.pos = Vector2(START_POS_M.x, WORLD_HEIGHT_M - extent_y)
                body.vel = Vector2(0, 0)
                smooth_controller.reset()
                fast_controller.reset()
            elif body.pos.y + extent_y > WORLD_HEIGHT_M:
                body.pos.y = WORLD_HEIGHT_M - extent_y
                body.vel.y = min(body.vel.y, 0.0)

        distance_to_target = body.pos.distance_to(target_pos_m)
        if mode == "fast":
            reached = distance_to_target <= TARGET_TOLERANCE_M
        else:
            reached = distance_to_target <= TARGET_TOLERANCE_M and body.vel.length() <= 0.25
        if reached and not paused:
            target_pos_m = generate_random_target(extent_x, extent_y, WORLD_WIDTH_M, WORLD_HEIGHT_M)
            smooth_controller.reset()
            fast_controller.reset()
            reached = False

        screen.fill(BG_COLOR)
        draw_markers(screen, target_pos_m)
        body.draw(screen, world_to_screen, DRONE_COLOR)

        status = "PAUSED" if paused else "RUNNING"
        line_1 = font.render("Simple task version: regenerating target, smooth/fast autopilot", True, TEXT_COLOR)
        line_2 = small_font.render(
            f"{status}   mode={mode.upper()}   pos=({body.pos.x:.2f}, {body.pos.y:.2f}) m   vel=({body.vel.x:+.2f}, {body.vel.y:+.2f}) m/s",
            True,
            TEXT_COLOR,
        )
        line_3 = small_font.render(
            f"target=({target_pos_m.x:.2f}, {target_pos_m.y:.2f}) m   dist={distance_to_target:.2f} m   reached={reached}",
            True,
            TEXT_COLOR,
        )
        line_4 = small_font.render(
            f"throttle={command.throttle:.2f}   pitch={command.pitch:+.2f}   1 smooth   2 fast   R reset",
            True,
            TEXT_COLOR,
        )

        screen.blit(line_1, (20, 20))
        screen.blit(line_2, (20, 54))
        screen.blit(line_3, (20, 82))
        screen.blit(line_4, (20, 110))
        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()
