# Drone Test Environment

A compact Python solution for the 2D drone test task. The active runtime lives in `drone.py`, with controller logic in `controller.py` and UI support in `ui.py`.

## Requirements
- Python 3.12 (system package `python3.12` on Ubuntu 24.04)
- `python3.12-venv` to create isolated environments
- Optional: SDL libraries from your distro if you intend to build pygame from source. The pip wheel used below already bundles the necessary binaries.

## Setup
1. Clone or copy this directory onto your machine and move into it: `cd drone_test`.
2. Create a virtual environment: `python3 -m venv .venv`.
3. Activate it: `source .venv/bin/activate` (PowerShell: `.venv\\Scripts\\Activate.ps1`).
4. Upgrade pip if desired: `python -m pip install --upgrade pip`.
5. Install Python dependencies: `pip install -r requirements.txt`.

> The included `.venv` directory is local-only; recreate it on new machines rather than copying it. The pinned requirement (`pygame==2.6.1`) will pull the same wheel.

## Running the simulation
1. Ensure the environment is activated (`source .venv/bin/activate`).
2. Launch the script: `python drone.py`.
3. Controls:
   - `W` / `S`: increase or decrease the throttle slider one step per key press
   - `A` / `D`: move the pitch slider one step per key press
   - `0`: switch to `manual`
   - `1`: switch to `auto_smooth`
   - `2`: switch to `auto_fast`
   - `3`: switch to `auto_intercept`
   - Checkbox `New target on reach`: enable or disable automatic target regeneration after success; enabled by default
   - Checkbox `Enable wind`: enable or disable a smooth horizontal wind disturbance
   - Mouse drag on either slider also adjusts the command directly
   - `Space`: pause/resume integration
   - `R`: reset state
   - `Esc`: exit

The script opens a `1600x900` window at ~60 Hz. The left side is the flight area and the right side is a dedicated control panel. The window is still pixel-based, but the drone state is simulated in SI-style units and rendered through a pixels-per-meter scale from `constants.py`.

## Minimal task version
- `python drone_simple.py` runs a stripped-down single-file version of the assignment.
- It keeps only the minimum needed for the task: 2D drone, gravity, thrust, absolute pitch command, regenerating target points, tolerance check, and compact `smooth`/`fast` autopilot modes.
- In `drone_simple.py`, a new random target is generated automatically after each successful reach.
- It does not include sliders, wind, moving targets, multiple autopilot modes, or extra debug visualizations.

## Runtime behavior
- Only the red **1.0×0.2 m rectangle** (the “main” chassis) is simulated.
- The body is treated as a simple 2D drone. Pitch is an absolute attitude command around the neutral body angle, and throttle is a persistent thrust command.
- On each application launch, the target is generated randomly in the upper part of the world.
- `New target on reach` is enabled by default. While enabled, reaching the target immediately generates a new random target in the upper part of the world.
- The target now moves continuously inside the upper part of the world with a smoothly changing random velocity, and its current velocity is shown by an arrow near the target marker.
- `Enable wind` adds a smooth random horizontal disturbance with magnitude varying between `0.5` and `1.5 m/s^2`, and direction drifting within `±15°` from the horizontal axis.
- `A`/`D` adjust pitch in small snapped steps per press, `W`/`S` adjust throttle in small snapped steps per press, and the right-side panel visualizes both.
- Both sliders snap cleanly back to `0.0`, so neutral command is reachable from the keyboard without fighting floating-point drift.
- `manual` mode uses the sliders directly.
- `auto_smooth` uses a smooth PID-style controller that tries to reach and hold the target.
- `auto_fast` uses an aggressive PID-based hit controller that tries to reach the target as fast as possible; terminal speed is not part of the success criterion.
- `auto_intercept` uses target velocity to aim ahead of the moving target and intercept it instead of chasing only the current target position.
- Physics state uses meters, meters per second, kilograms, Newtons, and `m/s^2`. The simulation uses an explicit drone mass and computes thrust acceleration as `F / m`.
- The active plant now uses only gravity and thrust as forces. The earlier linear damping term has been removed to stay closer to the original task statement.
- The world uses screen-style axes for convenience: `x` grows to the right and `y` grows downward, even though the units themselves are SI.
- `R` resets the drone to the start point for the current target. Restarting the application generates a new target.
- Contact with the ground clamps the body in place without bounce.
- Contact with the left wall, right wall, or top boundary resets the body to its start state.
- No other simulated bodies or auxiliary physics objects exist.

## Support modules
- `constants.py` centralizes window sizing, palette values, and physics coefficients so the main script stays lean.
- `controller.py` defines the autopilot/controller data contracts plus both auto controllers:
  smooth hold-oriented, fast reach-oriented, and intercept-oriented.
- `ui.py` owns the reusable slider widgets and the control panel rendering/input logic.
- `physics.py` remains available for future experiments but is not used by the current single-body demo.

## Key physical parameters
- `DRONE_MASS_KG`: body mass in kilograms
- `GRAVITY_MPS2`: gravity vector in `m/s^2`
- `MAX_THRUST_TO_WEIGHT`: thrust-to-weight ratio at full throttle
- `MAX_THRUST_N`: maximum thrust force at `throttle = 1.0`, derived as `mass * g * thrust_to_weight`
- `PIXELS_PER_METER`: rendering scale between the SI world and the window

## Troubleshooting
- If `python3 -m venv` fails complaining about `ensurepip`, install `python3.12-venv` and retry.
- When running inside remote/virtualized environments without an available display, set `SDL_VIDEODRIVER=dummy` before launching if you only need offscreen integration.
