# UAV Coverage Simulation (PX4 SITL + Gazebo + ROS2 Jazzy)

This repo simulates UAV coverage of a ground grid using PX4 SITL, Gazebo (gz sim 8),
ROS2 Jazzy, and Python nodes. Coverage is computed from UAV pose and a conical FOV.

Repository layout (important paths):
- `submodules/PX4-Autopilot` (PX4 source, built locally)
- `sim/models/quad7` (custom UAV model, based on x500)
- `sim/models/x500_base_7` (custom base with modified mass/inertia)
- `sim/worlds/multi_quad7.sdf` (3-UAV world)
- `ros2_ws/src/coverage_mapper` (coverage node)
- `ros2_ws/src/battery_model` (battery node)
- `tools/` (bridges, waypoint runner, plotters)

---
## Quick start (when everything is already built)

Single UAV:
```
cd ~/uav-coverage-sim
./start.sh
./run_mission.sh --start-current --alt 10 --speed 2
```

Multi UAV (3 drones):
```
cd ~/uav-coverage-sim
./start_multi.sh
./run_multi_mission.sh \
  --uav0 "0,0,10; 40,0,10" \
  --uav1 "0,5,10; 40,5,10" \
  --uav2 "0,-5,10; 40,-5,10" \
  --start-current \
  --speed 2
```

---
## 1) System prerequisites

- Ubuntu + ROS2 Jazzy installed
- Gazebo Sim 8 (gz sim) installed
- `gnome-terminal` available
- Python 3.12

ROS2 install (summary):
```
sudo apt update
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions
source /opt/ros/jazzy/setup.bash
```

---
## 2) PX4 source checkout

```
mkdir -p ~/uav-coverage-sim/submodules
cd ~/uav-coverage-sim/submodules
git clone https://github.com/PX4/PX4-Autopilot.git
```

---
## 3) Python venv for tooling (MAVProxy + PX4 build scripts)

```
cd ~/uav-coverage-sim
python3 -m venv .venv
source .venv/bin/activate
pip install mavproxy pymavlink kconfiglib jinja2 "empy==3.3.4" pyyaml pyros-genmsg
```

These are required for PX4 build and MAVProxy.

---
## 4) PX4 build (SITL)

```
cd ~/uav-coverage-sim/submodules/PX4-Autopilot
make px4_sitl gz_x500
```

If a build error mentions missing Python modules, install them in `.venv`
and rerun the build.

---
## 5) Gazebo models (custom x500_base_7)

PX4 repo does not include Gazebo models. Use `PX4-gazebo-models`.

```
cd ~/uav-coverage-sim
git clone https://github.com/PX4/PX4-gazebo-models.git tmp_models
rm -rf sim/models/x500_base_7
cp -r tmp_models/models/x500_base sim/models/x500_base_7
```

Rename model + fix URIs:
```
sed -i 's/<name>x500_base<\/name>/<name>x500_base_7<\/name>/' sim/models/x500_base_7/model.config
sed -i "s/<model name='x500_base'>/<model name='x500_base_7'>/" sim/models/x500_base_7/model.sdf
sed -i 's/<model name="x500_base">/<model name="x500_base_7">/' sim/models/x500_base_7/model.sdf
sed -i 's/model:\/\/x500_base\//model:\/\/x500_base_7\//g' sim/models/x500_base_7/model.sdf
```

Optional: apply mass/inertia scaling (example):
```
FILE="sim/models/x500_base_7/model.sdf"
sed -i "0,/<mass>2.0<\/mass>/s//<mass>2.2<\/mass>/" "$FILE"
sed -i "0,/<ixx>0.02166666666666667<\/ixx>/s//<ixx>0.03<\/ixx>/" "$FILE"
sed -i "0,/<iyy>0.02166666666666667<\/iyy>/s//<iyy>0.03<\/iyy>/" "$FILE"
sed -i "0,/<izz>0.04000000000000001<\/izz>/s//<izz>0.06<\/izz>/" "$FILE"
```

`sim/models/quad7` already includes `model://x500_base_7` and plugins.

---
## 6) Build ROS2 workspace

```
cd ~/uav-coverage-sim/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
```

---
## 7) Start the sim stack (canonical order)

Always keep MAVProxy running (GCS is required for stable SITL).
The scripts launch everything in the correct order and open terminals.

### Single UAV
```
cd ~/uav-coverage-sim
./start.sh
```

### Multi UAV (3 drones)
```
cd ~/uav-coverage-sim
./start_multi.sh
```

If Gazebo GUI does not appear, launch it manually:
```
GZ_CONFIG_PATH=/usr/share/gz \
GZ_SIM_RESOURCE_PATH=~/uav-coverage-sim/sim/models:~/uav-coverage-sim/sim/worlds:~/uav-coverage-sim/submodules/PX4-Autopilot/Tools/simulation/gz/models:~/uav-coverage-sim/submodules/PX4-Autopilot/Tools/simulation/gz/worlds \
gz sim --force-version 8 -g
```

Warnings about `gz_frame_id` are harmless.

---
## 8) Run missions (coverage)

### Single UAV (coverage integrates over time)
```
cd ~/uav-coverage-sim
./run_mission.sh --start-current --alt 10 --speed 2
```

### Multi UAV (coverage score per tick, sums over drones)
```
cd ~/uav-coverage-sim
./run_multi_mission.sh \
  --uav0 "0,0,10; 40,0,10" \
  --uav1 "0,5,10; 40,5,10" \
  --uav2 "0,-5,10; 40,-5,10" \
  --start-current \
  --speed 2
```

Artifacts:
- `data/coverage/multi_runs/<timestamp>/coverage_score.npy`
- `data/coverage/multi_runs/<timestamp>/coverage_score.png`
- `data/coverage/multi_runs/<timestamp>/waypoints.txt`

---
## 9) Waypoints via text file

`tools/waypoint_runner.py` supports `--waypoints-file`.

File format:
- One waypoint per line: `x,y,z` or `x,y`
- Comma or whitespace separated
- `#` comments allowed

Example file `uav0.txt`:
```
0,0,10
5,10,10
10,-10,10
15,10,10
```

Multi example:
```
./run_multi_mission.sh \
  --uav0-file ~/uav0.txt \
  --uav1-file ~/uav1.txt \
  --uav2-file ~/uav2.txt \
  --start-current \
  --speed 2
```

---
## 10) Notes and tuning

- Pose topics:
  - single: `/uav/pose`
  - multi: `/uav_0/pose`, `/uav_1/pose`, `/uav_2/pose`
- Coverage grid resolution: `ros2_ws/src/coverage_mapper/coverage_mapper/coverage_core.py` (`res`).
- MAVProxy is required for stable PX4 behavior.

Takeoff altitude:
- Controlled by `--alt` in `run_mission.sh` / `run_multi_mission.sh`

Speed:
- `--speed` is m/s and is enforced by velocity setpoints.

---
## 11) Troubleshooting

PX4 build errors (Python deps):
- `kconfiglib`, `jinja2`, `empy==3.3.4`, `pyyaml`, `pyros-genmsg`

MAVProxy not found:
```
source ~/uav-coverage-sim/.venv/bin/activate
pip install mavproxy pymavlink
```

No GUI:
```
GZ_CONFIG_PATH=/usr/share/gz \
GZ_SIM_RESOURCE_PATH=~/uav-coverage-sim/sim/models:~/uav-coverage-sim/sim/worlds:~/uav-coverage-sim/submodules/PX4-Autopilot/Tools/simulation/gz/models:~/uav-coverage-sim/submodules/PX4-Autopilot/Tools/simulation/gz/worlds \
gz sim --force-version 8 -g
```

Old processes interfering:
```
pkill -9 -f gz_pose_to_ros_pose.py
pkill -9 -f "gz sim"
pkill -9 -f px4
pkill -9 -f mavproxy.py
```
