# sonar_return_home_file_detector

Drop-in return-home detector package for sonar snapshots saved as files.

## Public API

Only one public function is intended for callers:

```cpp
#include <sonar_return_home_file_detector/return_home_file_api.h>

bool returned =
    sonar_return_home_file_detector::detectReturnHomeFromDirectory(snapshot_dir);
```

The first valid snapshot in `snapshot_dir` initializes home and returns `false`.
Later new snapshots run return-home detection. `true` means the module detected
that the robot returned to home.

## Mapper Directory Contract

For the current `multi_position_sonar_mapper` output, pass the log root:

```text
<log_root>/
  sonar_odom_log.csv
  maps/
    phase_multi_submap_1_pos1.pcd
    phase_multi_submap_2_pos2.pcd
```

The detector reads the latest valid CSV row, resolves the matching PCD under
`maps/`, and processes only that new frame.

## Legacy Snapshot Contract

The older sidecar format is still accepted:

```text
<base>.pcd
<base>.return_home.yaml
```

The metadata file must reference the PCD with a relative path and must include
odom. See `docs/snapshot_file_contract.md`.

## Build

Copy this folder into a catkin workspace `src/` directory:

```bash
cd <catkin_ws>
export ROS_SETUP=<path_to_ros_noetic_setup.bash>
source "$ROS_SETUP"
catkin_make -j1
source devel/setup.bash
```

If the target workspace already uses `catkin_tools`, replace the build command
with `catkin build sonar_return_home_file_detector -j1`.

If FFTW3/CGAL/GMP/MPFR/OpenCV/OpenMP are missing, the optional Fourier backend
is disabled at CMake time and the detector fails closed rather than returning
spurious `true` results.

## Probe

```bash
rosrun sonar_return_home_file_detector return_home_directory_probe \
  <snapshot_dir> \
  --reset \
  --watch 100 \
  --poll-ms 100 \
  --csv return_home_profile.csv
```

## Replay Robot Logs

Replay saved logs as if mapper were producing snapshots live:

```bash
rosrun sonar_return_home_file_detector return_home_directory_probe --help

python3 src/sonar_return_home_file_detector/scripts/replay_sonar_logs_directory.py \
  --source <maps_dir> \
  --csv return_home_profile.csv
```
