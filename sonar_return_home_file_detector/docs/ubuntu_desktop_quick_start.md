# Ubuntu Desktop Quick Start

1. Copy the package into a catkin workspace:

```bash
cp -r sonar_return_home_file_detector <catkin_ws>/src/
```

2. Install dependencies:

```bash
sudo apt update
sudo apt install ros-noetic-roscpp ros-noetic-pcl-ros ros-noetic-pcl-conversions \
  libpcl-dev libeigen3-dev libfftw3-dev libcgal-dev libopencv-dev libgmp-dev libmpfr-dev
```

3. Build:

```bash
cd <catkin_ws>
export ROS_SETUP=<path_to_ros_noetic_setup.bash>
source "$ROS_SETUP"
catkin_make -j1
source devel/setup.bash
```

For a workspace that already uses `catkin_tools`, use this build command
instead:

```bash
catkin build sonar_return_home_file_detector -j1
```

4. Smoke test:

```bash
python3 src/sonar_return_home_file_detector/scripts/generate_sample_snapshots.py \
  src/sonar_return_home_file_detector/sample_data/generated
python3 src/sonar_return_home_file_detector/scripts/replay_sonar_logs_directory.py \
  --source src/sonar_return_home_file_detector/sample_data/generated \
  --interval-ms 100 --csv return_home_profile.csv
```

5. Replay real saved scans:

```bash
python3 src/sonar_return_home_file_detector/scripts/replay_sonar_logs_directory.py \
  --source <maps_dir> --csv return_home_profile.csv
```
