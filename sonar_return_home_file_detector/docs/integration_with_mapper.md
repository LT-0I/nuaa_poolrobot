# Integration With `multi_position_sonar_mapper`

This package is separate from `sonar_slam`; it can be copied directly into:

```text
<catkin_ws>/src/
```

The current mapper saves point cloud files through `saveStageMap()` under:

```text
ros::package::getPath("sonar_slam") + "/maps/"
```

In a normal catkin workspace this resolves to:

```text
<catkin_ws>/src/sonar_slam/maps/
```

Current mapper output:

```text
<base>.pcd
<base>.pgm
<base>.yaml
```

Current limitation: `saveCurrentPhase1Submap()` obtains odom and stores it in
`submap.robot_pose`, but `saveStageMap()` does not write that odom to disk.

## Required Mapper Addition

After each complete scan is saved, the mapper must write:

```text
<base>.return_home.yaml
```

next to the saved PCD. The metadata format is defined in
`snapshot_file_contract.md`.

Recommended write order:

1. Save `<base>.pcd`.
2. Save normal map files if needed.
3. Write `<base>.return_home.yaml.tmp`.
4. Rename it to `<base>.return_home.yaml`.
5. Call:

```cpp
bool returned =
    sonar_return_home_file_detector::detectReturnHomeFromDirectory(maps_dir);
```

The detector returns `false` for the first valid snapshot because it initializes
home. It returns `true` only when a later snapshot is detected as home.

## Note About Position Count Parameters

The mapper command may pass `_phase1/num_positions:=20`, but this source file
currently reads other parameter names in several places. The detector does not
depend on total position count; it only needs:

```text
pointcloud + odom + scan_id/stamp + position_number
```
