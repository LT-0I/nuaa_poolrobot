# Snapshot File Contract

The directory input function reads only complete return-home snapshots. A
complete snapshot is:

```text
<base>.pcd
<base>.return_home.yaml
```

The YAML file is the commit marker. Write the PCD first, write metadata to a
temporary file, then rename it into place.

```text
<base>.return_home.yaml.tmp -> <base>.return_home.yaml
```

## Required YAML

```yaml
schema: sonar_return_home_v1
pointcloud: submap_1_pos1_of_20_20260524_153000.pcd
scan_id: 1
position_number: 1
stamp: 1779419227.985
full_scan: true
coverage_deg: 360.0
cloud_frame_id: sonar_frame
odom:
  valid: true
  source: tf
  frame_id: odom
  child_frame_id: base_link
  epoch: 0
  x: 0.0
  y: 0.0
  yaw: 0.0
```

## Rules

- `pointcloud` must be relative to the metadata file directory.
- `odom.x`, `odom.y`, and `odom.yaw` must be in the same odom frame and epoch
  for all snapshots in one run.
- The first valid snapshot read by the detector becomes home.
- Later snapshots with larger `scan_id` are checked against home.
- `total_positions` is intentionally not required by this detector.

