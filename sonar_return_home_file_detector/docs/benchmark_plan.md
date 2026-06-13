# Benchmark Plan

The benchmark target is the complete directory API path:

```text
module initialization -> directory scan -> metadata read -> PCD load -> detection
```

The probe executable records:

```text
state
processed_new_snapshot
home_initialized
returned
scan_id
position_number
point_count
init_ms
scan_dir_ms
pcd_load_ms
detect_ms
total_ms
max_rss_kb
reason
```

## Synthetic Test

Generate sample snapshots:

```bash
python3 src/sonar_return_home_file_detector/scripts/generate_sample_snapshots.py \
  src/sonar_return_home_file_detector/sample_data/generated
```

Replay the snapshots one by one and let the probe record timing:

```bash
python3 src/sonar_return_home_file_detector/scripts/replay_sonar_logs_directory.py \
  --source src/sonar_return_home_file_detector/sample_data/generated \
  --interval-ms 100 \
  --csv return_home_sample_profile.csv
```

This exercises the same file-directory API path as live mapper integration.

## Log Replay

Replay it into a temporary directory and run the probe while files appear:

```bash
python3 src/sonar_return_home_file_detector/scripts/replay_sonar_logs_directory.py \
  --source <maps_dir> \
  --csv return_home_profile.csv
```

The replay script creates standard `*.return_home.yaml` files from file names.
If real odom sidecars are available later, use those instead of filename
fallback odom.
