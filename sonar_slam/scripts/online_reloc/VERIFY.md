# Layer A - Online Reloc Stepper: bit-exact verification

**Objective.** Prove that an *online, per-stop-point, stateful* relocalization shell that
reuses the vendored offline kernel (`sonar_slam/scripts/reloc/bbs-1.py`) produces results
**bit-exact** to the sonar-vendor golden `poses_in_map.csv`. Layer A only (produces the
fix / pose per stop point). No Layer B (rviz / dead-reckoning / display).

**Result: PASS.** For all 4 tasks the online stepper is byte-identical to a fresh batch
`relocation-auto.py` run on the same platform (every column, including `final_score`), and
matches the vendor golden exactly on all pose + ICP-quality columns. The only deltas vs the
Windows golden are the *same* `final_score` end-bit differences (max 2.25e-3) that the batch
pipeline itself exhibits vs golden — i.e. the platform epsilon documented in Phase 0, **not**
introduced by the online shell.

---

## Files (deliverable)

- `sonar_slam/scripts/online_reloc/reloc_stepper.py`
  - `RelocStepper` class: `__init__(global_map_path)` seeds RNG once + preprocesses the
    global map once; `step(scan_path, odom_row) -> fix dict` processes exactly one stop
    point and keeps incremental state; `save_poses_csv(out_dir)` writes via the vendored
    writer.
  - `run_task(global_map, maps_dir, points_index_csv, out_dir)` convenience runner
    (Step1 filter + Step2 rad->deg + step loop), returns poses csv + per-point fixes/timing.
  - `assert_vendored_checksums()` aborts unless all 3 vendored files match
    `CHECKSUMS.sha256` (called at init — proves vendored code untouched).
- `sonar_slam/scripts/online_reloc/verify_online_reloc.py`
  - In-container harness: runs the stepper + a fresh batch run for all 4 tasks and does a
    3-way byte comparison (stepper-vs-golden, stepper-vs-batch, batch-vs-golden).

The vendored files in `sonar_slam/scripts/reloc/` were **not modified** (checksum-asserted at
every init). The stepper only imports them via `importlib` (filenames contain a hyphen).

---

## Why it is bit-exact (the RNG contract)

The entire `bbs-1.py` has exactly one source of randomness (verified by grep):
- `np.random.seed(0)` once, in `main()` (bbs-1.py:1954)
- `np.random.choice(...)` in `random_sample_points_xy` (bbs-1.py:290), reached only from
  `preprocess_local_scan` (bbs-1.py:1435) — once per stop point.

Everything else (SOR, DBSCAN concentric-arc filter, voxel downsample, the custom 2D
point-to-line ICP, BBS grid search) is deterministic and does not draw from numpy's RNG.
`relocation-auto.py` has no RNG at all.

`main()` consumes the stream in this order: `seed(0)` -> parse csv (no RNG) ->
`preprocess_global_map` (no RNG) -> for each point in point_id order:
`preprocess_local_scan` (**the one draw**) -> `odom_row_to_T` -> relocalize (no RNG).

The stepper reproduces that order exactly:
- `__init__`: `np.random.seed(0)`; `preprocess_global_map(...)`  (seed consumed once, never re-seeded)
- `step()`: `preprocess_local_scan(...)`; `odom_row_to_T(...)`; first point
  `relocalize_first_scan_global`, later points `predict_pose_from_odom_incremental` (with
  vendored default `ODOM_YAW_DELTA_SIGN = 1.0`) then `relocalize_scan_local`.

Stop points are stepped in **point_id-ascending order** (== batch order, since
`parse_points_index_csv` sorts by point_id) and **nothing draws from `np.random` between two
points**. Global BBS runs **only for the first point**; later points use odom prediction +
local BBS + ICP. Therefore the incremental online stream == the batch stream, and the results
are identical. CSV output reuses the vendored `save_all_poses_csv` verbatim, so the file
format is byte-identical too.

Reused vendored functions: `preprocess_global_map`, `preprocess_local_scan`,
`parse_points_index_csv`, `get_scan_path`, `odom_row_to_T`, `relocalize_first_scan_global`,
`predict_pose_from_odom_incremental`, `relocalize_scan_local`, `params_from_T_2d`,
`save_all_poses_csv`, `OUT_ALL_POSES_CSV`, `ODOM_YAW_DELTA_SIGN` (bbs-1.py); `filter_all_scans`,
`convert_points_index_yaw_to_deg` (relocation-auto.py).

---

## Verification method

Container `pool-reloc:repro38` (open3d 0.19.0 / numpy 1.24.4 / scipy 1.10.1 / pandas 2.0.3 /
py3.8). Mounts: `-v <vendor data>:/deliv  -v sonar_slam/scripts:/scripts  -v <scratch>:/out`.
For each task: run stepper (per-point online), run a fresh `relocation-auto.py` batch, then
compare `poses_in_map.csv` cell-by-cell (csv.reader; line-ending agnostic). Task->global-map
pairings kept strict: `task_z`, `task_l`, `task_20260628_223722(straight2)` ->
`circle-pcd/global.pcd`; `task_20260629_202646` -> `mid_pcd/global.pcd`.

---

## Results (all 4 tasks)

| task | pts | stepper vs GOLDEN (Win) | stepper vs BATCH (same platform) | batch vs GOLDEN |
|---|---|---|---|---|
| task_z | 5 | pose+ICP byte-identical; `final_score` differs 2/5 cells, max **2.250e-3** (ACCEPTABLE) | **byte-identical, 0 diff cells** | `final_score` 2.250e-3 |
| task_l | 7 | pose+ICP byte-identical; `final_score` differs 1/7 cells, max **2.250e-3** (ACCEPTABLE) | **byte-identical, 0 diff cells** | `final_score` 2.250e-3 |
| task_20260628_223722(straight2) | 4 | **all cells identical (0 diff)** | **byte-identical, 0 diff cells** | all cells identical |
| task_20260629_202646 | 9 | **all cells identical (0 diff)** | **byte-identical, 0 diff cells** | all cells identical |

**Decisive check = stepper vs BATCH (same platform): 0 differing cells for every task,
including `final_score`.** This proves the online incremental shell adds *zero* deviation
from the batch kernel. Because `batch vs GOLDEN` shows the identical `final_score` deltas,
those deltas are the Win->Linux platform end-bit (Phase 0), not a shell artifact.

`final_score` is the BBS likelihood diagnostic (`evaluate_icp_candidate` = bbs mean-score +
icp terms); it does not enter the pose. All pose columns (`x_map`, `y_map`, `yaw_map_deg`,
`yaw_map_rad`, `pred_*`) and ICP-quality columns (`icp_fitness`, `icp_rmse`, `icp_corr`,
`icp_success`) are byte-identical vs golden on every task. This exactly reproduces the Phase 0
signature (task_z/task_l: `final_score` 2.25e-3 only; 202646 + 223722(straight2): fully equal).

---

## Timing (per stop point; supports ARM latency planning)

Measured in the repro38 container (x86, single run). First point = global BBS + ICP;
subsequent points = odom prediction + local BBS + ICP.

| task | global-map prep | first point (global BBS) | subsequent points (local), each | subsequent mean |
|---|---|---|---|---|
| task_z (circle) | 1.53 s | 108.7 s | 10.22 / 10.05 / 10.24 / 10.06 s | ~10.1 s |
| task_l (circle) | 1.45 s | 104.6 s | 10.04 / 10.07 / 10.08 / 10.20 / 10.03 / 9.88 s | ~10.0 s |
| straight2 (circle) | 1.44 s | 104.7 s | 10.09 / 10.08 / 10.09 s | ~10.1 s |
| 202646 (mid) | 0.87 s | 94.4 s | 8.88 / 8.68 / 8.85 / 8.72 / 8.87 / 8.67 / 8.79 / 8.81 s | ~8.8 s |

Observations: the first-point global BBS dominates (~95-109 s on x86; expect substantially
more on Orange Pi 5 ARM). Subsequent per-point cost is ~9-10 s and stable — this is exactly
the point of the online design (global BBS once, cheap local corrections thereafter). ARM
timing must be re-measured on the target; global BBS parameters
(`BBS_THETA_STEP_DEG`, `BBS_MAX_LEVEL`, `BBS_MAX_EXPANSIONS_PER_THETA`) are the main knobs if
first-fix latency needs reducing (but changing them changes results and would break bit-exact
equivalence to the vendor golden — keep them for the frozen kernel).

---

## Online NODE (the DEPLOYED path) — separate verification

`reloc_stepper.run_task` (above) filters the WHOLE `maps/` dir in one batch, exactly like the
vendor. The **on-robot node** cannot do that (points arrive one at a time), so it runs a
different code path that must be verified on its own:

- `online_reloc_core.OnlineRelocProcessor.localize(raw_scan, odom_row)` copies ONE raw scan into
  an isolated temp `maps/` and runs the vendored `filter_all_scans` on it alone (-> `scan_1.pcd`),
  then `RelocStepper.step(scan_1, odom_row)`. Per-file filtering carries no cross-scan state, so
  the isolated filter output is byte-identical to the batch's per-file output.
- `read_odom_rows` reuses the vendored `convert_points_index_yaw_to_deg` + `parse_points_index_csv`
  over the whole `points_index.csv`, so the rad->deg(.12f)->deg2rad round-trip matches the batch.
- `online_reloc_node.py` polls the collector task dir, localizes each READY point exactly once in
  ascending point_id order, and publishes `/reloc/fix`.

**Verifier:** `verify_online_node.py` drives `OnlineRelocProcessor` point-by-point over all 4
tasks and compares its `poses_in_map.csv` to golden (decisive) and, with `--with-stepper`, to the
same-platform batch stepper (isolates just the isolated-filter change). It also runs a fast
`gapped-selection` unit test (no BBS) and sanity-checks every `/reloc/fix` payload (7 finite
floats, pose matches the fix). **Result: PASS** — for all 4 pristine tasks the online-node poses
are byte-identical to golden (same `final_score` end-bit only, max 2.25e-3), identical to the
`stepper` verification above. Run:

```
docker run --rm -v "<vendor data>":/deliv -v "<repo>/sonar_slam/scripts":/scripts -v "<scratch>":/out \
  -e RELOC_DIR=/scripts/reloc -e DELIV=/deliv -e OUT=/out -e PYTHONDONTWRITEBYTECODE=1 \
  pool-reloc:repro38 python3 -u /scripts/online_reloc/verify_online_node.py --with-stepper
```

## Equivalence condition — bit-exact-to-golden holds ONLY for a PRISTINE session

**The 4 golden tasks are all pristine** (every point `sweep_complete=true` AND delivered a cloud,
contiguous point_ids). Byte-identity to golden is proven **only** under that condition. On a
**non-pristine** session the online node and the offline batch diverge, by design:

- The batch's `parse_points_index_csv` keeps every parseable row and `filter_all_scans` renumbers
  only the *written* pcds to `scan_1..scan_N`; `bbs-1.py` then pairs each row positionally via
  `scan_{point_id}`. So an **empty `(none)` point** shifts that renumbering and MISPAIRS every
  later point onto the wrong physical scan, and an **aborted `sweep_complete=false`** point is
  still localized by the batch.
- The online node pairs each scan with its OWN point by `deliverable_basename` (the MORE correct
  behavior) and skips `(none)`/aborted points. From the first such point the two results differ.

The node does **NOT** diverge silently: `scan_task` classifies these rows and the node logs a loud
ASCII `NON-PRISTINE point N (...) -> BIT-EXACT-TO-GOLDEN BROKEN from here` (and marks the run
chain-broken). Reasons: `empty` ((none) basename), `aborted` (sweep_complete=false with a pcd),
`lost_cloud` (real basename whose pcd is missing while a LATER point exists -> the cloud was lost
after write-ordering, a batch would mispair). The ONE silent case is a real-basename pcd that is
merely the NEWEST-and-not-yet-present: it is a NOT-YET-READY wait (a batch also skips a missing last
scan, so no divergence). A per-point `localize` step failure is likewise flagged `BIT-EXACT CHAIN
BROKEN`. The `gapped-selection` unit test in `verify_online_node.py` locks the split (ready
`[1,2,4,7]`; nonpristine `(3,empty)+(5,aborted)+(6,lost_cloud)`; pid 8 = newest pcd-missing stays
silent). **Operational rule:** for a run that must reproduce golden, every point must be
`sweep_complete` and non-empty; if the node logs a break, re-collect cleanly.

**Task-dir auto-pick caveat:** with no `~task_dir`, the node latches the newest `task_*` whose
`points_index.csv` mtime is later than node start (the live session), using wall-clock `time.time()`
vs filesystem mtime -- so it never binds a stale prior session, but it also will not pick a session
whose dir was fully written BEFORE the node launched. For any run that matters, pass `~task_dir`
explicitly (the launch supports it); auto-pick is a convenience for "launch node, then collect".

## Robot deployment caveats (ARM / environment) — validate before trusting on-robot

Byte-identity to golden is proven **only in `pool-reloc:repro38` (x86, numpy 1.24.4 / open3d
0.19.0 / scipy 1.10.1)**. On the Orange Pi 5 (aarch64, py3.8) the guarantee weakens unless the
environment matches:

- **open3d 0.19.0 must actually install on aarch64/py3.8** — verify first (`python3 -c "import
  open3d, numpy; print(open3d.__version__, numpy.__version__)"`); if the version differs, results
  may differ in the last bits. The node prints its numpy/open3d versions at startup for this check.
- Different BLAS/OpenMP thread counts can introduce end-bit differences. Pin `OMP_NUM_THREADS` /
  `OPENBLAS_NUM_THREADS` and re-run this verifier in the container at the same values.
- On-robot the honest claim is **same-machine batch equivalence** (online == a batch re-run on the
  robot); byte-identity to the Windows golden holds only for the pose columns and only if the
  numeric stack matches repro38. State this in any on-robot report.
- `reloc/*.py` are vendored VERBATIM as Windows **CRLF** and `CHECKSUMS.sha256` is anchored to
  those CRLF bytes; `.gitattributes` marks `reloc/*.py -text` so they are never normalized. Deliver
  via `git checkout` (not a Windows-normalizing archive) or the checksum guard aborts the node.
- The **frozen vendored kernel prints some Chinese to stdout** (e.g. bbs-1.py `[WARN] 跳过...`),
  which shows as `???` on the robot terminal (no CN font). This is the vendor's code, NOT modified
  here (out of scope); the NODE's own logging is ASCII-only. Do not mistake it for a node bug.

## Pitfalls avoided

- **Feed converted csv, not raw radians.** `parse_points_index_csv` treats `yaw_at_scan` as
  degrees and applies `deg2rad`; feeding the raw radian csv would shrink yaw ~57x. `run_task`
  reuses `convert_points_index_yaw_to_deg` (Step 2) first.
- **Seed once, never between points.** Seeding is done exactly once in `__init__`; no code path
  draws from `np.random` between stop points, so the stream stays aligned with the batch.
- **Reuse the vendored writer** (`save_all_poses_csv`) instead of re-formatting, guaranteeing
  byte-identical CSV.
- **Reuse the vendored filters** (`filter_all_scans`) so filtered scans are byte-identical to
  the batch (DBSCAN determinism confirmed by Phase 0).
- Golden CSVs use CRLF (Windows); the Linux stepper writes LF, so raw byte comparison differs
  by line endings only — the authoritative measure is the cell-wise comparison.

## Reproduce

```
docker run --rm \
  -v "<vendor data root>":/deliv \
  -v "<repo>/sonar_slam/scripts":/scripts \
  -v "<scratch>/out":/out \
  -e RELOC_DIR=/scripts/reloc -e DELIV=/deliv -e OUT=/out \
  pool-reloc:repro38 python3 -u /scripts/online_reloc/verify_online_reloc.py
```
Exit 0 = PASS. Add `--no-batch` to skip the same-platform cross-check (golden-only, faster).
