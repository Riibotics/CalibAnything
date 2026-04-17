## Samples

This directory stores sample documentation for CalibAnything.

### Available sample package

#### Processed Data Version
- `riibotics_mid70_accumulated_success.zip`

#### ROSBAG Version
- `rosbag2_2026_04_17-10_29_33_0.db3`

This sample archive is distributed separately from the repository because it
contains prepared dataset assets. Data is located at Synology (Riibotics)

The archive should contain:

- `input/calib.json`
- `input/images/000000.png`
- `input/pc/000000.pcd`
- `input/processed_masks/000000/*.png`
- `output/init_proj.png`
- `output/init_proj_seg.png`
- `output/refined_proj.png`
- `output/refined_proj_seg.png`
- `output/extrinsic.txt`

### How to use the sample

#### Option A — Processed Data Version (`riibotics_mid70_accumulated_success.zip`)

Use this option to verify the C++ calibration binary immediately, without
running SAM or any data preparation scripts.

1. Download `riibotics_mid70_accumulated_success.zip` from Synology (Riibotics).
2. Extract it under `samples/`:
   ```bash
   unzip riibotics_mid70_accumulated_success.zip -d samples/
   ```
3. Build the project if not already built:
   ```bash
   cmake -S . -B build && cmake --build build -j"$(nproc)"
   ```
4. Run the calibration binary directly:
   ```bash
   ./bin/run_lidar2camera ./samples/riibotics_mid70_accumulated_success/input/calib.json
   ```
5. Compare the output files written to the working directory against the
   reference images inside `samples/riibotics_mid70_accumulated_success/output/`:
   - `init_proj.png` / `init_proj_seg.png`
   - `refined_proj.png` / `refined_proj_seg.png`
   - `extrinsic.txt`

---

#### Option B — ROSBAG Version (`rosbag2_2026_04_17-10_29_33_0.db3`)

Use this option to run the full end-to-end pipeline starting from a raw bag.

1. Download `rosbag2_2026_04_17-10_29_33_0.db3` from Synology (Riibotics) and
   place it inside `data/`:
   ```
   CalibAnything/
   └── data/
       └── rosbag2_2026_04_17-10_29_33_0.db3
   ```

2. Extract the dataset from the bag:
   ```bash
   python3 data_processing/prepare_data.py \
     --bag-path ./data/rosbag2_2026_04_17-10_29_33_0.db3 \
     --lidar-topic /fork_lidar \
     --image-topic /fork_camera/rgb \
     --cam-info-topic /fork_camera/rgb_info \
     --output-dir ./dataset1
   ```
   This creates `dataset1/images/000000.png`, `dataset1/pc/000000.pcd`, and
   `dataset1/calib.json` with the recommended default parameters.

3. Review `dataset1/calib.json` and verify that `T_lidar_to_cam` reflects your
   actual sensor mounting. Adjust `params` if needed (see the Parameter
   Descriptions table in the top-level README).

4. Run the full pipeline (SAM mask generation → mask post-processing → build →
   calibration):
   ```bash
   DATA_DIR=$(pwd)/dataset1 ./run_pipeline.sh
   ```

5. Inspect the output files written to the repository root:
   - `init_proj.png` — projection before optimisation
   - `refined_proj.png` — projection after optimisation
   - `extrinsic.txt` — estimated LiDAR-to-camera extrinsic matrix

> **Tip** If `refined_proj.png` looks poorly aligned, check the
> [When Calibration Quality Is Poor](../README.md#when-calibration-quality-is-poor)
> section in the top-level README.

### Notes

- The processed-data sample is based on the accumulated MID70 workflow.
- Raw rosbag files and raw SAM mask outputs are intentionally omitted from the
  processed-data archive.
- This repository does not store either sample file; both are distributed via
  Synology (Riibotics).
