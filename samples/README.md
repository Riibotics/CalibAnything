## Samples

This directory stores sample documentation for CalibAnything.

### Available sample package

- `riibotics_mid70_accumulated_success.zip`

This sample archive is distributed separately from the repository because it
contains prepared dataset assets.

Suggested distribution options:

- Google Drive
- GitHub Release asset
- Internal file storage

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

1. Download `riibotics_mid70_accumulated_success.zip` from your external sample location.
2. Extract it under `samples/`.
3. Build the project.
4. Run:

```bash
./bin/run_lidar2camera ./samples/riibotics_mid70_accumulated_success/input/calib.json
```

The executable writes new result images and `extrinsic.txt` into the current
working directory. The files inside the sample archive under `output/` are the
reference results captured for that sample.

### Notes

- The sample is based on the accumulated MID70 workflow.
- Raw rosbag files and raw SAM mask outputs are intentionally omitted.
- This repository does not store the sample archive itself.
