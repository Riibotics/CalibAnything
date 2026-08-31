#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)

cd "${REPO_DIR}"

ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
BAG_DIR="${BAG_DIR:-${REPO_DIR}/dataset/multimodal_calib_20260708_093812}"
OUT_DIR="${OUT_DIR:-${REPO_DIR}/dataset_prepared/multimodal_calib_20260708_093812}"
DATASET_NAME="${DATASET_NAME:-$(basename "${BAG_DIR}")}"
SAM_DIR="${SAM_DIR:-${REPO_DIR}/../segment-anything}"
DEPS_DIR="${DEPS_DIR:-${REPO_DIR}/.python_deps}"
BUILD_DIR="${BUILD_DIR:-${REPO_DIR}/build}"
RESULTS_ROOT="${RESULTS_ROOT:-${REPO_DIR}/calibration_results}"
INITIAL_TRANSFORM_PRESET="${INITIAL_TRANSFORM_PRESET:-ros_lidar_to_opencv_roll_180}"
RUN_NAME="${RUN_NAME:-$(date '+%Y%m%d_%H%M%S')_${INITIAL_TRANSFORM_PRESET}}"
RESULT_DIR="${RESULT_DIR:-${RESULTS_ROOT}/${DATASET_NAME}/${RUN_NAME}}"
SAM_DEVICE="${SAM_DEVICE:-cpu}"
SAM_CHECKPOINT_NAME="${SAM_CHECKPOINT_NAME:-sam_vit_l_0b3195.pth}"
SAM_CHECKPOINT_URL="${SAM_CHECKPOINT_URL:-https://dl.fbaipublicfiles.com/segment_anything/sam_vit_l_0b3195.pth}"
SAM_CHECKPOINT_PATH="${SAM_DIR}/${SAM_CHECKPOINT_NAME}"

log() {
    printf '\n[%s] %s\n' "$(date '+%H:%M:%S')" "$*"
}

require_file() {
    if [ ! -f "$1" ]; then
        printf 'Required file does not exist: %s\n' "$1" >&2
        exit 1
    fi
}

require_dir() {
    if [ ! -d "$1" ]; then
        printf 'Required directory does not exist: %s\n' "$1" >&2
        exit 1
    fi
}

log "Checking ROS environment"
require_file "${ROS_SETUP}"
# shellcheck source=/dev/null
source "${ROS_SETUP}"

require_dir "${BAG_DIR}"
mkdir -p "${RESULT_DIR}/config" "${RESULT_DIR}/input" "${RESULT_DIR}/output" "${RESULT_DIR}/logs"

log "Preparing Segment Anything"
if [ ! -d "${SAM_DIR}" ]; then
    git clone https://github.com/facebookresearch/segment-anything.git "${SAM_DIR}"
fi

if [ ! -d "${DEPS_DIR}/torch" ] || [ ! -d "${DEPS_DIR}/torchvision" ]; then
    python3 -m pip install --target "${DEPS_DIR}" \
        --index-url https://download.pytorch.org/whl/cpu \
        torch torchvision
fi

# Avoid shadowing the system NumPy used by the system OpenCV package.
rm -rf "${DEPS_DIR}"/numpy "${DEPS_DIR}"/numpy-*.dist-info "${DEPS_DIR}"/numpy.libs

if [ ! -f "${SAM_CHECKPOINT_PATH}" ]; then
    wget -O "${SAM_CHECKPOINT_PATH}" "${SAM_CHECKPOINT_URL}"
fi

log "Extracting ROS bag into CalibAnything dataset"
python3 data_processing/prepare_data.py \
    --bag-path "${BAG_DIR}" \
    --lidar-topic /fork_lidar \
    --image-topic /fork_camera/rgb \
    --cam-info-topic /fork_camera/rgb_info \
    --output-dir "${OUT_DIR}" \
    --initial-transform-preset "${INITIAL_TRANSFORM_PRESET}"

log "Generating SAM masks"
rm -rf "${OUT_DIR}/masks"
mkdir -p "${OUT_DIR}/masks"

PYTHONPATH="${DEPS_DIR}:${SAM_DIR}:${PYTHONPATH:-}" \
python3 "${SAM_DIR}/scripts/amg.py" \
    --checkpoint "${SAM_CHECKPOINT_PATH}" \
    --model-type vit_l \
    --device "${SAM_DEVICE}" \
    --input "${OUT_DIR}/images/" \
    --output "${OUT_DIR}/masks/" \
    --stability-score-thresh 0.9 \
    --box-nms-thresh 0.5 \
    --stability-score-offset 0.9

log "Post-processing masks"
rm -rf "${OUT_DIR}/processed_masks"
python3 processed_mask.py \
    -i "${OUT_DIR}/masks/" \
    -o "${OUT_DIR}/processed_masks/"

log "Building CalibAnything"
cmake -S "${REPO_DIR}" -B "${BUILD_DIR}"
cmake --build "${BUILD_DIR}" -j"$(nproc)"

log "Running LiDAR-camera calibration"
"${REPO_DIR}/bin/run_lidar2camera" "${OUT_DIR}/calib.json" \
    2>&1 | tee "${RESULT_DIR}/logs/calibration.log"

log "Archiving calibration result"
require_file "${REPO_DIR}/extrinsic.txt"
require_file "${REPO_DIR}/init_proj.png"
require_file "${REPO_DIR}/init_proj_seg.png"
require_file "${REPO_DIR}/refined_proj.png"
require_file "${REPO_DIR}/refined_proj_seg.png"
require_file "${OUT_DIR}/calib.json"
require_file "${OUT_DIR}/images/000000.png"
require_file "${OUT_DIR}/pc/000000.pcd"

cp "${OUT_DIR}/calib.json" "${RESULT_DIR}/config/calib.json"
cp "${OUT_DIR}/images/000000.png" "${RESULT_DIR}/input/000000.png"
cp "${OUT_DIR}/pc/000000.pcd" "${RESULT_DIR}/input/000000.pcd"
cp "${REPO_DIR}/extrinsic.txt" "${RESULT_DIR}/output/extrinsic.txt"
cp "${REPO_DIR}/init_proj.png" "${RESULT_DIR}/output/init_proj.png"
cp "${REPO_DIR}/init_proj_seg.png" "${RESULT_DIR}/output/init_proj_seg.png"
cp "${REPO_DIR}/refined_proj.png" "${RESULT_DIR}/output/refined_proj.png"
cp "${REPO_DIR}/refined_proj_seg.png" "${RESULT_DIR}/output/refined_proj_seg.png"

SUMMARY_PATH="${RESULT_DIR}/summary.json" \
BAG_DIR="${BAG_DIR}" \
OUT_DIR="${OUT_DIR}" \
DATASET_NAME="${DATASET_NAME}" \
INITIAL_TRANSFORM_PRESET="${INITIAL_TRANSFORM_PRESET}" \
SAM_DEVICE="${SAM_DEVICE}" \
python3 - <<'PY'
import json
import os
from datetime import datetime, timezone
from pathlib import Path

summary_path = Path(os.environ["SUMMARY_PATH"])
summary = {
    "created_at_utc": datetime.now(timezone.utc).isoformat(),
    "dataset_name": os.environ["DATASET_NAME"],
    "bag_dir": os.environ["BAG_DIR"],
    "prepared_dataset_dir": os.environ["OUT_DIR"],
    "initial_transform_preset": os.environ["INITIAL_TRANSFORM_PRESET"],
    "sam_device": os.environ["SAM_DEVICE"],
    "artifacts": {
        "config": "config/calib.json",
        "input_image": "input/000000.png",
        "input_pcd": "input/000000.pcd",
        "extrinsic": "output/extrinsic.txt",
        "init_projection": "output/init_proj.png",
        "init_segment_projection": "output/init_proj_seg.png",
        "refined_projection": "output/refined_proj.png",
        "refined_segment_projection": "output/refined_proj_seg.png",
        "calibration_log": "logs/calibration.log",
    },
}
summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
PY

log "Done"
printf 'Check:\n'
printf '  %s/output/init_proj_seg.png\n' "${RESULT_DIR}"
printf '  %s/output/refined_proj_seg.png\n' "${RESULT_DIR}"
printf '  %s/output/extrinsic.txt\n' "${RESULT_DIR}"
printf '  %s/summary.json\n' "${RESULT_DIR}"
