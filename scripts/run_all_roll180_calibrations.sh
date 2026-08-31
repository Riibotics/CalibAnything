#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)

DATASET_ROOT="${DATASET_ROOT:-${REPO_DIR}/dataset}"
PREPARED_ROOT="${PREPARED_ROOT:-${REPO_DIR}/dataset_prepared}"
RESULTS_ROOT="${RESULTS_ROOT:-${REPO_DIR}/calibration_results}"
INITIAL_TRANSFORM_PRESET="${INITIAL_TRANSFORM_PRESET:-ros_lidar_to_opencv_roll_180}"

if [ ! -d "${DATASET_ROOT}" ]; then
    printf 'Dataset root does not exist: %s\n' "${DATASET_ROOT}" >&2
    exit 1
fi

run_started_at=$(date '+%Y%m%d_%H%M%S')

for bag_dir in "${DATASET_ROOT}"/multimodal_calib_*; do
    if [ ! -d "${bag_dir}" ]; then
        continue
    fi

    dataset_name=$(basename "${bag_dir}")
    printf '\n==================================================\n'
    printf 'Running calibration for %s\n' "${dataset_name}"
    printf '==================================================\n'

    BAG_DIR="${bag_dir}" \
    OUT_DIR="${PREPARED_ROOT}/${dataset_name}" \
    RESULTS_ROOT="${RESULTS_ROOT}" \
    DATASET_NAME="${dataset_name}" \
    RUN_NAME="${run_started_at}_${INITIAL_TRANSFORM_PRESET}" \
    INITIAL_TRANSFORM_PRESET="${INITIAL_TRANSFORM_PRESET}" \
    bash "${REPO_DIR}/scripts/run_roll180_calibration.sh"
done

printf '\nAll dataset calibrations completed.\n'
printf 'Result root: %s\n' "${RESULTS_ROOT}"
