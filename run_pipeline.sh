#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

CALIB_ANYTHING_DIR="${SCRIPT_DIR}"
DATA_DIR="${DATA_DIR:-${CALIB_ANYTHING_DIR}/dataset1}"
CALIB_JSON="${CALIB_JSON:-${DATA_DIR}/calib.json}"
SAM_DIR="${SAM_DIR:-${CALIB_ANYTHING_DIR}/../segment-anything}"
BUILD_DIR="${BUILD_DIR:-${CALIB_ANYTHING_DIR}/build}"

SAM_CHECKPOINT_NAME="${SAM_CHECKPOINT_NAME:-sam_vit_l_0b3195.pth}"
SAM_CHECKPOINT_URL="${SAM_CHECKPOINT_URL:-https://dl.fbaipublicfiles.com/segment_anything/sam_vit_l_0b3195.pth}"
SAM_CHECKPOINT_PATH="${SAM_DIR}/${SAM_CHECKPOINT_NAME}"

require_dir() {
    local target="$1"
    if [ ! -d "$target" ]; then
        echo "Required directory does not exist: $target" >&2
        exit 1
    fi
}

require_file() {
    local target="$1"
    if [ ! -f "$target" ]; then
        echo "Required file does not exist: $target" >&2
        exit 1
    fi
}

echo "=================================================="
echo "1. Validating dataset and repository paths..."
echo "=================================================="
require_dir "$CALIB_ANYTHING_DIR"
require_dir "$DATA_DIR"
require_dir "$DATA_DIR/images"
require_file "$CALIB_JSON"

echo "=================================================="
echo "2. Preparing SAM (Segment Anything) environment and model..."
echo "=================================================="
if [ ! -d "$SAM_DIR" ]; then
    echo "SAM repository not found. Cloning from GitHub..."
    git clone https://github.com/facebookresearch/segment-anything.git "$SAM_DIR"
    python3 -m pip install -e "$SAM_DIR"
    python3 -m pip install opencv-python pycocotools matplotlib onnxruntime onnx
else
    echo "SAM repository already exists."
fi

require_file "$SAM_DIR/scripts/amg.py"

if [ ! -f "$SAM_CHECKPOINT_PATH" ]; then
    echo "SAM model checkpoint (${SAM_CHECKPOINT_NAME}) not found. Downloading..."
    wget -O "$SAM_CHECKPOINT_PATH" "$SAM_CHECKPOINT_URL"
else
    echo "SAM model checkpoint file already exists."
fi

echo "=================================================="
echo "3. Generating image masks automatically using SAM..."
echo "=================================================="
rm -rf "$DATA_DIR/masks"
mkdir -p "$DATA_DIR/masks"

cd "$SAM_DIR"
python3 scripts/amg.py --checkpoint "$SAM_CHECKPOINT_PATH" --model-type vit_l \
    --input "$DATA_DIR/images/" --output "$DATA_DIR/masks/" \
    --stability-score-thresh 0.9 --box-nms-thresh 0.5 --stability-score-offset 0.9

echo "=================================================="
echo "4. Post-processing masks to match CalibAnything format..."
echo "=================================================="
cd "$CALIB_ANYTHING_DIR"
rm -rf "$DATA_DIR/processed_masks"
python3 processed_mask.py -i "$DATA_DIR/masks/" -o "$DATA_DIR/processed_masks/"

echo "=================================================="
echo "5. Building CalibAnything..."
echo "=================================================="
cmake -S "$CALIB_ANYTHING_DIR" -B "$BUILD_DIR"
cmake --build "$BUILD_DIR" -j"$(nproc)"

echo "=================================================="
echo "6. Running CalibAnything calibration algorithm..."
echo "=================================================="
./bin/run_lidar2camera "$CALIB_JSON"

echo "=================================================="
echo "Calibration pipeline completed."
echo "Check ${CALIB_ANYTHING_DIR}/refined_proj.png and ${CALIB_ANYTHING_DIR}/extrinsic.txt."
echo "=================================================="
