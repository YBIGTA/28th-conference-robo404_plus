#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# Pipeline to convert PyTorch (.pt) weights to ONNX and TensorRT (.engine) format.
# 
# Usage:
#   ./convert_model.sh <path_to_model.pt> [output_directory]
# 
# Example:
#   ./convert_model.sh ~/Downloads/best_model_12.pt ~/models
# -----------------------------------------------------------------------------

set -euo pipefail

PT_PATH="${1:?Usage: $0 <path_to_model.pt> [output_directory]}"
OUT_DIR="${2:-$(dirname "$PT_PATH")}"

mkdir -p "$OUT_DIR"

echo "=================================================="
echo " Starting YOLO Weights Conversion Pipeline"
echo " Input PT:      $PT_PATH"
echo " Output Dir:    $OUT_DIR"
echo "=================================================="

# 1. Validate input file
if [ ! -f "$PT_PATH" ]; then
    echo "ERROR: Input file '$PT_PATH' does not exist."
    exit 1
fi

# 2. Check for python environment
echo "Checking Python environment..."
export PYTHONPATH="$HOME/.local/lib/python3.8/site-packages:${PYTHONPATH:-}"

# Get the base name of the model
MODEL_BASE=$(basename "$PT_PATH" .pt)
ONNX_PATH="${OUT_DIR}/${MODEL_BASE}.onnx"
ENGINE_PATH="${OUT_DIR}/${MODEL_BASE}.engine"

# 3. Step 1: Export PT to ONNX
echo ""
echo "--- Step 1: Exporting PyTorch (.pt) to ONNX ---"
python3 "$(dirname "$0")/convert_yolo_to_onnx.py" "$PT_PATH" --output-dir "$OUT_DIR" --opset 11

if [ ! -f "$ONNX_PATH" ]; then
    echo "ERROR: ONNX file was not generated."
    exit 1
fi

echo "ONNX model generated at: $ONNX_PATH"

# 4. Step 2: Export ONNX to TensorRT (.engine)
echo ""
echo "--- Step 2: Converting ONNX to TensorRT (.engine) ---"

# Find trtexec path
TRTEXEC_BIN=""
if command -v trtexec &>/dev/null; then
    TRTEXEC_BIN="trtexec"
elif [ -f "/usr/src/tensorrt/bin/trtexec" ]; then
    TRTEXEC_BIN="/usr/src/tensorrt/bin/trtexec"
elif [ -f "/usr/local/cuda/bin/trtexec" ]; then
    TRTEXEC_BIN="/usr/local/cuda/bin/trtexec"
else
    # Let's search common paths
    for path in "/usr/src/tensorrt/bin/trtexec" "/usr/local/cuda/bin/trtexec" "/usr/lib/tensorrt/bin/trtexec"; do
        if [ -f "$path" ]; then
            TRTEXEC_BIN="$path"
            break
        fi
    done
fi

if [ -n "$TRTEXEC_BIN" ]; then
    echo "Found trtexec at: $TRTEXEC_BIN"
    echo "Running trtexec conversion (FP16 mode)..."
    
    # Run trtexec
    "$TRTEXEC_BIN" \
        --onnx="$ONNX_PATH" \
        --saveEngine="$ENGINE_PATH" \
        --fp16
        
    if [ -f "$ENGINE_PATH" ]; then
        echo ""
        echo "=================================================="
        echo " SUCCESS! Model converted to TensorRT Engine."
        echo " Engine path: $ENGINE_PATH"
        echo "=================================================="
    else
        echo "ERROR: trtexec run finished, but engine file was not created."
        exit 1
    fi
else
    echo "WARNING: 'trtexec' not found on this system."
    echo "This is expected if you are not running on the Jetson Nano."
    echo ""
    echo "=================================================="
    echo " ONNX EXPORT SUCCESSFUL!"
    echo " ONNX file: $ONNX_PATH"
    echo ""
    echo "To generate the TensorRT (.engine) file, copy the ONNX file to your"
    echo "Jetson Nano and run the following command:"
    echo ""
    echo "  /usr/src/tensorrt/bin/trtexec \\"
    echo "    --onnx=$ONNX_PATH \\"
    echo "    --saveEngine=$ENGINE_PATH \\"
    echo "    --fp16"
    echo "=================================================="
fi
