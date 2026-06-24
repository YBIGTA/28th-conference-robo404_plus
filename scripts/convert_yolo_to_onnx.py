#!/usr/bin/env python3
"""
General pipeline to convert PyTorch (.pt) weights to ONNX.
Supports both Ultralytics YOLOv8/v5 models and Darknet PyTorch (YOLOv3/v3-SPP) models.
"""

import os
import sys
import argparse
import torch

def parse_args():
    parser = argparse.ArgumentParser(description="Convert YOLO .pt weights to ONNX format")
    parser.add_argument("pt_path", type=str, help="Path to input .pt weights file")
    parser.add_argument("--output-dir", type=str, default=None, help="Directory to save the output ONNX file")
    parser.add_argument("--imgsz", type=int, default=608, help="Inference image size (default: 608)")
    parser.add_argument("--opset", type=int, default=11, help="ONNX opset version (default: 11)")
    return parser.parse_args()

def convert_ultralytics(pt_path, output_dir, imgsz, opset):
    print("Detected Ultralytics checkpoint format. Exporting using Ultralytics...")
    try:
        from ultralytics import YOLO
    except ImportError:
        print("ERROR: 'ultralytics' package is not installed. Please install it using 'pip install ultralytics'.")
        sys.exit(1)
        
    model = YOLO(pt_path)
    # Export to ONNX
    onnx_path = model.export(format="onnx", imgsz=imgsz, opset=opset)
    print(f"ONNX model saved successfully to: {onnx_path}")
    
    # Move to output directory if specified
    if output_dir and os.path.dirname(onnx_path) != os.path.abspath(output_dir):
        dest = os.path.join(output_dir, os.path.basename(onnx_path))
        os.rename(onnx_path, dest)
        print(f"Moved ONNX model to: {dest}")

def convert_darknet(pt_path, output_dir, imgsz, opset, sd):
    print("Detected Darknet PyTorch checkpoint format. Exporting using PyTorch-YOLOv3...")
    
    # 1. Detect architecture and classes from state dict
    sd_indices = sorted(list(set(int(k.split('.')[1]) for k in sd.keys() if k.startswith('module_list.'))))
    max_idx = max(sd_indices) if sd_indices else 0
    print(f"Detected max module index: {max_idx}")
    
    if max_idx == 112:
        model_type = "yolov3-spp"
    elif max_idx == 107:
        model_type = "yolov3"
    elif max_idx == 32:
        model_type = "yolov3-tiny"
    else:
        print(f"Unknown Darknet layer count (max index: {max_idx}). Defaulting to yolov3-spp config structure.")
        model_type = "yolov3-spp"
        
    print(f"Inferred model architecture: {model_type}")
    
    # Guess number of classes from last conv output size
    # In YOLOv3, the last layer is module_list.[max_idx].Conv2d.weight with shape [3 * (classes + 5), ...]
    last_conv_key = f"module_list.{max_idx}.Conv2d.weight"
    if last_conv_key not in sd:
        last_conv_key = f"module_list.{max_idx}.conv_{max_idx}.weight"
        
    if last_conv_key in sd:
        out_channels = sd[last_conv_key].shape[0]
        classes = int(out_channels / 3 - 5)
        print(f"Detected output channels: {out_channels} -> Inferred classes: {classes}")
    else:
        classes = 6
        print(f"Could not find last conv layer in state dict. Defaulting to classes={classes}")
        
    # 2. Set up PyTorch-YOLOv3 repository clone for model definitions
    scratch_dir = "/home/jiucai/.gemini/antigravity-ide/scratch"
    repo_path = os.path.join(scratch_dir, "PyTorch-YOLOv3")
    
    if not os.path.exists(repo_path):
        print("Cloning eriklindernoren/PyTorch-YOLOv3 repository...")
        os.system(f"git clone https://github.com/eriklindernoren/PyTorch-YOLOv3.git {repo_path}")
        
    sys.path.append(repo_path)
    try:
        from pytorchyolo.models import Darknet
    except ImportError:
        print("ERROR: Failed to import Darknet from cloned PyTorch-YOLOv3 repo.")
        sys.exit(1)
        
    # 3. Create or download the base config file
    base_cfg = os.path.join(repo_path, "config", f"{model_type}.cfg")
    if not os.path.exists(base_cfg):
        # If it doesn't exist, download from pjreddie's official darknet config
        print(f"Downloading base config {model_type}.cfg...")
        os.system(f"curl -sS https://raw.githubusercontent.com/pjreddie/darknet/master/cfg/{model_type}.cfg -o {base_cfg}")
        
    # 4. Generate custom config with correct classes and filters
    custom_cfg = os.path.join(scratch_dir, f"{model_type}-custom.cfg")
    print(f"Generating custom configuration at {custom_cfg}...")
    
    with open(base_cfg, "r") as f:
        lines = f.readlines()
        
    new_lines = []
    for i, line in enumerate(lines):
        if line.strip().startswith("classes="):
            new_lines.append(f"classes={classes}\n")
        elif line.strip().startswith("filters=255"):
            # Only change conv filters right before yolo layers
            is_yolo_preconv = False
            for j in range(i+1, min(i+10, len(lines))):
                if "[yolo]" in lines[j]:
                    is_yolo_preconv = True
                    break
            if is_yolo_preconv:
                new_lines.append(f"filters={3 * (classes + 5)}\n")
            else:
                new_lines.append(line)
        else:
            new_lines.append(line)
            
    with open(custom_cfg, "w") as f:
        f.writelines(new_lines)
        
    # 5. Instantiate Darknet and rename state dict keys to match PyTorch-YOLOv3 naming
    print("Instantiating Darknet model and loading weights...")
    model = Darknet(custom_cfg)
    
    new_sd = {}
    for k, v in sd.items():
        parts = k.split('.')
        if len(parts) >= 4 and parts[0] == 'module_list':
            idx = parts[1]
            layer_type = parts[2]
            param_name = '.'.join(parts[3:])
            if layer_type == 'Conv2d':
                new_key = f"module_list.{idx}.conv_{idx}.{param_name}"
            elif layer_type == 'BatchNorm2d':
                new_key = f"module_list.{idx}.batch_norm_{idx}.{param_name}"
            else:
                new_key = k
            new_sd[new_key] = v
        else:
            new_sd[k] = v
            
    model.load_state_dict(new_sd, strict=False)
    model.eval()
    
    # 6. Export to ONNX
    output_dir = output_dir or os.path.dirname(pt_path)
    model_name = os.path.splitext(os.path.basename(pt_path))[0]
    onnx_path = os.path.join(output_dir, f"{model_name}.onnx")
    
    print(f"Exporting to ONNX (size: {imgsz}x{imgsz}, opset: {opset})...")
    dummy_input = torch.zeros(1, 3, imgsz, imgsz)
    torch.onnx.export(
        model,
        dummy_input,
        onnx_path,
        verbose=False,
        input_names=["images"],
        output_names=["output0"],
        opset_version=opset
    )
    print(f"ONNX model saved successfully to: {onnx_path}")

def main():
    args = parse_args()
    pt_path = os.path.abspath(args.pt_path)
    
    if not os.path.exists(pt_path):
        print(f"ERROR: File '{pt_path}' does not exist.")
        sys.exit(1)
        
    output_dir = args.output_dir
    if output_dir:
        output_dir = os.path.abspath(output_dir)
        os.makedirs(output_dir, exist_ok=True)
        
    # Load the checkpoint using PyTorch
    print(f"Loading checkpoint {pt_path}...")
    try:
        data = torch.load(pt_path, map_location="cpu")
    except Exception as e:
        print(f"ERROR: Failed to load PyTorch file: {e}")
        sys.exit(1)
        
    # Determine the model type
    if isinstance(data, dict) and "model" in data:
        model_data = data["model"]
        if isinstance(model_data, dict):  # state dict
            # Check if it uses Darknet naming (module_list)
            is_darknet = any(k.startswith("module_list.") for k in model_data.keys())
            if is_darknet:
                convert_darknet(pt_path, output_dir, args.imgsz, args.opset, model_data)
            else:
                print("Detected custom PyTorch state dict checkpoint. Cannot convert without custom model code.")
                sys.exit(1)
        else:
            # Serialized model object (YOLOv5 or YOLOv8)
            convert_ultralytics(pt_path, output_dir, args.imgsz, args.opset)
    else:
        # Weights only or standard PyTorch save
        is_darknet = any(k.startswith("module_list.") for k in data.keys())
        if is_darknet:
            convert_darknet(pt_path, output_dir, args.imgsz, args.opset, data)
        else:
            print("Detected custom PyTorch weights file. Cannot convert without custom model code.")
            sys.exit(1)

if __name__ == "__main__":
    main()
