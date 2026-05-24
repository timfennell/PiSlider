#!/usr/bin/env python3
import os
import sys
import glob
from pathlib import Path

# Try to import Shine Stacker's core engine blocks
try:
    from shinestacker import StackJob, CombinedActions
    from shinestacker.algorithms import AlignFrames, BalanceFrames, PyramidStack
except ImportError:
    print("Error: 'shinestacker' python library is not found.")
    print("Please install it into your environment by running: pip install shinestacker")
    sys.exit(1)

def batch_process_project(parent_dir):
    parent_path = Path(parent_dir).resolve()
    if not parent_path.exists():
        print(f"Error: Target directory '{parent_path}' does not exist.")
        return

    print(f"Scanning project root: {parent_path}")
    
    # Locate all subfolders that contain images (e.g., orbit_001/stack_001, orbit_001/stack_002, etc.)
    # DNG is the primary format from PiSlider macro scans; ARW/CR3/NEF for Sony/Canon/Nikon raw.
    # JPG/JPEG included but _preview.jpg sidecars (written alongside DNG) are excluded below.
    extensions = ('*.dng', '*.jpg', '*.jpeg', '*.png', '*.tif', '*.tiff', '*.arw', '*.cr3', '*.nef')

    # Find all image paths and extract unique directories containing them
    image_files = []
    for ext in extensions:
        matches = glob.glob(os.path.join(parent_path, "**", ext), recursive=True)
        # Exclude PiSlider preview sidecars (_preview.jpg written alongside each DNG)
        matches = [f for f in matches if not f.endswith('_preview.jpg')]
        image_files.extend(matches)
    
    # Get a unique, sorted list of every subfolder that holds source frames
    stack_folders = sorted(list(set(os.path.dirname(f) for f in image_files)))
    
    if not stack_folders:
        print("No image folders found inside the target directory tree.")
        return

    print(f"Found {len(stack_folders)} stack directories to process.\n" + "="*60)

    for idx, src_dir in enumerate(stack_folders, 1):
        src_path = Path(src_dir)
        
        # Define where the output should go
        output_name = "best_focus"
        output_file_check = src_path / f"{output_name}.jpg"
        
        print(f"[{idx}/{len(stack_folders)}] Processing: {src_path.relative_to(parent_path)}")
        
        if output_file_check.exists():
            print(f" -> Skipping: {output_name}.jpg already exists in this folder.")
            print("-" * 60)
            continue

        try:
            # Set up the Shine Stacker Job Engine
            job = StackJob(
                name="shine_batch_job", 
                working_path=str(src_path), 
                input_path="" # Empty string forces it to use the raw working_path as direct source
            )
            
            # Step 1: Add Image Alignment & Exposure Leveling Actions
            job.add_action(CombinedActions("align_and_balance", [AlignFrames(), BalanceFrames()]))
            
            # Step 2: Add Focus Blending Layering Action using Laplacian Pyramids
            job.add_action(CombinedActions("pyramid_stack", [PyramidStack(output_name=output_name)]))
            
            # Step 3: Execute the pipeline for this folder
            job.run()
            print(f" -> Successfully created focus stack artifact.")
            
        except Exception as e:
            print(f" -> ERROR processing folder {src_path.name}: {e}")
            
        print("-" * 60)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 shine_batch.py /path/to/parent_project_folder")
        sys.exit(1)
        
    batch_process_project(sys.argv[1])