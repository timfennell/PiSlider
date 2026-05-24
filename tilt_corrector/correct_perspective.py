import os
import glob
import subprocess
import json
import xml.etree.ElementTree as ET
import numpy as np
import cv2
import rawpy
import tifffile
import imageio

# Constants
SENSOR_WIDTH_MM = 35.6 # Sony A7 III sensor width is approx 35.6 mm
SENSOR_HEIGHT_MM = 23.8 # Sony A7 III sensor height is approx 23.8 mm

def get_lens_focal_length(dng_path):
    cmd = ['exiftool', '-FocalLength', '-j', dng_path]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode == 0:
        data = json.loads(result.stdout)
        if len(data) > 0 and 'FocalLength' in data[0]:
            focal_str = data[0]['FocalLength']
            try:
                return float(focal_str.split()[0])
            except ValueError:
                pass
    return 14.0

def get_tilt_angle(xmp_path):
    try:
        tree = ET.parse(xmp_path)
        root = tree.getroot()
        ns = {
            'rdf': 'http://www.w3.org/1999/02/22-rdf-syntax-ns#',
            'ps': 'http://ns.pislider.io/1.0/'
        }
        for desc in root.findall('.//rdf:Description', ns):
            tilt = desc.find('ps:Rig_Tilt_Deg', ns)
            if tilt is not None:
                return float(tilt.text)
    except Exception as e:
        print(f"Error parsing XMP {xmp_path}: {e}")
    return 0.0

def find_max_crop_scale(H_inv, w, h):
    """
    Finds the maximum scale factor s in (0, 1] such that a centered rectangle
    of size (s*w, s*h) in the warped image maps entirely within the valid
    region [0, w] x [0, h] of the original image.
    H_inv maps from warped image coordinates to original image coordinates.
    """
    cx, cy = w / 2.0, h / 2.0
    low, high = 0.1, 1.0
    best_s = 0.1
    
    for _ in range(30): # Binary search
        mid = (low + high) / 2.0
        
        # Corners of the centered crop in warped space
        w_half = mid * w / 2.0
        h_half = mid * h / 2.0
        corners = np.array([
            [cx - w_half, cy - h_half],
            [cx + w_half, cy - h_half],
            [cx + w_half, cy + h_half],
            [cx - w_half, cy + h_half]
        ], dtype=np.float64)
        
        # Add homogeneous coordinate
        corners_h = np.hstack([corners, np.ones((4, 1))])
        
        # Map back to original image space
        mapped_corners = (H_inv @ corners_h.T).T
        mapped_corners = mapped_corners[:, :2] / mapped_corners[:, 2:]
        
        # Check if all mapped corners are inside the original image
        inside = True
        for (mx, my) in mapped_corners:
            if mx < 0 or mx > w - 1 or my < 0 or my > h - 1:
                inside = False
                break
                
        if inside:
            best_s = mid
            low = mid
        else:
            high = mid
            
    return best_s

def process_image(dng_path, xmp_path, out_dir):
    filename = os.path.basename(dng_path)
    base_name = os.path.splitext(filename)[0]
    out_tiff = os.path.join(out_dir, f"{base_name}.tiff")
    
    focal_length = get_lens_focal_length(dng_path)
    tilt_deg = get_tilt_angle(xmp_path)
    print(f"Processing {filename}: Focal Length = {focal_length}mm, Tilt = {tilt_deg} degrees")
    
    with rawpy.imread(dng_path) as raw:
        rgb = raw.postprocess(
            output_bps=16,
            use_camera_wb=True,
            no_auto_bright=True,
            gamma=(1, 1) # Linear gamma
        )
    
    h, w = rgb.shape[:2]
    
    fx = focal_length * (w / SENSOR_WIDTH_MM)
    fy = focal_length * (h / SENSOR_HEIGHT_MM)
    cx = w / 2.0
    cy = h / 2.0
    
    K = np.array([
        [fx,  0, cx],
        [ 0, fy, cy],
        [ 0,  0,  1]
    ], dtype=np.float64)
    
    tilt_rad = np.radians(-tilt_deg) # Negative may be needed based on camera rig
    R = np.array([
        [1,                 0,                  0],
        [0, np.cos(tilt_rad), -np.sin(tilt_rad)],
        [0, np.sin(tilt_rad),  np.cos(tilt_rad)]
    ], dtype=np.float64)
    
    K_inv = np.linalg.inv(K)
    H = K @ R @ K_inv
    H_inv = np.linalg.inv(H)
    
    # Calculate global max crop for the sequence if we want stable animation
    # But since tilt changes per frame, we'll just calculate it per frame
    # Wait: The user wants "while maintianing 1:1 scale for the smooth aniamtion, crop the images to the largest possible area maintaining aspect ratio"
    # This implies a global crop across all frames to keep animation smooth without zooming in/out.
    # To do a global crop, we need to find the maximum crop scale for all images and use the minimum of those scales.
    
    # Let's just process the warp first
    warped = cv2.warpPerspective(rgb, H, (w, h), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP, borderMode=cv2.BORDER_CONSTANT, borderValue=(0,0,0))
    
    return warped, H_inv, w, h, dng_path, out_tiff

def main():
    base_dir = "/Users/timothyfennell/Documents/tilt correction/example images"
    dng_dir = os.path.join(base_dir, "lens corrected dng")
    xmp_dir = os.path.join(base_dir, "sidecars")
    out_dir = os.path.join(base_dir, "corrected_tiff")
    
    os.makedirs(out_dir, exist_ok=True)
    
    dng_files = sorted(glob.glob(os.path.join(dng_dir, "*.dng")))
    xmp_files = sorted(glob.glob(os.path.join(xmp_dir, "*.xmp")))
    
    if len(dng_files) != len(xmp_files):
        print(f"Warning: Number of DNGs ({len(dng_files)}) does not match XMPs ({len(xmp_files)})")
    
    num_to_process = min(len(dng_files), len(xmp_files))
    
    # Pass 1: Find the global minimum crop scale across the sequence
    global_min_scale = 1.0
    processed_data = []
    
    for i in range(num_to_process):
        dng_path = dng_files[i]
        xmp_path = xmp_files[i]
        
        focal_length = get_lens_focal_length(dng_path)
        tilt_deg = get_tilt_angle(xmp_path)
        
        # We can calculate H without loading the image to save time
        # We assume w, h from first image or a standard size
        with rawpy.imread(dng_path) as raw:
            w = raw.sizes.width
            h = raw.sizes.height
        
        fx = focal_length * (w / SENSOR_WIDTH_MM)
        fy = focal_length * (h / SENSOR_HEIGHT_MM)
        cx = w / 2.0
        cy = h / 2.0
        
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        tilt_rad = np.radians(-tilt_deg)
        R = np.array([
            [1, 0, 0],
            [0, np.cos(tilt_rad), -np.sin(tilt_rad)],
            [0, np.sin(tilt_rad), np.cos(tilt_rad)]
        ], dtype=np.float64)
        H = K @ R @ np.linalg.inv(K)
        H_inv = np.linalg.inv(H)
        
        scale = find_max_crop_scale(H_inv, w, h)
        if scale < global_min_scale:
            global_min_scale = scale
            
        processed_data.append((dng_path, xmp_path, H, w, h))
        
    print(f"Global crop scale determined: {global_min_scale}")
    
    # Pass 2: Process, warp, crop, and save
    for i in range(num_to_process):
        dng_path, xmp_path, H, w, h = processed_data[i]
        filename = os.path.basename(dng_path)
        base_name = os.path.splitext(filename)[0]
        out_tiff = os.path.join(out_dir, f"{base_name}.tiff")
        
        print(f"Warping {filename}...")
        with rawpy.imread(dng_path) as raw:
            rgb = raw.postprocess(
                output_bps=16,
                use_camera_wb=True,
                no_auto_bright=True,
                gamma=(1, 1)
            )
            
        warped = cv2.warpPerspective(rgb, H, (w, h), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)
        
        # Crop
        cw, ch = int(global_min_scale * w), int(global_min_scale * h)
        x_start = int((w - cw) / 2)
        y_start = int((h - ch) / 2)
        cropped = warped[y_start:y_start+ch, x_start:x_start+cw]
        
        print(f"Saving {out_tiff}...")
        tifffile.imwrite(out_tiff, cropped, photometric='rgb')
        
        # Copy EXIF
        cmd = ['exiftool', '-tagsFromFile', dng_path, '-All:All', '-overwrite_original', out_tiff]
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        
    print(f"Done processing {num_to_process} frames.")

if __name__ == "__main__":
    main()
