import os
import glob
import subprocess
import json
import xml.etree.ElementTree as ET
import numpy as np
import cv2
import tifffile

# Constants
SENSOR_LONG_MM = 35.6 # Sony A7 III sensor width is approx 35.6 mm
SENSOR_SHORT_MM = 23.8 # Sony A7 III sensor height is approx 23.8 mm

def get_lens_focal_length(img_path):
    cmd = ['exiftool', '-FocalLength', '-j', img_path]
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
    cx, cy = w / 2.0, h / 2.0
    low, high = 0.1, 1.0
    best_s = 0.1
    
    for _ in range(30): # Binary search
        mid = (low + high) / 2.0
        w_half = mid * w / 2.0
        h_half = mid * h / 2.0
        corners = np.array([
            [cx - w_half, cy - h_half],
            [cx + w_half, cy - h_half],
            [cx + w_half, cy + h_half],
            [cx - w_half, cy + h_half]
        ], dtype=np.float64)
        
        corners_h = np.hstack([corners, np.ones((4, 1))])
        mapped_corners = (H_inv @ corners_h.T).T
        mapped_corners = mapped_corners[:, :2] / mapped_corners[:, 2:]
        
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

def main():
    base_dir = "/Users/timothyfennell/Documents/tilt correction/example images"
    in_dir = os.path.join(base_dir, "lens corrected tiff")
    xmp_dir = os.path.join(base_dir, "sidecars")
    out_dir = os.path.join(base_dir, "corrected_tiff_final")
    
    os.makedirs(out_dir, exist_ok=True)
    
    img_files = sorted(glob.glob(os.path.join(in_dir, "*.tif*")))
    xmp_files = sorted(glob.glob(os.path.join(xmp_dir, "*.xmp")))
    dng_dir = os.path.join(base_dir, "lens corrected dng")
    dng_files = sorted(glob.glob(os.path.join(dng_dir, "*.dng")))
    
    if len(img_files) == 0:
        print("No TIFF files found in", in_dir)
        return
        
    # Build mapping from DNG base name to XMP index
    dng_bases = [os.path.splitext(os.path.basename(d))[0] for d in dng_files]
    
    # Pass 1: Find the global minimum crop scale across the sequence
    global_min_scale = 1.0
    processed_data = []
    
    for img_path in img_files:
        base_name = os.path.splitext(os.path.basename(img_path))[0]
        try:
            dng_idx = dng_bases.index(base_name)
        except ValueError:
            print(f"Warning: {base_name} not found in DNG list. Skipping.")
            continue
            
        if dng_idx >= len(xmp_files):
            print(f"Warning: No XMP found for index {dng_idx} ({base_name}). Skipping.")
            continue
            
        xmp_path = xmp_files[dng_idx]
        
        focal_length = get_lens_focal_length(img_path)
        tilt_deg = get_tilt_angle(xmp_path)
        
        # Get image dimensions without loading entire array if possible
        # tifffile can do this
        with tifffile.TiffFile(img_path) as tif:
            page = tif.pages[0]
            h, w = page.shape[:2]
        
        # Determine sensor dimensions mapped to pixels
        if w >= h: # Landscape
            sensor_w = SENSOR_LONG_MM
            sensor_h = SENSOR_SHORT_MM
        else: # Portrait
            sensor_w = SENSOR_SHORT_MM
            sensor_h = SENSOR_LONG_MM
            
        fx = focal_length * (w / sensor_w)
        fy = focal_length * (h / sensor_h)
        cx = w / 2.0
        cy = h / 2.0
        
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        
        tilt_rad = np.radians(-tilt_deg) # Negative sign assumes positive tilt_deg = pitching UP
        R = np.array([
            [1, 0, 0],
            [0, np.cos(tilt_rad), -np.sin(tilt_rad)],
            [0, np.sin(tilt_rad), np.cos(tilt_rad)]
        ], dtype=np.float64)
        
        # Shift Lens simulation: translate image back to optical center
        dy = fy * np.tan(tilt_rad)
        T_mat = np.array([
            [1, 0, 0],
            [0, 1, dy],
            [0, 0, 1]
        ], dtype=np.float64)
        
        H_forward = T_mat @ K @ R @ np.linalg.inv(K)
        H_inv = np.linalg.inv(H_forward)
        
        scale = find_max_crop_scale(H_inv, w, h)
        if scale < global_min_scale:
            global_min_scale = scale
            
        processed_data.append((img_path, xmp_path, H_inv, w, h))
        
    print(f"Global crop scale determined: {global_min_scale}")
    
    # Pass 2: Process, warp, crop, and save
    for i in range(len(processed_data)):
        img_path, xmp_path, H_inv, w, h = processed_data[i]
        filename = os.path.basename(img_path)
        base_name = os.path.splitext(filename)[0]
        out_tiff = os.path.join(out_dir, f"{base_name}.tiff")
        
        print(f"Warping {filename}...")
        rgb = tifffile.imread(img_path)
        
        warped = cv2.warpPerspective(rgb, H_inv, (w, h), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)
        
        # Crop
        cw, ch = int(global_min_scale * w), int(global_min_scale * h)
        x_start = int((w - cw) / 2)
        y_start = int((h - ch) / 2)
        cropped = warped[y_start:y_start+ch, x_start:x_start+cw]
        
        print(f"Saving {out_tiff}...")
        tifffile.imwrite(out_tiff, cropped, photometric='rgb')
        
        # Copy EXIF
        cmd = ['exiftool', '-tagsFromFile', img_path, '-All:All', '-overwrite_original', out_tiff]
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        
    print(f"Done processing {len(processed_data)} frames.")

if __name__ == "__main__":
    main()
