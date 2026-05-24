import os
import glob
import subprocess
import json
import xml.etree.ElementTree as ET
import numpy as np
import cv2
import tifffile
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import shutil

# Constants
SENSOR_LONG_MM = 35.6
SENSOR_SHORT_MM = 23.8

def find_exiftool():
    # PyInstaller apps might not have standard PATH. Let's find exiftool.
    paths = ['/opt/homebrew/bin/exiftool', '/usr/local/bin/exiftool', '/usr/bin/exiftool']
    which_exiftool = shutil.which('exiftool')
    if which_exiftool:
        return which_exiftool
    for path in paths:
        if os.path.exists(path):
            return path
    return 'exiftool' # fallback to system path

EXIFTOOL_PATH = find_exiftool()

def get_lens_focal_length(img_path):
    cmd = [EXIFTOOL_PATH, '-FocalLength', '-j', img_path]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode == 0:
            data = json.loads(result.stdout)
            if len(data) > 0 and 'FocalLength' in data[0]:
                focal_str = data[0]['FocalLength']
                try:
                    return float(focal_str.split()[0])
                except ValueError:
                    pass
    except Exception as e:
        print(f"Exiftool error: {e}")
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
        pass
    return 0.0

def find_max_crop_scale(H_inv, w, h):
    cx, cy = w / 2.0, h / 2.0
    low, high = 0.1, 1.0
    best_s = 0.1
    for _ in range(30):
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

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Tilt Corrector")
        self.geometry("400x250")
        self.eval('tk::PlaceWindow . center')
        
        self.label = tk.Label(self, text="Select a folder containing your TIFFs and XMPs", pady=10)
        self.label.pack()
        
        # Override Frame
        override_frame = tk.Frame(self)
        override_frame.pack(pady=5)
        
        tk.Label(override_frame, text="Manual Focal Length (mm):").grid(row=0, column=0, padx=5, pady=2, sticky="e")
        self.focal_entry = tk.Entry(override_frame, width=10)
        self.focal_entry.grid(row=0, column=1, padx=5, pady=2)
        
        tk.Label(override_frame, text="Crop Factor (1.0=FF, 1.5=APS-C):").grid(row=1, column=0, padx=5, pady=2, sticky="e")
        self.crop_entry = tk.Entry(override_frame, width=10)
        self.crop_entry.grid(row=1, column=1, padx=5, pady=2)
        
        self.btn = tk.Button(self, text="Select Folder", command=self.select_folder)
        self.btn.pack(pady=10)
        
        self.progress = ttk.Progressbar(self, orient=tk.HORIZONTAL, length=300, mode='determinate')
        self.progress.pack(pady=10)
        
        self.status = tk.Label(self, text="Ready", fg="gray")
        self.status.pack()
        
    def select_folder(self):
        folder_path = filedialog.askdirectory(title="Select Source Folder")
        if folder_path:
            # Parse overrides
            focal_override = None
            crop_override = 1.0
            try:
                if self.focal_entry.get().strip():
                    focal_override = float(self.focal_entry.get().strip())
                if self.crop_entry.get().strip():
                    crop_override = float(self.crop_entry.get().strip())
            except ValueError:
                messagebox.showerror("Error", "Focal Length and Crop Factor must be valid numbers.")
                return

            self.btn.config(state=tk.DISABLED)
            threading.Thread(target=self.process_folder, args=(folder_path, focal_override, crop_override), daemon=True).start()

    def process_folder(self, base_dir, focal_override, crop_override):
        try:
            self.status.config(text="Scanning folder...", fg="black")
            
            img_files = sorted(glob.glob(os.path.join(base_dir, "*.tif*")))
            xmp_files = sorted(glob.glob(os.path.join(base_dir, "*.xmp")))
            
            if not img_files:
                messagebox.showerror("Error", "No TIFF files found.")
                self.reset()
                return
            if not xmp_files:
                messagebox.showerror("Error", "No XMP files found.")
                self.reset()
                return
                
            if len(img_files) != len(xmp_files):
                messagebox.showerror("Error", f"Count Mismatch!\nFound {len(img_files)} TIFFs and {len(xmp_files)} XMPs.\nThe counts must be exactly equal to proceed.")
                self.reset()
                return
                
            num_to_process = len(img_files)
            out_dir = os.path.join(base_dir, "corrected_tiff_final")
            os.makedirs(out_dir, exist_ok=True)
            
            self.progress['maximum'] = num_to_process * 2 # 2 passes
            self.progress['value'] = 0
            
            self.status.config(text="Pass 1: Calculating global crop...")
            
            global_min_scale = 1.0
            processed_data = []
            
            for i in range(num_to_process):
                img_path = img_files[i]
                xmp_path = xmp_files[i] # 1:1 alphabetical match
                
                if focal_override is not None:
                    focal_length = focal_override
                else:
                    focal_length = get_lens_focal_length(img_path)
                    
                tilt_deg = get_tilt_angle(xmp_path)
                
                with tifffile.TiffFile(img_path) as tif:
                    page = tif.pages[0]
                    h, w = page.shape[:2]
                
                if w >= h:
                    sensor_w = SENSOR_LONG_MM / crop_override
                    sensor_h = SENSOR_SHORT_MM / crop_override
                else:
                    sensor_w = SENSOR_SHORT_MM / crop_override
                    sensor_h = SENSOR_LONG_MM / crop_override
                    
                fx = focal_length * (w / sensor_w)
                fy = focal_length * (h / sensor_h)
                cx = w / 2.0
                cy = h / 2.0
                
                K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
                
                tilt_rad = np.radians(-tilt_deg)
                R = np.array([
                    [1, 0, 0],
                    [0, np.cos(tilt_rad), -np.sin(tilt_rad)],
                    [0, np.sin(tilt_rad), np.cos(tilt_rad)]
                ], dtype=np.float64)
                
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
                
                self.progress['value'] += 1
                self.update_idletasks()
                
            self.status.config(text=f"Pass 2: Warping {num_to_process} images...")
            
            for i in range(len(processed_data)):
                img_path, xmp_path, H_inv, w, h = processed_data[i]
                filename = os.path.basename(img_path)
                base_name = os.path.splitext(filename)[0]
                out_tiff = os.path.join(out_dir, f"{base_name}.tiff")
                
                rgb = tifffile.imread(img_path)
                warped = cv2.warpPerspective(rgb, H_inv, (w, h), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)
                
                cw, ch = int(global_min_scale * w), int(global_min_scale * h)
                x_start = int((w - cw) / 2)
                y_start = int((h - ch) / 2)
                cropped = warped[y_start:y_start+ch, x_start:x_start+cw]
                
                tifffile.imwrite(out_tiff, cropped, photometric='rgb')
                
                cmd = [EXIFTOOL_PATH, '-tagsFromFile', img_path, '-All:All', '-overwrite_original', out_tiff]
                subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                
                self.progress['value'] += 1
                self.update_idletasks()
                
            self.status.config(text="Complete!", fg="green")
            messagebox.showinfo("Success", f"Processed {num_to_process} images successfully!")
            self.reset()
            
        except Exception as e:
            messagebox.showerror("Error", f"An error occurred:\n{str(e)}")
            self.reset()

    def reset(self):
        self.btn.config(state=tk.NORMAL)
        self.progress['value'] = 0

if __name__ == "__main__":
    app = App()
    app.mainloop()
