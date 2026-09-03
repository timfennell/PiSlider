#!/usr/bin/env python3
"""
stitching.py — Panorama & mask stitching helpers using OpenCV.

This module provides:
- stitch_images():    stitch multiple RGB/BGR images into a panorama
- stitch_masks():     stitch multiple binary masks (0/255) into a panorama mask

Features:
- Uses OpenCV's Stitcher (PANORAMA or SCANS mode)
- Optional spherical warper for wide-angle sweeps
- Accepts either file paths or preloaded np.ndarray images
- Designed to degrade gracefully (returns status != 0 on failure)
- Compatible with Raspberry Pi OS (uses opencv-python-headless)

Intended use:
    from stitching import stitch_images, stitch_masks

    status, pano = stitch_images(image_paths=["a.jpg", "b.jpg", "c.jpg"])
    if status == 0:
        cv2.imwrite("pano.jpg", pano)
"""

from __future__ import annotations

import os
import logging
from typing import List, Optional, Tuple

import numpy as np

# Try OpenCV import
try:
    import cv2
    _CV2_OK = True
except Exception:
    _CV2_OK = False


# -----------------------------------------------------------------------------
# Internal utilities
# -----------------------------------------------------------------------------
def _load_bgr(path: str) -> np.ndarray:
    """
    Load an image from disk as BGR numpy array.
    """
    if not _CV2_OK:
        raise RuntimeError("OpenCV not available; install opencv-python-headless.")
    img = cv2.imread(path, cv2.IMREAD_COLOR)
    if img is None:
        raise FileNotFoundError(f"Failed to read image: {path}")
    return img


def _load_gray(path: str) -> np.ndarray:
    """
    Load a grayscale image from disk (for mask stitching).
    """
    if not _CV2_OK:
        raise RuntimeError("OpenCV not available; install opencv-python-headless.")
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Failed to read image: {path}")
    return img


# -----------------------------------------------------------------------------
# Public API: Image stitching
# -----------------------------------------------------------------------------
def stitch_images(
    images: Optional[List[np.ndarray]] = None,
    image_paths: Optional[List[str]] = None,
    mode: str = "panorama",
    try_spherical: bool = True
) -> Tuple[int, Optional[np.ndarray]]:
    """
    Stitch multiple color images into a panorama.

    Parameters
    ----------
    images : list of np.ndarray
        Images in BGR format. If None, image_paths is used instead.
    image_paths : list of str
        Paths to images to load.
    mode : str
        "panorama" (default) or "scans". Matches OpenCV stitcher modes.
    try_spherical : bool
        If True, attempt to use a spherical warper (for wide sweeps).

    Returns
    -------
    status : int
        OpenCV status code. 0 = OK, others indicate failure.
    pano : np.ndarray or None
        Stitched panorama in BGR format on success; None on failure.
    """
    if not _CV2_OK:
        raise RuntimeError("OpenCV not available; install opencv-python-headless.")

    if images is None and image_paths is None:
        raise ValueError("Provide either 'images' or 'image_paths'.")

    # Load from file paths if needed
    try:
        if images is None:
            imgs = [_load_bgr(p) for p in image_paths]
        else:
            imgs = images
    except Exception as e:
        logging.error(f"Error loading images for stitching: {e}")
        return -1, None

    # Choose stitcher mode
    if mode.lower() == "scans":
        stitcher = cv2.Stitcher_create(cv2.Stitcher_SCANS)
    else:
        stitcher = cv2.Stitcher_create(cv2.Stitcher_PANORAMA)

    # Spherical warper (if available)
    if try_spherical:
        try:
            warper = cv2.detail.SphericalWarper()
            stitcher.setWarper(warper)
        except Exception:
            pass  # Some builds of OpenCV lack cv2.detail

    # Run stitching
    try:
        status, pano = stitcher.stitch(imgs)
    except Exception as e:
        logging.error(f"Stitcher encountered an error: {e}")
        return -1, None

    if status == cv2.Stitcher_OK:
        return 0, pano
    else:
        logging.warning(f"Stitch failed with code {status}")
        return status, None


# -----------------------------------------------------------------------------
# Public API: Mask stitching
# -----------------------------------------------------------------------------
def stitch_masks(
    masks: Optional[List[np.ndarray]] = None,
    mask_paths: Optional[List[str]] = None,
    mode: str = "panorama",
    binarize: bool = True
) -> Tuple[int, Optional[np.ndarray]]:
    """
    Stitch binary sky masks (0/255 uint8) into a panorama mask.

    Parameters
    ----------
    masks : list of np.ndarray
        If provided, used directly. Each mask should be uint8 and 0/255.
    mask_paths : list of str
        Paths to mask images (will be loaded as grayscale).
    mode : str
        Stitcher mode ("panorama" or "scans").
    binarize : bool
        If True, re-threshold result to ensure 0/255 binary output.

    Returns
    -------
    status : int
        0 = OK, else OpenCV error code.
    pano_mask : np.ndarray or None
        Stitched mask (uint8), or None on failure.
    """
    if not _CV2_OK:
        raise RuntimeError("OpenCV not available; install opencv-python-headless.")

    if masks is None and mask_paths is None:
        raise ValueError("Provide either 'masks' or 'mask_paths'.")

    # Load masks
    try:
        if masks is None:
            imgs = [_load_gray(p) for p in mask_paths]
        else:
            imgs = [m if m.ndim == 2 else cv2.cvtColor(m, cv2.COLOR_BGR2GRAY) for m in masks]
    except Exception as e:
        logging.error(f"Error loading masks for stitching: {e}")
        return -1, None

    # Convert to 3-channel grayscale so stitcher processes them like images
    imgs_bgr = [cv2.cvtColor(m, cv2.COLOR_GRAY2BGR) for m in imgs]

    # Call main stitcher
    status, pano_bgr = stitch_images(images=imgs_bgr, image_paths=None, mode=mode)
    if status != 0 or pano_bgr is None:
        return status, None

    # Convert back to grayscale mask
    pano_gray = cv2.cvtColor(pano_bgr, cv2.COLOR_BGR2GRAY)

    if binarize:
        _, pano_gray = cv2.threshold(pano_gray, 127, 255, cv2.THRESH_BINARY)

    return 0, pano_gray
