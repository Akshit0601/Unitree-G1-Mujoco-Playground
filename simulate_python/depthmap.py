frame_width = 1200
frame_height = 720

import cv2
import numpy as np

def depth_to_colormap(depth_flat, width=1200, height=720):
    """
    Convert a 1D depth array (in meters) into a heatmap image.

    Args:
        depth_flat (np.ndarray): 1D array of depth in meters, length = width*height
        width (int): frame width
        height (int): frame height

    Returns:
        heatmap (np.ndarray): RGB heatmap image
    """
    # Reshape into (H, W)
    depth_map = depth_flat.reshape((height, width))

    # Normalize to 0–255 for visualization
    depth_norm = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX)
    depth_norm = depth_norm.astype(np.uint8)

    # Apply a heatmap (JET: Blue→Red)
    heatmap = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
    heatmap = cv2.rotate(heatmap, cv2.ROTATE_180)

    return heatmap

depth_m = np.load("log.npy")

heatmap = depth_to_colormap(depth_m)

cv2.imshow("Depth Heatmap", heatmap)
cv2.waitKey(0)
cv2.destroyAllWindows()
