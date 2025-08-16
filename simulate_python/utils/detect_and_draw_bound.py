import os
import cv2
import numpy as np

debug_opencv = 0

def detect_and_draw_bound(frame, width=640, height=480):
    height = int(height)
    width = int(width)
    
    frame_reshaped = frame.reshape((height, width, 3))
    frame_bgr = cv2.cvtColor(frame_reshaped, cv2.COLOR_RGB2BGR)
    hsv_frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    color_ranges = {
        "red": [
            (np.array([0, 100, 100]), np.array([10, 255, 255])),
            (np.array([170, 100, 100]), np.array([180, 255, 255]))
        ],

    }

    results = {}

    for color, ranges in color_ranges.items():
        mask = None
        for lower, upper in ranges:
            partial_mask = cv2.inRange(hsv_frame, lower, upper)
            mask = partial_mask if mask is None else cv2.bitwise_or(mask, partial_mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        best_bb = None
        max_area = 0

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500 and area > max_area:
                max_area = area
                x, y, w, h = cv2.boundingRect(contour)
                best_bb = (x, y, w, h)

        if best_bb:
            x, y, w, h = best_bb
            cv2.rectangle(frame_bgr, (x, y), (x+w, y+h), (0, 255, 0), 2)

            bb_center_x = (x + x+w) / 2
            bb_center_y = (y + y+h) / 2
            dx = (width / 2) - bb_center_x
            dy = (height / 2) - bb_center_y

            results[color] = {
                "bbox": np.array([x, y, x+w, y+h]),
                "dx": dx,
                "dy": dy
            }

    frame_bdbox = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    frame_boundbox = frame_bdbox.flatten()

    if debug_opencv:
        dirname = os.path.dirname(os.path.abspath(__file__))
        parent_dir = os.path.dirname(dirname)
        cv2.imwrite(os.path.join(parent_dir, 'images/test_hsvframe.png'), cv2.flip(hsv_frame, -1))
        cv2.imwrite(os.path.join(parent_dir, 'images/test_bgr_filtered.png'), cv2.flip(frame_bgr, -1))
    
    return frame_boundbox, results
