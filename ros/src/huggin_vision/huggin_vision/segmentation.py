"""Frame preprocessing and HSV color segmentation."""
import cv2

# Default HSV thresholds, (lower, upper) with H 0-179, S and V 0-255.
# This is also the list of detectable colors.
HSV_RANGES = {
    'red': ((0, 120, 80), (10, 255, 255)),
    'blue': ((100, 120, 80), (130, 255, 255)),
    'green': ((40, 80, 80), (80, 255, 255)),
}

# RGB (0-1) used to draw each color
MARKER_COLORS = {'red': (1.0, 0.0, 0.0), 'blue': (0.0, 0.0, 1.0),
                 'green': (0.0, 1.0, 0.0)}

BLUR_KERNEL = (5, 5)
OPEN_KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))


def preprocess(bgr, size):
    """Resize a BGR image to size (width, height); return (frame, hsv)."""
    frame = cv2.resize(bgr, size)
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, BLUR_KERNEL, 0),
                       cv2.COLOR_BGR2HSV)
    return frame, hsv


def clean_mask(mask):
    """Remove isolated noise pixels from a binary mask."""
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, OPEN_KERNEL)


def color_masks(hsv, ranges):
    """Return {color: cleaned binary mask} for a {color: (lower, upper)}."""
    return {color: clean_mask(cv2.inRange(hsv, tuple(lower), tuple(upper)))
            for color, (lower, upper) in ranges.items()}
