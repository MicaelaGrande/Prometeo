"""Frame preprocessing and HSV color segmentation."""
import cv2
import numpy as np

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

# Automatic saturation threshold (see color_masks). Light and camera change
# how saturated everything looks, so the threshold is set again per image
BACKGROUND_PERCENTILE = 50  # median: walls and floor are most of the image
COLOR_PERCENTILE = 75  # how saturated the pixels of one color are
MIN_COLOR_PIXELS = 30  # fewer candidate pixels: that color is not in view
# Where the threshold falls from background (0) to color level (1)
COLOR_SPLIT = 0.5

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


def in_range(hsv, lower, upper):
    """Binary mask of the pixels between lower and upper.

    If the lower hue is above the upper one the range wraps around the end
    of the hue circle, as red does: [165, ...] to [10, ...] takes 165-179 and
    0-10.
    """
    if lower[0] <= upper[0]:
        return cv2.inRange(hsv, tuple(lower), tuple(upper))
    high = cv2.inRange(hsv, (lower[0], lower[1], lower[2]),
                       (179, upper[1], upper[2]))
    low = cv2.inRange(hsv, (0, lower[1], lower[2]),
                      (upper[0], upper[1], upper[2]))
    return cv2.bitwise_or(high, low)


def auto_saturation(hsv, lower, upper, margin):
    """Lowest saturation of one color in this image.

    The background saturation (walls and floor) is the median of the
    whole image. Pixels of the color's hue at least margin above it are
    candidates, and their saturation is the color level. The threshold is
    half way (COLOR_SPLIT) between background and color, so the outline of
    the blob falls on the edge of the object instead of a pixel out or in.
    Never below the S of lower, nor less than margin above the background.
    """
    background = float(np.percentile(hsv[..., 1], BACKGROUND_PERCENTILE))
    least = max(float(lower[1]), background + margin)
    candidates = in_range(hsv, (lower[0], least, lower[2]), upper)
    saturation = hsv[..., 1][candidates > 0]
    if saturation.size < MIN_COLOR_PIXELS:
        return least
    level = float(np.percentile(saturation, COLOR_PERCENTILE))
    return max(least, background + COLOR_SPLIT * (level - background))


def color_masks(hsv, ranges, saturation_margin=0):
    """Return {color: cleaned binary mask} for a {color: (lower, upper)}.

    With saturation_margin > 0 the S of lower is only a floor and the real
    threshold is found in each image (auto_saturation).
    """
    masks = {}
    for color, (lower, upper) in ranges.items():
        if saturation_margin > 0:
            s_min = auto_saturation(hsv, lower, upper, saturation_margin)
            lower = (lower[0], s_min, lower[2])
        masks[color] = clean_mask(in_range(hsv, lower, upper))
    return masks
