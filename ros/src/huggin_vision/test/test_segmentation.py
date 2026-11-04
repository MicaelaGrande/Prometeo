import numpy as np

from huggin_vision.segmentation import HSV_RANGES, color_masks, preprocess


def test_preprocess_resizes():
    frame, hsv = preprocess(np.zeros((100, 200, 3), np.uint8), (64, 48))
    assert frame.shape == (48, 64, 3)
    assert hsv.shape == (48, 64, 3)


def test_pure_colors_land_in_their_own_mask():
    bgr = np.zeros((30, 90, 3), np.uint8)
    bgr[:, :30] = (0, 0, 255)  # red
    bgr[:, 30:60] = (255, 0, 0)  # blue
    bgr[:, 60:] = (0, 255, 0)  # green
    _, hsv = preprocess(bgr, (90, 30))
    masks = color_masks(hsv, HSV_RANGES)
    assert set(masks) == {'red', 'blue', 'green'}
    assert masks['red'][15, 15] and not masks['red'][15, 45]
    assert masks['blue'][15, 45] and not masks['blue'][15, 15]
    assert masks['green'][15, 75] and not masks['green'][15, 15]


def test_hue_range_wraps_around_red():
    # Red seen at both ends of the hue circle: H 5 and H 175
    hsv = np.zeros((10, 30, 3), np.uint8)
    hsv[:, :10] = (5, 200, 200)
    hsv[:, 10:20] = (175, 200, 200)
    hsv[:, 20:] = (90, 200, 200)  # cyan, not red
    mask = color_masks(hsv, {'red': ((165, 70, 70), (10, 255, 255))})['red']
    assert mask[5, 5] and mask[5, 15] and not mask[5, 25]
