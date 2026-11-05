import numpy as np

from huggin_vision.segmentation import (
    HSV_RANGES, auto_saturation, color_masks, preprocess)


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


def washed_out_image():
    """Return a grey-brown wall (S 25) with a dull green patch (S 60).

    That is how the ESP32-CAM sees a green arrow: too pale for a fixed
    threshold.
    """
    hsv = np.zeros((60, 80, 3), np.uint8)
    hsv[:] = (20, 25, 180)  # wall
    hsv[20:40, 20:60] = (65, 60, 150)  # green arrow
    return hsv


def test_fixed_threshold_misses_a_pale_color():
    masks = color_masks(washed_out_image(), {'green': ((40, 70, 60),
                                                       (85, 255, 255))})
    assert not masks['green'].any()


def test_auto_saturation_finds_a_pale_color():
    ranges = {'green': ((40, 30, 60), (85, 255, 255))}
    mask = color_masks(washed_out_image(), ranges, saturation_margin=15)
    assert mask['green'][30, 40] and not mask['green'][5, 5]


def test_auto_saturation_is_half_way():
    # Background 25, color 60: the threshold falls at about 42
    s_min = auto_saturation(washed_out_image(), (40, 30, 60),
                            (85, 255, 255), 15)
    assert 38 <= s_min <= 46


def test_auto_saturation_without_the_color():
    # Nothing of that color: just the floor (background + margin or lower S)
    hsv = washed_out_image()
    assert auto_saturation(hsv, (100, 30, 60), (130, 255, 255), 15) == 40
