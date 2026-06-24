import pytest
import numpy as np
from follower.line_tracker import LineTracker

def test_line_tracker_init():
    """Test LineTracker initialization and default parameters."""
    tracker = LineTracker(kp=0.01, linear_speed=0.5, min_area=100, threshold_val=75)
    assert tracker.kp == 0.01
    assert tracker.linear_speed == 0.5
    assert tracker.min_area == 100
    assert tracker.threshold_val == 75

def test_line_tracker_empty_input():
    """Test that LineTracker handles invalid or empty inputs gracefully (defensive programming)."""
    tracker = LineTracker()
    
    # None image
    ang, lin, status, cx = tracker.process_image(None)
    assert status == "lost"
    assert ang == 0.0
    assert lin == 0.0
    assert cx is None

    # Empty array
    ang, lin, status, cx = tracker.process_image(np.array([]))
    assert status == "lost"
    assert ang == 0.0
    assert lin == 0.0
    assert cx is None

def test_line_tracker_black_input():
    """Test that completely black image is handled as lost (defensive programming)."""
    tracker = LineTracker()
    black_img = np.zeros((480, 640, 3), dtype=np.uint8)
    ang, lin, status, cx = tracker.process_image(black_img)
    assert status == "lost"
    assert ang == 0.0
    assert lin == 0.0
    assert cx is None

def test_line_tracker_centered_line():
    """Test that a centered black line results in zero error and zero angular speed."""
    tracker = LineTracker(kp=0.005, linear_speed=0.2, min_area=100)
    
    # Create white image background
    img = np.ones((480, 640, 3), dtype=np.uint8) * 255
    # Draw a black line down the middle (X: 310 to 330)
    img[:, 310:330, :] = 0
    
    ang, lin, status, cx = tracker.process_image(img)
    
    assert status == "visible"
    assert lin == 0.2
    # The centroid should be exactly at X=319 (center of indices 310 to 329)
    assert cx == 319
    # error = 319 - 320 = -1
    # ang = -0.005 * -1 = 0.005
    assert ang == pytest.approx(0.005)

def test_line_tracker_offset_right_line():
    """Test that a black line offset to the right produces negative angular speed."""
    tracker = LineTracker(kp=0.01, linear_speed=0.2, min_area=100)
    
    # Create white image background
    img = np.ones((480, 640, 3), dtype=np.uint8) * 255
    # Draw a black line down the right side (X: 410 to 430)
    img[:, 410:430, :] = 0
    
    ang, lin, status, cx = tracker.process_image(img)
    
    assert status == "visible"
    assert lin == 0.2
    # Centroid should be exactly at X=419
    assert cx == 419
    
    # Error = 419 - 320 = 99
    # angular_z = -kp * error = -0.01 * 99 = -0.99
    assert ang == pytest.approx(-0.99)

def test_line_tracker_offset_left_line():
    """Test that a black line offset to the left produces positive angular speed."""
    tracker = LineTracker(kp=0.01, linear_speed=0.2, min_area=100)
    
    # Create white image background
    img = np.ones((480, 640, 3), dtype=np.uint8) * 255
    # Draw a black line down the left side (X: 210 to 230)
    img[:, 210:230, :] = 0
    
    ang, lin, status, cx = tracker.process_image(img)
    
    assert status == "visible"
    assert lin == 0.2
    # Centroid should be exactly at X=219
    assert cx == 219
    
    # Error = 219 - 320 = -101
    # angular_z = -kp * error = -0.01 * (-101) = 1.01
    assert ang == pytest.approx(1.01)
