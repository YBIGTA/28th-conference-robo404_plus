import cv2
import numpy as np

class LineTracker:
    def __init__(self, kp=0.005, linear_speed=0.2, min_area=500, threshold_val=80):
        """
        Decoupled OpenCV-based Line Tracker.
        
        Args:
            kp (float): Proportional gain for steering control.
            linear_speed (float): Linear speed when line is detected.
            min_area (int): Minimum contour area to be considered a valid lane line.
            threshold_val (int): Grayscale intensity threshold (pixels below this are black line).
        """
        self.kp = kp
        self.linear_speed = linear_speed
        self.min_area = min_area
        self.threshold_val = threshold_val

    def process_image(self, bgr_image):
        """
        Process BGR image and calculate steering error.
        
        Args:
            bgr_image (numpy.ndarray): The input BGR image.
            
        Returns:
            angular_z (float): Proposed angular speed.
            linear_x (float): Proposed linear speed.
            status (str): "visible" or "lost"
            cx (int or None): Center X coordinate of the line.
        """
        # Defensive programming check: empty, invalid or None image
        if bgr_image is None or not isinstance(bgr_image, np.ndarray) or bgr_image.size == 0:
            return 0.0, 0.0, "lost", None

        # Check for completely black image
        if np.all(bgr_image == 0):
            return 0.0, 0.0, "lost", None

        try:
            height, width, _ = bgr_image.shape
            
            # Crop the lower 1/3 and use the full width of the image to maximize tracking range
            crop_h_start = int(2 * height / 3)
            crop_h_stop = height
            crop_w_start = 0
            crop_w_stop = width
            
            crop = bgr_image[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop]
            if crop.size == 0:
                return 0.0, 0.0, "lost", None

            # Convert to Grayscale
            gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
            
            # Threshold to isolate the black line on white ground (pixels below threshold_val)
            # Binary inverse threshold makes the line pixels white (255) and the background black (0)
            _, thresh = cv2.threshold(gray, self.threshold_val, 255, cv2.THRESH_BINARY_INV)
            
            # Find contours
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if not contours:
                return 0.0, 0.0, "lost", None
                
            # Filter largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            if area < self.min_area:
                return 0.0, 0.0, "lost", None
                
            # Compute moments
            M = cv2.moments(largest_contour)
            if M["m00"] == 0:
                return 0.0, 0.0, "lost", None
                
            # Centroid X coordinate relative to the crop
            cx_crop = int(M["m10"] / M["m00"])
            # Map back to global width coordinates
            cx = crop_w_start + cx_crop
            
            # Calculate error from image center
            error = cx - (width / 2.0)
            
            # P Controller: Angular speed is proportional to error
            # If line is to the right (error > 0), turn right (angular_z < 0)
            # If line is to the left (error < 0), turn left (angular_z > 0)
            angular_z = float(-self.kp * error)
            linear_x = self.linear_speed
            
            return angular_z, linear_x, "visible", cx
            
        except Exception:
            # Catch-all for defensive robustness
            return 0.0, 0.0, "lost", None
