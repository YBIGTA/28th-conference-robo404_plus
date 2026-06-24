#!/usr/bin/env python3
"""
A ROS2 node used to control a differential drive robot with a camera,
so it follows the line in a Robotrace style track.
You may change the parameters to your liking.
"""
__author__ = "Gabriel Nascarella Hishida do Nascimento"

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Empty

import numpy as np
import cv2
import cv_bridge

# Create a bridge between ROS and OpenCV
bridge = cv_bridge.CvBridge()

## User-defined parameters: (Update these values to your liking)
# Minimum size for a contour to be considered anything
MIN_AREA = 500 

# Minimum size for a contour to be considered part of the track
MIN_AREA_TRACK = 5000

# Robot's speed when following the line
LINEAR_SPEED = 20.0

# Proportional constant to be applied on speed when turning 
# (Multiplied by the error value)
KP = 1.5/100 

# If the line is completely lost, the error value shall be compensated by:
LOSS_FACTOR = 1.2

# Send messages every $TIMER_PERIOD seconds
TIMER_PERIOD = 0.0333

# When about to end the track, move for ~$FINALIZATION_PERIOD more seconds
FINALIZATION_PERIOD = 4

# The maximum error value for which the robot is still in a straight line
MAX_ERROR = 30

# BGR values to filter only the selected color range
lower_bgr_values = np.array([0, 0, 0])
upper_bgr_values = np.array([30, 30, 30])

def crop_size(height, width):
    """
    Get the measures to crop the image
    Output:
    (Height_upper_boundary, Height_lower_boundary,
     Width_left_boundary, Width_right_boundary)
    """
    ## Update these values to your liking.

    # Wider horizontal band so the line stays in view through curves.
    return (1*height//3, height, width//6, 5*width//6)


# Global vars. initial values
image_input = 0
image_header = None
error = 0
last_error = 0
just_seen_line = False
just_seen_right_mark = False
should_move = False
right_mark_count = 0
finalization_countdown = None
KD = 0.0
publish_debug_image = False
publish_mask_image = False
show_debug_window = False
enable_udp_stream = False
stream_host = "127.0.0.1"
stream_port = 5001
stream_width = 960
stream_height = 540
stream_framerate = 30
stream_bitrate = 4000000
use_hw_encoder = True
draw_fps = True
debug_image_publisher = None
mask_image_publisher = None
stream_writer = None
stream_failed = False
last_frame_time = None
fps = 0.0

PATH_LINE_VISIBLE = "LINE_VISIBLE"
PATH_LINE_LOST = "LINE_LOST"


def start_follower_callback(request, response):
    """
    Start the robot.
    In other words, allow it to move (again)
    """
    global should_move
    global right_mark_count
    global finalization_countdown
    should_move = True
    right_mark_count = 0
    finalization_countdown = None
    return response

def stop_follower_callback(request, response):
    """
    Stop the robot
    """
    global should_move
    global finalization_countdown
    should_move = False
    finalization_countdown = None
    return response

def image_callback(msg):
    """
    Function to be called whenever a new Image message arrives.
    Update the global variable 'image_input' and trigger control logic.
    """
    global image_input
    global image_header
    image_input = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    image_header = msg.header
    timer_callback()

def update_fps():
    """
    Update and return an exponentially smoothed timer callback FPS.
    """
    global last_frame_time
    global fps

    now = time.monotonic()
    if last_frame_time is not None:
        elapsed = now - last_frame_time
        if elapsed > 0.0:
            instant_fps = 1.0 / elapsed
            if fps <= 0.0:
                fps = instant_fps
            else:
                fps = fps * 0.9 + instant_fps * 0.1
    last_frame_time = now
    return fps

def draw_debug_overlay(output, path_state, mark_side, message):
    """
    Draw compact runtime state on the follower debug image.
    """
    lines = [
        "path_state: {}".format(path_state),
        "error: {:.1f}".format(float(error)),
        "angular_z: {:.3f}".format(float(message.angular.z)),
        "should_move: {}".format(should_move),
        "mark_side: {}".format(mark_side if mark_side is not None else "none"),
    ]
    if draw_fps and fps > 0.0:
        lines.append("fps: {:.2f}".format(fps))

    y = 24
    for line in lines:
        cv2.putText(
            output,
            line,
            (12, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            output,
            line,
            (12, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 0),
            1,
            cv2.LINE_AA,
        )
        y += 24

def publish_debug_outputs(output, mask):
    """
    Publish optional ROS image debug topics for headless Jetson validation.
    """
    if publish_debug_image and debug_image_publisher is not None:
        debug_msg = bridge.cv2_to_imgmsg(output, encoding='bgr8')
        if image_header is not None:
            debug_msg.header = image_header
        debug_image_publisher.publish(debug_msg)

    if publish_mask_image and mask_image_publisher is not None:
        mask_msg = bridge.cv2_to_imgmsg(mask, encoding='mono8')
        if image_header is not None:
            mask_msg.header = image_header
        mask_image_publisher.publish(mask_msg)

def validate_stream_parameters():
    """
    Check stream configuration before trying to open a GStreamer pipeline.
    """
    if not enable_udp_stream:
        return True

    numeric_parameters = {
        "stream_port": stream_port,
        "stream_width": stream_width,
        "stream_height": stream_height,
        "stream_framerate": stream_framerate,
        "stream_bitrate": stream_bitrate,
    }

    for name, value in numeric_parameters.items():
        if value <= 0:
            node.get_logger().error("{} must be > 0".format(name))
            return False

    if not stream_host:
        node.get_logger().error("stream_host must not be empty")
        return False

    return True

def hardware_pipeline():
    """
    Return the Jetson hardware H.264 RTP GStreamer pipeline.
    """
    return (
        "appsrc is-live=true block=false format=time do-timestamp=true"
        f" ! video/x-raw,format=BGR,width=(int){stream_width}"
        f",height=(int){stream_height}"
        f",framerate=(fraction){stream_framerate}/1"
        " ! queue leaky=downstream max-size-buffers=1"
        " ! videoconvert ! video/x-raw,format=I420"
        " ! nvvidconv"
        f" ! nvv4l2h264enc bitrate={stream_bitrate}"
        " insert-sps-pps=true"
        " ! h264parse"
        " ! rtph264pay config-interval=1 pt=96"
        f" ! udpsink host={stream_host} port={stream_port}"
        " sync=false async=false"
    )

def software_pipeline():
    """
    Return the software H.264 RTP GStreamer pipeline.
    """
    bitrate_kbps = max(1, int(stream_bitrate / 1000))
    return (
        "appsrc is-live=true block=false format=time do-timestamp=true"
        f" ! video/x-raw,format=BGR,width=(int){stream_width}"
        f",height=(int){stream_height}"
        f",framerate=(fraction){stream_framerate}/1"
        " ! queue leaky=downstream max-size-buffers=1"
        " ! videoconvert ! video/x-raw,format=I420"
        " ! x264enc tune=zerolatency speed-preset=ultrafast"
        f" bitrate={bitrate_kbps}"
        f" key-int-max={stream_framerate}"
        " ! rtph264pay config-interval=1 pt=96"
        f" ! udpsink host={stream_host} port={stream_port}"
        " sync=false async=false"
    )

def open_stream_writer():
    """
    Open the UDP stream writer with hardware then software fallback.
    """
    global stream_writer
    global stream_failed

    if stream_writer is not None or stream_failed:
        return

    pipelines = []
    if use_hw_encoder:
        pipelines.append(("hardware", hardware_pipeline()))
    pipelines.append(("software", software_pipeline()))

    for pipeline_name, pipeline in pipelines:
        writer = cv2.VideoWriter(
            pipeline,
            cv2.CAP_GSTREAMER,
            0,
            float(stream_framerate),
            (stream_width, stream_height),
            True,
        )
        if writer.isOpened():
            stream_writer = writer
            node.get_logger().info(
                "Opened {} follower debug stream to {}:{}".format(
                    pipeline_name,
                    stream_host,
                    stream_port,
                )
            )
            return

        writer.release()
        node.get_logger().warn(
            "Failed to open {} follower debug stream pipeline".format(
                pipeline_name
            )
        )

    stream_failed = True
    node.get_logger().error(
        "Follower UDP debug stream disabled after pipeline failures"
    )

def close_stream_writer():
    """
    Close the UDP stream writer.
    """
    global stream_writer

    if stream_writer is not None:
        stream_writer.release()
        stream_writer = None

def publish_stream(output):
    """
    Write one frame to the UDP stream.
    """
    if not enable_udp_stream or stream_failed:
        return

    open_stream_writer()
    if stream_writer is None:
        return

    if output.shape[1] != stream_width or output.shape[0] != stream_height:
        stream_image = cv2.resize(
            output,
            (stream_width, stream_height),
            interpolation=cv2.INTER_LINEAR,
        )
    else:
        stream_image = output

    stream_writer.write(np.ascontiguousarray(stream_image))

def get_contour_data(mask, out):
    """
    Return the centroid of the largest contour in
    the binary image 'mask' (the line) 
    and return the side in which the smaller contour is (the track mark) 
    (If there are any of these contours),
    and draw all contours on 'out' image
    """ 
    # get a list of contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # Filter contours by minimum area and calculate moments
    valid_contours = []
    for contour in contours:
        M = cv2.moments(contour)
        if M['m00'] > MIN_AREA:
            valid_contours.append((M['m00'], M, contour))

    # Sort contours by area descending: the largest is always the main track line
    valid_contours.sort(key=lambda x: x[0], reverse=True)

    line = {}
    mark = {}

    if len(valid_contours) > 0:
        area, M, contour = valid_contours[0]
        line['x'] = crop_w_start + int(M["m10"]/M["m00"])
        line['y'] = int(M["m01"]/M["m00"])
        
        # Plot the line area in light blue
        cv2.drawContours(out, [contour], -1, (255, 255, 0), 1)
        cv2.putText(out, f"Line:{int(area)}", (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])),
                    cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 255, 0), 2)

        # Any secondary contour closest to the robot is treated as a track mark
        for area, M, contour in valid_contours[1:]:
            if (not mark) or (int(M["m01"]/M["m00"]) > mark['y']):
                mark['y'] = int(M["m01"]/M["m00"])
                mark['x'] = crop_w_start + int(M["m10"]/M["m00"])
                
                # Plot the mark area in pink
                cv2.drawContours(out, [contour], -1, (255, 0, 255), 1)
                cv2.putText(out, f"Mark:{int(area)}", (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])),
                            cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 0, 255), 2)

    if mark and line:
        if mark['x'] > line['x']:
            mark_side = "right"
        else:
            mark_side = "left"
    else:
        mark_side = None

    return (line, mark_side)

def timer_callback():
    """
    Function to be called when the timer ticks.
    According to an image 'image_input', determine the speed of the robot
    so it can follow the contour
    """

    global error, last_error
    global image_input
    global just_seen_line
    global just_seen_right_mark
    global should_move
    global right_mark_count
    global finalization_countdown
    global publish_debug_image
    global publish_mask_image
    global show_debug_window
    global enable_udp_stream

    # Wait for the first image to be received
    if type(image_input) != np.ndarray:
        return

    height, width, _ = image_input.shape

    image = image_input.copy()

    global crop_w_start
    crop_h_start, crop_h_stop, crop_w_start, crop_w_stop = crop_size(height, width)

    # get the bottom part of the image (matrix slicing)
    crop = image[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop]
    
    # get a binary picture, where non-zero values represent the line.
    # (filter the color values so only the contour is seen)
    mask = cv2.inRange(crop, lower_bgr_values, upper_bgr_values)

    # get the centroid of the biggest contour in the picture,
    # and plot its detail on the cropped part of the output image
    output = image
    line, mark_side = get_contour_data(mask, output[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop])  
    # also get the side in which the track mark "is"
    
    message = Twist()
    
    if line:
    # if there even is a line in the image:
    # (as the camera could not be reading any lines)      
        x = line['x']

        # error:= The difference between the center of the image
        # and the center of the line
        error = x - width//2

        message.linear.x = LINEAR_SPEED
        just_seen_line = True

        # plot the line centroid on the image
        cv2.circle(output, (line['x'], crop_h_start + line['y']), 5, (0,255,0), 7)
        path_state = PATH_LINE_VISIBLE

    else:
        # There is no line in the image. 
        # Turn on the spot to find it again. 
        if just_seen_line:
            just_seen_line = False
            error = error * LOSS_FACTOR
        message.linear.x = 0.0
        path_state = PATH_LINE_LOST

    if mark_side != None:
        print("mark_side: {}".format(mark_side))

        if (mark_side == "right") and (finalization_countdown == None) and \
            (abs(error) <= MAX_ERROR) and (not just_seen_right_mark):

            right_mark_count += 1

            if right_mark_count > 1:
                # Start final countdown to stop the robot
                finalization_countdown = int(FINALIZATION_PERIOD / TIMER_PERIOD) + 1
                print("Finalization Process has begun!")

            
            just_seen_right_mark = True
    else:
        just_seen_right_mark = False

    
    # Determine the speed to turn and get the line in the center of the camera.
    if path_state == PATH_LINE_VISIBLE:
        # PD control when line is visible
        derivative = error - last_error
        last_error = error
        angular_z = float(error) * -KP + float(derivative) * -KD
        # Cap maximum angular velocity during tracking to prevent wild turns
        MAX_ANGULAR_VEL = 3.0
        message.angular.z = max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, angular_z))
    else:
        # Search mode when line is lost: spin slower on the spot
        derivative = 0
        last_error = error
        angular_z = float(error) * -KP
        # Cap searching velocity to a stable spin speed
        SEARCH_ANGULAR_VEL = 1.2
        message.angular.z = max(-SEARCH_ANGULAR_VEL, min(SEARCH_ANGULAR_VEL, angular_z))

    print("Error: {} | Angular Z: {}, ".format(error, message.angular.z))
    



    # Plot the boundaries where the image was cropped
    cv2.rectangle(output, (crop_w_start, crop_h_start), (crop_w_stop, crop_h_stop), (0,0,255), 2)
    update_fps()
    draw_debug_overlay(output, path_state, mark_side, message)

    # Uncomment to show the binary picture
    #cv2.imshow("mask", mask)

    publish_debug_outputs(output, mask)
    publish_stream(output)

    if show_debug_window:
        # Show the output image to the user when a local display is available.
        cv2.imshow("output", output)
        cv2.waitKey(5)

    # Check for final countdown
    if finalization_countdown != None:
        if finalization_countdown > 0:
            finalization_countdown -= 1

        elif finalization_countdown == 0:
            should_move = False


    # Publish the line-following candidate command.
    path_state_publisher.publish(String(data=path_state))

    if should_move:
        publisher.publish(message)
    else:
        empty_message = Twist()
        publisher.publish(empty_message)


def main():
    rclpy.init()
    global node, LINEAR_SPEED, KP
    node = Node('follower')

    node.declare_parameter('publish_debug_image', False)
    node.declare_parameter('publish_mask_image', False)
    node.declare_parameter('show_debug_window', False)
    node.declare_parameter('enable_udp_stream', False)
    node.declare_parameter('stream_host', "127.0.0.1")
    node.declare_parameter('stream_port', 5001)
    node.declare_parameter('stream_width', 960)
    node.declare_parameter('stream_height', 540)
    node.declare_parameter('stream_framerate', 30)
    node.declare_parameter('stream_bitrate', 4000000)
    node.declare_parameter('use_hw_encoder', True)
    node.declare_parameter('draw_fps', True)
    # Drive tuning (defaults match the real-robot constants; override in sim).
    node.declare_parameter('linear_speed', LINEAR_SPEED)
    node.declare_parameter('kp', KP)
    node.declare_parameter('kd', 0.0)
    LINEAR_SPEED = float(node.get_parameter('linear_speed').value)
    KP = float(node.get_parameter('kp').value)
    global KD
    KD = float(node.get_parameter('kd').value)

    global publish_debug_image
    publish_debug_image = bool(node.get_parameter('publish_debug_image').value)

    global publish_mask_image
    publish_mask_image = bool(node.get_parameter('publish_mask_image').value)

    global show_debug_window
    show_debug_window = bool(node.get_parameter('show_debug_window').value)

    global enable_udp_stream
    enable_udp_stream = bool(node.get_parameter('enable_udp_stream').value)

    global stream_host
    stream_host = node.get_parameter('stream_host').value

    global stream_port
    stream_port = int(node.get_parameter('stream_port').value)

    global stream_width
    stream_width = int(node.get_parameter('stream_width').value)

    global stream_height
    stream_height = int(node.get_parameter('stream_height').value)

    global stream_framerate
    stream_framerate = int(node.get_parameter('stream_framerate').value)

    global stream_bitrate
    stream_bitrate = int(node.get_parameter('stream_bitrate').value)

    global use_hw_encoder
    use_hw_encoder = bool(node.get_parameter('use_hw_encoder').value)

    global draw_fps
    draw_fps = bool(node.get_parameter('draw_fps').value)

    if not validate_stream_parameters():
        enable_udp_stream = False

    global publisher
    publisher = node.create_publisher(
        Twist,
        '/cmd_vel_line',
        rclpy.qos.qos_profile_system_default,
    )

    global path_state_publisher
    path_state_publisher = node.create_publisher(String, '/path_state', 10)

    global debug_image_publisher
    if publish_debug_image:
        debug_image_publisher = node.create_publisher(
            Image,
            '/follower/debug_image',
            10,
        )

    global mask_image_publisher
    if publish_mask_image:
        mask_image_publisher = node.create_publisher(
            Image,
            '/follower/mask_image',
            10,
        )

    subscription = node.create_subscription(
        Image,
        'camera/image_raw',
        image_callback,
        rclpy.qos.qos_profile_sensor_data,
    )

    # Run control loop directly inside image_callback to minimize latency
    # timer = node.create_timer(TIMER_PERIOD, timer_callback)

    start_service = node.create_service(
        Empty,
        'start_follower',
        start_follower_callback,
    )
    stop_service = node.create_service(
        Empty,
        'stop_follower',
        stop_follower_callback,
    )

    rclpy.spin(node)
    close_stream_writer()

try:
    main()
except (KeyboardInterrupt, rclpy.exceptions.ROSInterruptException):
    empty_message = Twist()
    publisher.publish(empty_message)
    close_stream_writer()

    node.destroy_node()
    rclpy.shutdown()
    exit()
    
