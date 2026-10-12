#!/usr/bin/env python3
"""
Detects the red cube in the overhead camera image and publishes its position
in the world frame (world == fr3_link0 in this setup) on /detected_cube.

Pipeline: red-pixel mask -> top-face centre (u, v) -> back-project through the
pinhole model onto the cube's top face -> (x, y) in world coordinates.

The camera is off to the side of the cube, so the mask contains the top face
plus a sliver of the side face that faces the image centre. A plain centroid is
therefore biased toward the image centre (~1 cm at x = 0.5). To remove that,
the top face is located from the mask edge that is FARTHEST from the image
centre (that edge always belongs to the top face) plus the known cube size.
This assumes the cube is axis-aligned with the belt, as it is in the sim.

No cv_bridge / OpenCV needed: the Gazebo image is raw rgb8, decoded with numpy.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image


# ---------------------------------------------------------------------------
# Must match models/overhead_camera/model.sdf and the camera pose in
# gazebo_franka_arm_with_conveyor.launch.py (CAM_XYZ).
# ---------------------------------------------------------------------------
CAM_X, CAM_Y, CAM_Z = 0.9, 0.0, 1.6     # camera position in world (looks straight down)
HFOV = 1.047                             # horizontal FOV [rad]

BELT_TOP_Z = 0.495                       # belt surface height
CUBE_SIZE = 0.06                         # cube edge length

MIN_PIXELS = 30                          # ignore blobs smaller than this

IMAGE_TOPIC = "/overhead_camera/image"
OUTPUT_TOPIC = "/detected_cube"
OUTPUT_FRAME = "world"                   # identical to fr3_link0 (zero offset)


def camera_scale(width):
    """Return (fx, depth_to_top_face). Square pixels, so fy == fx."""
    fx = (width / 2.0) / math.tan(HFOV / 2.0)
    depth = CAM_Z - (BELT_TOP_Z + CUBE_SIZE)
    return fx, depth


def pixel_to_world(u, v, width, height):
    """Back-project pixel (u, v) onto the cube's top face.

    Optical frame of the downward camera: image x -> world +X, image y -> world -Y,
    optical axis -> world -Z (verified with tf2_echo world overhead_camera_optical_frame).
    """
    fx, depth = camera_scale(width)
    x = CAM_X + (u - width / 2.0) / fx * depth
    y = CAM_Y - (v - height / 2.0) / fx * depth
    z = BELT_TOP_Z + CUBE_SIZE / 2.0            # cube centre height
    return x, y, z


def top_face_centre(idx, image_centre, top_face_px):
    """Centre of the top face along one image axis, from the pixel indices of the mask.

    The visible side face lies on the side of the blob facing the image centre, so
    the edge farthest from the image centre is the top face's own edge.
    """
    lo = float(idx.min())
    hi = float(idx.max()) + 1.0
    if (lo + hi) / 2.0 < image_centre:      # blob before the centre -> top face at the low end
        return lo + top_face_px / 2.0
    return hi - top_face_px / 2.0           # blob after the centre -> top face at the high end


def locate_cube(xs, ys, width, height):
    """(x, y, z) in world from the mask's pixel coordinates (column xs, row ys)."""
    fx, depth = camera_scale(width)
    top_face_px = CUBE_SIZE * fx / depth
    u = top_face_centre(xs, width / 2.0, top_face_px)
    v = top_face_centre(ys, height / 2.0, top_face_px)
    return pixel_to_world(u, v, width, height), (u, v)


class CubeDetector(Node):
    def __init__(self):
        super().__init__("cube_detector_node")
        self.pub = self.create_publisher(PointStamped, OUTPUT_TOPIC, 10)
        self.create_subscription(Image, IMAGE_TOPIC, self.on_image, 10)
        self.warned_encoding = False
        self.get_logger().info(f"Listening on {IMAGE_TOPIC}, publishing {OUTPUT_TOPIC}")

    def on_image(self, msg: Image):
        if msg.encoding not in ("rgb8", "bgr8"):
            if not self.warned_encoding:
                self.get_logger().error(f"Unsupported image encoding '{msg.encoding}'")
                self.warned_encoding = True
            return

        img = np.frombuffer(msg.data, dtype=np.uint8)
        img = img.reshape(msg.height, msg.step)[:, : msg.width * 3]
        img = img.reshape(msg.height, msg.width, 3).astype(np.int32)
        if msg.encoding == "bgr8":
            img = img[..., ::-1]
        r, g, b = img[..., 0], img[..., 1], img[..., 2]

        # "Red" relative to the other channels, so shading doesn't break it.
        mask = (r > 100) & (r > 1.8 * g) & (r > 1.8 * b)
        ys, xs = np.nonzero(mask)
        if xs.size < MIN_PIXELS:
            self.get_logger().info("No cube in view", throttle_duration_sec=2.0)
            return

        (x, y, z), (u, v) = locate_cube(xs, ys, msg.width, msg.height)

        out = PointStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = OUTPUT_FRAME
        out.point.x, out.point.y, out.point.z = float(x), float(y), float(z)
        self.pub.publish(out)

        self.get_logger().info(
            f"cube at x={x:.3f} y={y:.3f} z={z:.3f}  (pixel {u:.0f},{v:.0f}, {xs.size} px)",
            throttle_duration_sec=1.0,
        )


def main():
    rclpy.init()
    node = CubeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
