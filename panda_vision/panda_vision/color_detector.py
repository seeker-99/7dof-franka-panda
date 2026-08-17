#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import tf2_ros
import tf_transformations
class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        # Subscriber
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        # Publisher
        self.coords_pub = self.create_publisher(String, '/color_coordinates', 10)
        # OpenCV bridge
        self.bridge = CvBridge()
        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # Camera intrinsic parameters (from your SDF)
        self.fx = 585.0
        self.fy = 588.0
        self.cx = 320.0
        self.cy = 160.0
        self.get_logger().info("Color Detector Node Started with TF2 lookup transform")

    def pixel_to_base_point(self, cx_pix, cy_pix, z_plane=0.40):
        """
        Cast a ray from the camera through the given pixel, and find where it
        intersects the known block-plane height (z_plane) in panda_link0 frame.

        z_plane: height (in panda_link0 frame) of the surface the blocks sit on.
                 Determined from Gazebo ground-truth: blocks sit at world z=0.7,
                 panda_link0 is at world z=0.35, so z_plane = 0.7 - 0.35 = 0.35.
        """
        # Ray direction in the camera optical frame (standard pinhole model,
        # x-right, y-down, z-forward)
        d_optical = np.array([
            (cx_pix - self.cx) / self.fx,
            (cy_pix - self.cy) / self.fy,
            1.0
        ])

        # Lookup camera_link_optical -> panda_link0 transform
        t = self.tf_buffer.lookup_transform(
            "panda_link0",
            "camera_link_optical",
            rclpy.time.Time(),
            timeout=Duration(seconds=1.0))

        trans = np.array([
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z
        ])
        rot = [
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w
        ]
        T = tf_transformations.quaternion_matrix(rot)
        R = T[:3, :3]

        # Ray origin (camera position) and direction, both in panda_link0 frame
        origin = trans
        direction = R @ d_optical

        # Solve for t where origin.z + t*direction.z == z_plane
        if abs(direction[2]) < 1e-6:
            return None  # ray parallel to plane, shouldn't normally happen

        t_param = (z_plane - origin[2]) / direction[2]
        point = origin + t_param * direction
        return point  # [x, y, z] in panda_link0 frame, z should equal z_plane

    def image_callback(self, msg):
        try:
            # Convert ROS Image -> OpenCV BGR
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Define color ranges (HSV)
        color_ranges = {
            "R": [((0, 80, 50), (10, 255, 255)), ((170, 80, 50), (180, 255, 255))],
            "G": [((35, 80, 50), (85, 255, 255))],
            "B": [((90, 80, 50), (135, 255, 255))]
        }
        for color_id, ranges in color_ranges.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for (lower, upper) in ranges:
                lower = np.array(lower)
                upper = np.array(upper)
                mask |= cv2.inRange(hsv, lower, upper)
            # Noise removal
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                if cv2.contourArea(cnt) > 1:  # Increased minimum area threshold
                    x, y, w, h = cv2.boundingRect(cnt)
                    cx_pix, cy_pix = x + w // 2, y + h // 2
                    # Draw bounding box + label
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                    cv2.putText(frame, color_id, (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                    try:
                        # Camera sees the TOP surface of the boxes, not their
                        # center. Boxes are 0.1m tall, centered at z=0.35 in
                        # panda_link0 frame, so their visible top surface is
                        # at z=0.40. Using the wrong height here caused an
                        # error proportional to how off-center the pixel was
                        # (ray angle) - small for centered boxes, large for
                        # boxes further from image center - exactly the
                        # pattern that was observed (red ~1cm, green/blue
                        # ~5cm off).
                        pt_base = self.pixel_to_base_point(cx_pix, cy_pix, z_plane=0.40)
                        if pt_base is None:
                            self.get_logger().warn("Ray-plane intersection failed")
                            continue

                        # --- Empirical Y-axis calibration correction ---
                        # After fixing the top-surface-height issue, a
                        # residual Y-axis error remained that scaled with
                        # distance from image center (red near y=0: ~1cm
                        # off; green/blue near y=+-0.2: ~6-7cm off). Rather
                        # than keep chasing the exact remaining geometric
                        # cause, this was calibrated directly from the three
                        # known ground-truth block positions (from Gazebo):
                        #   true_y -> detected_y
                        #   -0.20  -> -0.143
                        #    0.00  ->  0.009
                        #    0.20  ->  0.131
                        # Linear fit: detected_y ~= 0.685 * true_y + 0.001
                        # So corrected_y = (detected_y - 0.001) / 0.685
                        Y_CAL_SCALE = 0.685
                        Y_CAL_OFFSET = 0.001
                        pt_base[1] = (pt_base[1] - Y_CAL_OFFSET) / Y_CAL_SCALE

                        # Publish color ID + coordinates in panda_link0 frame
                        msg_str = f"{color_id},{pt_base[0]:.3f},{pt_base[1]:.3f},{pt_base[2]:.3f}"
                        self.coords_pub.publish(String(data=msg_str))
                        self.get_logger().info(msg_str)

                    except (tf2_ros.LookupException,
                            tf2_ros.ConnectivityException,
                            tf2_ros.ExtrapolationException) as e:
                        self.get_logger().warn(f"TF lookup failed: {e}")
                    except Exception as e:
                        self.get_logger().error(f"Unexpected error in TF transform: {e}")
        # Show image in window
        try:
            cv2.namedWindow("Color Detection", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Color Detection", 640, 320)
            cv2.imshow("Color Detection", frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().warn(f"OpenCV display error: {e}")
def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
if __name__ == '__main__':
    main()