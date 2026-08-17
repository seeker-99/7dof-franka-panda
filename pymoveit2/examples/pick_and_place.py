#!/usr/bin/env python3
"""
Pick and place node combining Cartesian and joint-space moves with smooth joint transitions.
Locks the detected color coordinates before starting the motion.

ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=R
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=G
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=B

"""

from threading import Thread, Event
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from std_msgs.msg import String
import tf2_ros

from pymoveit2 import MoveIt2, GripperInterface
from pymoveit2.robots import panda

import math
import time


class PickAndPlace(Node):
    def __init__(self):
        super().__init__("pick_and_place")

        # Parameters
        self.declare_parameter("target_color", "R")
        self.target_color = self.get_parameter("target_color").value.upper()

        # --- Grasp geometry parameters ---
        # /color_coordinates now publishes the REAL block position in the
        # panda_link0 frame (computed via camera ray / table-plane
        # intersection in color_detector.py). These offsets are small and
        # physically meaningful, unlike the old hardcoded -0.60 hack that
        # was only compensating for broken vision math.

        # How far above the block to hover before/after grasping (safe
        # approach height, avoids clipping other objects on the way in/out).
        self.declare_parameter("hover_height", 0.15)
        self.hover_height = float(self.get_parameter("hover_height").value)

        # Vertical offset applied to the detected block position for the
        # actual grasp pose. 0.0 means "close the gripper exactly at the
        # detected height". Increase slightly (e.g. 0.01-0.03) if the
        # gripper is grasping too low / colliding with the table; decrease
        # (negative) if it's grasping too high and missing the block.
        self.declare_parameter("grasp_z_offset", 0.0)
        self.grasp_z_offset = float(self.get_parameter("grasp_z_offset").value)

        # Flags
        self.already_moved = False
        self.ready = False  # becomes True only after the initial start_joints
                             # move fully completes; coords_callback must not
                             # touch self.moveit2 before that, or pymoveit2's
                             # internal executor spin (used by
                             # wait_until_executed) can be re-entered from a
                             # second thread, corrupting its internal state.
        self.target_coords = None  # Stores the locked coordinates
        self.target_locked_event = Event()  # Signals main thread that a
                                             # target has been locked and the
                                             # pick sequence should begin.
                                             # The actual MoveIt calls run in
                                             # the MAIN thread (see main()),
                                             # never inside this callback -
                                             # doing blocking pymoveit2 calls
                                             # from within a callback that's
                                             # running on the SAME executor
                                             # that is spinning it corrupts
                                             # the action client's internal
                                             # wait-set (this was the actual
                                             # cause of both the "Executor is
                                             # already spinning" error and
                                             # the later "wait set index ...
                                             # out of bounds" RCLError).

        self.callback_group = ReentrantCallbackGroup()

        # Arm MoveIt2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=panda.joint_names(),
            base_link_name=panda.base_link_name(),
            end_effector_name=panda.end_effector_name(),
            group_name=panda.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )

        # Set lower velocity & acceleration for smoother motion
        self.moveit2.max_velocity = 0.1
        self.moveit2.max_acceleration = 0.1

        # Gripper interface
        self.gripper = GripperInterface(
            node=self,
            gripper_joint_names=panda.gripper_joint_names(),
            open_gripper_joint_positions=panda.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=panda.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=panda.MOVE_GROUP_GRIPPER,
            callback_group=self.callback_group,
            gripper_command_action_name="gripper_action_controller/gripper_cmd",
        )

        # Subscriber
        self.sub = self.create_subscription(
            String, "/color_coordinates", self.coords_callback, 10
        )
        self.get_logger().info(f"Waiting for {self.target_color} from /color_coordinates...")

        # TF listener - used only for debug logging of the actual
        # end-effector pose during the pick sequence, to compare against
        # the commanded hover/grasp positions.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Predefined joint positions (in radians)
        # NOTE: panda_joint4's valid range is roughly [-3.07, -0.07] rad
        # (see safety_controller soft limits in arm.xacro) - 0.0 is OUTSIDE
        # this range and will make every planning request fail with
        # "Start state out of bounds" / generic FAILURE. Use -90deg like
        # home_joints, which is confirmed valid.
        self.start_joints = [0.0, 0.0, 0.0, math.radians(-90.0), 0.0, 0.0, math.radians(-125.0)]
        self.home_joints  = [0.0, 0.0, 0.0, math.radians(-90.0), 0.0, math.radians(92.0), math.radians(50.0)]
        self.drop_joints  = [math.radians(-155.0), math.radians(30.0), math.radians(-20.0),
                             math.radians(-124.0), math.radians(44.0), math.radians(163.0), math.radians(7.0)]

        # Move to start joint configuration
        self.moveit2.move_to_configuration(self.start_joints)
        self.moveit2.wait_until_executed()
        self.ready = True
        self.get_logger().info("Initial move to start_joints complete - ready for detections.")

    def coords_callback(self, msg):
        if not self.ready:
            return  # Ignore detections until the initial start_joints move
                     # has fully completed - see comment in __init__.
        if self.already_moved:
            return  # Ignore messages once motion starts

        try:
            color_id, x, y, z = msg.data.split(",")
            color_id = color_id.strip().upper()

            if color_id == self.target_color:
                # Lock coordinates immediately. IMPORTANT: this callback
                # does NOT call any self.moveit2 / self.gripper methods -
                # see the comment on target_locked_event in __init__ for
                # why. It only stores data and signals the main thread.
                self.target_coords = [float(x), float(y), float(z)]
                self.get_logger().info(
                    f"Target {self.target_color} locked at: "
                    f"[{self.target_coords[0]:.3f}, {self.target_coords[1]:.3f}, {self.target_coords[2]:.3f}]"
                )
                self.already_moved = True
                self.target_locked_event.set()

        except Exception as e:
            self.get_logger().error(f"Error parsing /color_coordinates: {e}")

    def log_actual_ee_pose(self, label):
        """Debug helper: logs the ACTUAL current pose of panda_hand and
        panda_leftfinger (via TF) so it can be compared against the
        commanded hover_position / grasp_position. This tells us whether
        any mismatch is in X/Y alignment, Z height, or both."""
        try:
            t_hand = self.tf_buffer.lookup_transform(
                "panda_link0", "panda_hand", rclpy.time.Time(),
                timeout=Duration(seconds=1.0))
            t_finger = self.tf_buffer.lookup_transform(
                "panda_link0", "panda_leftfinger", rclpy.time.Time(),
                timeout=Duration(seconds=1.0))
            self.get_logger().info(
                f"[DEBUG {label}] panda_hand actual pose: "
                f"x={t_hand.transform.translation.x:.3f}, "
                f"y={t_hand.transform.translation.y:.3f}, "
                f"z={t_hand.transform.translation.z:.3f}"
            )
            self.get_logger().info(
                f"[DEBUG {label}] panda_leftfinger actual pose: "
                f"x={t_finger.transform.translation.x:.3f}, "
                f"y={t_finger.transform.translation.y:.3f}, "
                f"z={t_finger.transform.translation.z:.3f}"
            )
        except Exception as e:
            self.get_logger().warn(f"[DEBUG {label}] TF lookup failed: {e}")

    def get_actual_hand_z(self):
        """Returns the current panda_hand Z position in panda_link0 frame,
        or None if TF lookup fails."""
        try:
            t = self.tf_buffer.lookup_transform(
                "panda_link0", "panda_hand", rclpy.time.Time(),
                timeout=Duration(seconds=1.0))
            return t.transform.translation.z
        except Exception:
            return None

    def run_pick_sequence(self):
        """Runs the full pick-and-place motion sequence. MUST be called from
        the main thread (see main()), never from inside a subscription
        callback - pymoveit2's blocking wait_until_executed() calls do
        internal spinning that is not safe to nest inside a callback that is
        already running on the same executor that is spinning it."""

        bx, by, bz = self.target_coords

        # Hover pose: directly above the block, safe height
        hover_position = [bx, by, bz + self.hover_height]

        # Grasp pose: at (or very near) the block's actual height
        grasp_position = [bx, by, bz + self.grasp_z_offset]

        self.get_logger().info(
            f"[DEBUG] Detected block at x={bx:.3f}, y={by:.3f}, z={bz:.3f}"
        )
        self.get_logger().info(
            f"[DEBUG] Commanding hover_position="
            f"[{hover_position[0]:.3f}, {hover_position[1]:.3f}, {hover_position[2]:.3f}]"
        )
        self.get_logger().info(
            f"[DEBUG] Commanding grasp_position="
            f"[{grasp_position[0]:.3f}, {grasp_position[1]:.3f}, {grasp_position[2]:.3f}]"
        )

        quat_xyzw = [0.0, 1.0, 0.0, 0.0]  # gripper pointing straight down

        # --- Pick-and-place sequence ---

        # 1. Move to home joint configuration
        self.moveit2.move_to_configuration(self.home_joints)
        self.moveit2.wait_until_executed()

        # 2. Move above target (Cartesian)
        self.moveit2.move_to_pose(position=hover_position, quat_xyzw=quat_xyzw)
        self.moveit2.wait_until_executed()
        self.log_actual_ee_pose("after hover move")

        # 3. Open gripper
        self.gripper.open()
        self.gripper.wait_until_executed()

        # 4. Move down to the block to grasp it - VERIFY it actually got
        # there before proceeding. A Cartesian move can silently achieve
        # only a small fraction of the path (e.g. because of an IK issue
        # partway down) without wait_until_executed() raising any error,
        # which previously caused the gripper to close far above the block
        # instead of at grasp height. We now check the real height via TF
        # and retry - first with more Cartesian attempts, then falling back
        # to a regular (non-Cartesian) planned move - until it's actually
        # close to the target height, or we give up and abort loudly.
        height_tolerance = 0.03  # 3cm
        reached_grasp_height = False

        for attempt in range(4):
            if attempt < 2:
                self.moveit2.move_to_pose(
                    position=grasp_position,
                    quat_xyzw=quat_xyzw,
                    cartesian=True
                )
            else:
                # Fall back to a regular planned move (non-Cartesian) -
                # sometimes succeeds where a Cartesian path fails.
                self.get_logger().warn(
                    "[DEBUG] Cartesian descent didn't reach target height, "
                    "falling back to regular planned move..."
                )
                self.moveit2.move_to_pose(
                    position=grasp_position,
                    quat_xyzw=quat_xyzw,
                    cartesian=False
                )
            self.moveit2.wait_until_executed()

            actual_z = self.get_actual_hand_z()
            if actual_z is not None:
                self.get_logger().info(
                    f"[DEBUG] Grasp descent attempt {attempt+1}: "
                    f"target hand z={grasp_position[2]:.3f}, actual z={actual_z:.3f}"
                )
                if abs(actual_z - grasp_position[2]) <= height_tolerance:
                    reached_grasp_height = True
                    break
            else:
                self.get_logger().warn("[DEBUG] Could not read actual hand height via TF")

        self.log_actual_ee_pose("after grasp descent")

        if not reached_grasp_height:
            self.get_logger().error(
                "[DEBUG] Failed to reach grasp height after all attempts - "
                "aborting pick to avoid closing the gripper on nothing."
            )
            self.get_logger().info("Pick-and-place sequence complete.")
            return

        time.sleep(1.0)  # let physics settle before closing

        # 5. Close gripper
        self.gripper.close()
        self.gripper.wait_until_executed()

        # 6. Lift back up to hover height
        self.moveit2.move_to_pose(position=hover_position, quat_xyzw=quat_xyzw)
        self.moveit2.wait_until_executed()

        # 7. Move to home joint configuration
        self.moveit2.move_to_configuration(self.home_joints)
        self.moveit2.wait_until_executed()

        # 8. Move to drop joint configuration
        self.moveit2.move_to_configuration(self.drop_joints)
        self.moveit2.wait_until_executed()

        # 9. Open gripper to release
        self.gripper.open()
        self.gripper.wait_until_executed()

        # 10. Close gripper
        self.gripper.close()
        self.gripper.wait_until_executed()

        # 11. Return to start joint configuration
        self.moveit2.move_to_configuration(self.start_joints)
        self.moveit2.wait_until_executed()

        self.get_logger().info("Pick-and-place sequence complete.")


def main():
    rclpy.init()
    node = PickAndPlace()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        # Wait here, in the MAIN thread, until coords_callback (running on
        # the executor thread) locks a target. Once it does, run the whole
        # pick sequence here in the main thread - this avoids nesting
        # pymoveit2's blocking calls inside a callback that's running on the
        # same executor that's spinning it (see comments in __init__ and
        # run_pick_sequence for why that corrupts the action client).
        node.target_locked_event.wait()
        node.run_pick_sequence()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        executor_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()