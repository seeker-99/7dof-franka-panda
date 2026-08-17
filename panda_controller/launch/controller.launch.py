import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import UnlessCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    is_sim = LaunchConfiguration("is_sim")

    is_sim_arg = DeclareLaunchArgument("is_sim", default_value="True")

    ros_distro = os.environ.get("ROS_DISTRO", "humble")
    is_ignition = "True" if ros_distro == "humble" else "False"

    robot_description = ParameterValue(
        Command([
            "xacro ",
            os.path.join(get_package_share_directory("panda_description"), "urdf", "panda.urdf.xacro"),
            " is_sim:=True",
            " is_ignition:=", is_ignition
        ]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": is_sim}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": is_sim},
            os.path.join(get_package_share_directory("panda_controller"), "config", "panda_controllers.yaml"),
        ],
        condition=UnlessCondition(is_sim),
    )

    # --- Retry-loop based controller activation ---
    # Instead of guessing a fixed startup delay (which is unreliable because
    # Gazebo's warm-up time varies a lot run to run, especially under WSL2),
    # each spawner keeps retrying "load + activate" every few seconds until
    # it actually succeeds. This eliminates the "Switch controller timed out"
    # race that happens when a spawner fires before the hardware interface
    # is fully ready.

    def make_retrying_controller(controller_name, max_attempts=100, retry_delay=5, call_timeout=30):
        # Retries the FULL load+configure+activate sequence together each
        # attempt. Each individual ros2 control call is wrapped in `timeout`
        # so a slow/stuck call fails fast instead of eating the whole retry
        # budget on one attempt. call_timeout is generous (30s) because on
        # this system, under heavy load, even a single `ros2 control ...`
        # CLI invocation can itself take a long time just to start up and
        # discover the ROS graph over DDS - independent of whether
        # controller_manager itself is ready.
        cmd = (
            f"for i in $(seq 1 {max_attempts}); do "
            f"timeout {call_timeout} ros2 control load_controller {controller_name} "
            f"--controller-manager /controller_manager; "
            f"timeout {call_timeout} ros2 control set_controller_state {controller_name} inactive "
            f"--controller-manager /controller_manager; "
            f"timeout {call_timeout} ros2 control set_controller_state {controller_name} active "
            f"--controller-manager /controller_manager && exit 0; "
            f"echo '[{controller_name}] attempt '$i' failed, retrying in {retry_delay}s...'; "
            f"sleep {retry_delay}; "
            f"done; "
            f"echo '[{controller_name}] FAILED after {max_attempts} attempts'; exit 1"
        )
        return ExecuteProcess(
            cmd=["bash", "-c", cmd],
            output="screen",
        )

    jsb_start = make_retrying_controller("joint_state_broadcaster")
    arm_start = make_retrying_controller("arm_controller")
    gripper_start = make_retrying_controller("gripper_controller")

    # Larger head start before the first attempt - on this system Gazebo
    # itself can take well over a minute to spawn the robot entity and
    # bring controller_manager's hardware interface online under load, so a
    # short delay just means the first several attempts are guaranteed
    # failures that clutter the log without helping.
    delayed_jsb_start = TimerAction(period=30.0, actions=[jsb_start])

    jsb_to_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_start,
            on_exit=[arm_start],
        )
    )

    arm_to_gripper = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_start,
            on_exit=[gripper_start],
        )
    )

    return LaunchDescription([
        is_sim_arg,
        robot_state_publisher_node,
        controller_manager,
        delayed_jsb_start,
        jsb_to_arm,
        arm_to_gripper,
    ])