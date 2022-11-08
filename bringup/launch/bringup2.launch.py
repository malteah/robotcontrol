import os
import json
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    frames_path = "/home/student/Desktop/scenario/frames2/"
    rviz_config_file = "/home/student/Desktop/scenario/config/config2.rviz"
    dir = FindPackageShare("simple_robot_simulator_bringup").find(
        "simple_robot_simulator_bringup"
    )
    urc_setup_dir = FindPackageShare("ur_controller").find("ur_controller")

    r1_ur_controller_params = {
        "simple_robot_simulator": True,
        "templates_path": os.path.join(urc_setup_dir),
        "prefix": "r1_",
    }

    r2_ur_controller_params = {
        "simple_robot_simulator": True,
        "templates_path": os.path.join(urc_setup_dir),
        "prefix": "r2_",
    }

    parameters = {"scenario_path": frames_path}

    r1_robot_parameters_path = os.path.join(dir, "robots", "ursim10e", "general.json")

    with open(r1_robot_parameters_path) as jsonfile:
        r1_robot_parameters = json.load(jsonfile)

    r2_robot_parameters_path = os.path.join(dir, "robots", "ursim10e", "general.json")

    with open(r2_robot_parameters_path) as jsonfile:
        r2_robot_parameters = json.load(jsonfile)

    r1_declared_arguments = []

    r1_declared_arguments.append(
        DeclareLaunchArgument(
            "r1_ur_type",
            default_value=r1_robot_parameters["ur_type"],
            description="Type/series of used UR robot.",
        )
    )

    r1_declared_arguments.append(
        DeclareLaunchArgument(
            "r1_safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    r1_declared_arguments.append(
        DeclareLaunchArgument(
            "r1_safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    r1_declared_arguments.append(
        DeclareLaunchArgument(
            "r1_safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    r1_declared_arguments.append(
        DeclareLaunchArgument(
            "r1_description_package",
            default_value="ur_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    r1_declared_arguments.append(
        DeclareLaunchArgument(
            "r1_description_file",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    r1_declared_arguments.append(
        DeclareLaunchArgument(
            "r1_prefix",
            default_value="r1_",  # robot_parameters["prefix"],
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    r1_declared_arguments.append(
        DeclareLaunchArgument(
            "ghost_prefix",
            default_value="ghost_",  # robot_parameters["prefix"],
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )

    ur_type = LaunchConfiguration("r1_ur_type")
    safety_limits = LaunchConfiguration("r1_safety_limits")
    safety_pos_margin = LaunchConfiguration("r1_safety_pos_margin")
    safety_k_position = LaunchConfiguration("r1_safety_k_position")
    description_package = LaunchConfiguration("r1_description_package")
    description_file = LaunchConfiguration("r1_description_file")
    prefix = LaunchConfiguration("r1_prefix")
    # ghost_prefix = LaunchConfiguration("ghost_prefix")

    joint_limit_params = PathJoinSubstitution(
        [
            os.path.join(dir, "robots"),
            r1_robot_parameters["name"],
            "joint_limits.yaml",
        ]
    )
    kinematics_params = PathJoinSubstitution(
        [
            os.path.join(dir, "robots"),
            r1_robot_parameters["name"],
            "default_kinematics.yaml",
        ]
    )
    physical_params = PathJoinSubstitution(
        [
            os.path.join(dir, "robots"),
            r1_robot_parameters["name"],
            "physical_parameters.yaml",
        ]
    )
    visual_params = PathJoinSubstitution(
        [
            os.path.join(dir, "robots"),
            r1_robot_parameters["name"],
            "visual_parameters.yaml",
        ]
    )
    script_filename = PathJoinSubstitution(
        [FindPackageShare(description_package), "resources", "ros_control.urscript"]
    )
    input_recipe_filename = PathJoinSubstitution(
        [FindPackageShare(description_package), "resources", "rtde_input_recipe.txt"]
    )
    output_recipe_filename = PathJoinSubstitution(
        [FindPackageShare(description_package), "resources", "rtde_output_recipe.txt"]
    )

    r1_robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            ur_type,
            " ",
            "script_filename:=",
            script_filename,
            " ",
            "input_recipe_filename:=",
            input_recipe_filename,
            " ",
            "output_recipe_filename:=",
            output_recipe_filename,
            " ",
            "prefix:=",
            prefix,
        ]
    )

    r1_robot_description = {"robot_description": r1_robot_description_content}

    r1_robot_parameters = {
        "prefix": "r1_",
        "urdf_raw": r1_robot_description_content,
        "initial_joint_state": [
            "0.0",
            "-1.5707",
            "1.5707",
            "-1.5707",
            "-1.5707",
            "0.0",
        ],
        "initial_base_link_id": "base",
        "initial_face_plate_id": "tool0",
        "initial_tcp_id": "svt_tcp",
    }

    r2_declared_arguments = []

    r2_declared_arguments.append(
        DeclareLaunchArgument(
            "r2_ur_type",
            default_value=r2_robot_parameters["ur_type"],
            description="Type/series of used UR robot.",
        )
    )

    r2_declared_arguments.append(
        DeclareLaunchArgument(
            "r2_safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    r2_declared_arguments.append(
        DeclareLaunchArgument(
            "r2_safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    r2_declared_arguments.append(
        DeclareLaunchArgument(
            "r2_safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    r2_declared_arguments.append(
        DeclareLaunchArgument(
            "r2_description_package",
            default_value="ur_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    r2_declared_arguments.append(
        DeclareLaunchArgument(
            "r2_description_file",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    r2_declared_arguments.append(
        DeclareLaunchArgument(
            "r2_prefix",
            default_value="r2_",  # robot_parameters["prefix"],
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    r2_declared_arguments.append(
        DeclareLaunchArgument(
            "r2_ghost_prefix",
            default_value="ghost_",  # robot_parameters["prefix"],
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )

    ur_type = LaunchConfiguration("r2_ur_type")
    safety_limits = LaunchConfiguration("r2_safety_limits")
    safety_pos_margin = LaunchConfiguration("r2_safety_pos_margin")
    safety_k_position = LaunchConfiguration("r2_safety_k_position")
    description_package = LaunchConfiguration("r2_description_package")
    description_file = LaunchConfiguration("r2_description_file")
    prefix = LaunchConfiguration("r2_prefix")
    # ghost_prefix = LaunchConfiguration("ghost_prefix")

    joint_limit_params = PathJoinSubstitution(
        [
            os.path.join(dir, "robots"),
            r2_robot_parameters["name"],
            "joint_limits.yaml",
        ]
    )
    kinematics_params = PathJoinSubstitution(
        [
            os.path.join(dir, "robots"),
            r2_robot_parameters["name"],
            "default_kinematics.yaml",
        ]
    )
    physical_params = PathJoinSubstitution(
        [
            os.path.join(dir, "robots"),
            r2_robot_parameters["name"],
            "physical_parameters.yaml",
        ]
    )
    visual_params = PathJoinSubstitution(
        [
            os.path.join(dir, "robots"),
            r2_robot_parameters["name"],
            "visual_parameters.yaml",
        ]
    )
    script_filename = PathJoinSubstitution(
        [FindPackageShare(description_package), "resources", "ros_control.urscript"]
    )
    input_recipe_filename = PathJoinSubstitution(
        [FindPackageShare(description_package), "resources", "rtde_input_recipe.txt"]
    )
    output_recipe_filename = PathJoinSubstitution(
        [FindPackageShare(description_package), "resources", "rtde_output_recipe.txt"]
    )

    r2_robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            ur_type,
            " ",
            "script_filename:=",
            script_filename,
            " ",
            "input_recipe_filename:=",
            input_recipe_filename,
            " ",
            "output_recipe_filename:=",
            output_recipe_filename,
            " ",
            "prefix:=",
            prefix,
        ]
    )

    r2_robot_description = {"robot_description": r2_robot_description_content}

    r2_robot_parameters = {
        "prefix": "r2_",
        "urdf_raw": r2_robot_description_content,
        "initial_joint_state": [
            "0.0",
            "-1.5707",
            "1.5707",
            "-1.5707",
            "-1.5707",
            "0.0",
        ],
        "initial_base_link_id": "base",
        "initial_face_plate_id": "tool0",
        "initial_tcp_id": "svt_tcp",
    }

    r1_robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="r1",
        output="screen",
        parameters=[r1_robot_description],
        # remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    r1_simple_robot_simulator_node = Node(
        package="simple_robot_simulator",
        executable="simple_robot_simulator",
        namespace="r1",
        output="screen",
        parameters=[r1_robot_parameters],
        # remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    r2_robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="r2",
        output="screen",
        parameters=[r2_robot_description],
        # remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    r2_simple_robot_simulator_node = Node(
        package="simple_robot_simulator",
        executable="simple_robot_simulator",
        namespace="r2",
        output="screen",
        parameters=[r2_robot_parameters],
        # remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    sms_node = Node(
        package="scene_manipulation_service",
        executable="sms",
        namespace="",
        output="screen",
        parameters=[parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace="",
        output="screen",
        arguments=["-d", rviz_config_file],
        emulate_tty=True,
    )

    sms_gui_node = Node(
        package="scene_manipulation_gui",
        executable="scene_manipulation_gui",
        namespace="",
        output="screen",
        parameters=[parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    sms_marker_node = Node(
        package="scene_manipulation_marker",
        executable="scene_manipulation_marker",
        namespace="",
        output="screen",
        parameters=[parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace="",
        output="screen",
        arguments=["-d", rviz_config_file],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    r1_ur_controller_node = Node(
        package="ur_controller",
        executable="ur_controller",
        namespace="r1",
        output="screen",
        parameters=[r1_ur_controller_params],
        emulate_tty=True,
    )

    r2_ur_controller_node = Node(
        package="ur_controller",
        executable="ur_controller",
        namespace="r2",
        output="screen",
        parameters=[r2_ur_controller_params],
        emulate_tty=True,
    )

    control_gui_node = Node(
        package="gui_tools",
        executable="control_gui",
        namespace="",
        output="screen",
        parameters=[],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    visualization_node = Node(
        package="visualization_server",
        executable="visualization_server",
        namespace="",
        output="screen",
        parameters=[],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    nodes_to_start = [
        r1_robot_state_publisher_node,
        r1_simple_robot_simulator_node,
        r1_ur_controller_node,
        r2_robot_state_publisher_node,
        r2_simple_robot_simulator_node,
        r2_ur_controller_node,
        rviz_node,
        sms_node,
        # sms_gui_node, # won't really need it for a4
        sms_marker_node,
        # control_gui_node,
        visualization_node,
    ]

    return LaunchDescription(r1_declared_arguments + r2_declared_arguments + nodes_to_start)
