#!/usr/bin/python3
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, RegisterEventHandler, TimerAction, SetEnvironmentVariable
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import ExecuteProcess
from launch.actions import ExecuteProcess

import xacro
import yaml

def load_yaml(package_name, file_path):
    pkg = get_package_share_directory(package_name)
    with open(os.path.join(pkg, file_path), 'r') as f:
        return yaml.safe_load(f)

def spawn_crane(xacro_file, robot_name="robot_2"):
    tf_prefix = robot_name + "_"
    robot_description_topic = f"{robot_name}/robot_description"
    robot_state_publisher_name = robot_name + "_robot_state_publisher"
    joint_state_topic = f"{robot_name}/joint_states"

    # robot_description from xacro
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc, mappings={"robot_name": robot_name, "tf_prefix": tf_prefix})
    robot_description = {'robot_description': doc.toxml()}

    # Pose
    position = [0.0, 0.0, 0.0]
    rpy = [0.0, 0.0, 0.0]

    spawn_entity_2 = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', robot_description_topic,
                   '-x', str(position[0]), '-y', str(position[1]), '-z', str(position[2]),
                   '-R', str(rpy[0]), '-P', str(rpy[1]), '-Y', str(rpy[2]),
                   '-robot_namespace', robot_name,
                   '-entity', robot_name],
        output='both'
    )

    # robot_2_controllers = PathJoinSubstitution(
    # [FindPackageShare('multi_robot_sim'), 'config', 'robot_2_controllers.yaml']
    # )

    state_pub_2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=robot_state_publisher_name,
        namespace=robot_name,
        output='both',
        parameters=[
            robot_description,
            {"use_sim_time": True}
        ],
        remappings=[("/robot_description", robot_description_topic),
                    ("/joint_states", joint_state_topic)
            ]
    )


    return [
    spawn_entity_2,
   
]


def spawn_mobile(xacro_file, robot_name="robot_1"):
    tf_prefix = robot_name + "_"
    robot_description_topic = f"{robot_name}/robot_description"
    joint_state_topic = f"{robot_name}/joint_states"

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc, mappings={"robot_name": robot_name, "tf_prefix": tf_prefix})
    robot_description = {'robot_description': doc.toxml()}

    position = [-5, -5.0, 1.0]
    rpy = [0.0, 0.0, 0.0]

    spawn_entity = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', robot_description_topic,
                   '-x', str(position[0]), '-y', str(position[1]), '-z', str(position[2]),
                   '-R', str(rpy[0]), '-P', str(rpy[1]), '-Y', str(rpy[2]),
                   '-robot_namespace', robot_name,
                   '-entity', robot_name],
        output='both'
    )

    state_pub = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name=f'{robot_name}_robot_state_publisher', namespace=robot_name, output='both',
        parameters=[robot_description, {"use_sim_time": True}],
        remappings=[("/robot_description", robot_description_topic),
                    ("/joint_states", joint_state_topic)]
    )

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare('multi_robot_sim'), 'config', 'robot_1_controllers.yaml']
    )

    js_broadcaster = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", f"{robot_name}/controller_manager"],
    )
    joint_traj_ctrl = Node(
        package="controller_manager", executable="spawner",
        arguments=["ur_controller", "-c", f"{robot_name}/controller_manager"],
    )
    diff_drive_ctrl = Node(
        package="controller_manager", executable="spawner",
        arguments=['diff_drive_base_controller', '--param-file', robot_controllers, "-c", f"{robot_name}/controller_manager"],
    )

    # Robotiq controllers (as in your file)

    # MoveIt config
    srdf_name = 'robot_1_mobile_ur10_v2.srdf'
    srdf_file = os.path.join(get_package_share_directory('multi_robot_sim'), 'config', 'moveit', srdf_name)
    doc_srdf = xacro.parse(open(srdf_file))
    xacro.process_doc(doc_srdf, mappings={"tf_prefix": tf_prefix})
    robot_description_semantic = {"robot_description_semantic": doc_srdf.toxml()}

    kinematics_yaml = load_yaml("multi_robot_sim", f"config/{robot_name}_kinematics.yaml")

    ompl_yaml = load_yaml("multi_robot_sim", f"config/{robot_name}_ompl_planning_robotiq.yaml")
    sensors_3d_yaml = load_yaml("multi_robot_sim", f"config/{robot_name}_sensors_3d.yaml")

     # Joint limits file #
    joint_limits_yaml = load_yaml("multi_robot_sim", "config/" + robot_name + "_joint_limits.yaml")
    joint_limits = {'robot_description_planning': joint_limits_yaml}

    # Pilz #
    pilz_planning = {
        "move_group": {
            "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
            "request_adapters": """ """,
            "start_state_max_bounds_error": 0.1,
            "default_planner_config": "PTP",
        }
    }
    pilz_cartesian_limits_yaml = load_yaml("multi_robot_sim", "config/" + robot_name + "_pilz_cartesian_limits.yaml")
    pilz_cartesian_limits = {'robot_description_planning': pilz_cartesian_limits_yaml}

    pilz_planning = {
        "move_group": {
            "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
            "request_adapters": """ """,
            "start_state_max_bounds_error": 0.1,
            "default_planner_config": "PTP",
        }
    }

    # Controllers #
    moveit_simple_controllers_yaml = load_yaml(
        "multi_robot_sim", "config/" + robot_name + "_moveit_controllers.yaml"
    )

    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }
    move_group_capabilities = {
        "capabilities": """pilz_industrial_motion_planner/MoveGroupSequenceAction \
            pilz_industrial_motion_planner/MoveGroupSequenceService"""
    }


    ################
    # START MoveIt2! #
    ################
    run_move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        name="move_group",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            pilz_planning,
            # ompl_planning_pipeline_config,
            joint_limits,
            pilz_cartesian_limits,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            move_group_capabilities,
            {'publish_robot_description_semantic': True},
            {"use_sim_time": True},
            {"move_group_namespace": robot_name},
            #sensors_3d_config
        ],
        remappings=[("/robot_description", robot_description_topic),
                    ("/joint_states", joint_state_topic),
                    
            ]
    )
    
    ################
    # End MoveIt2! #
    ################

    ########
    # RVIZ #
    ########
    rviz_base = os.path.join(get_package_share_directory("multi_robot_sim"), "config", "rviz")
    rviz_full_config = os.path.join(rviz_base, tf_prefix + "mobile_ur10.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            pilz_planning,
            # ompl_planning_pipeline_config,
            joint_limits,
            pilz_cartesian_limits,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            move_group_capabilities,
            {'publish_robot_description_semantic': True},
            {"use_sim_time": True},
            {"move_group_namespace": robot_name},
        ],
        remappings=[("/robot_description", robot_description_topic),
                    ("/joint_states", joint_state_topic),
                    
            ]
    )

    ##########################
    # Action Managment Nodes #
    ##########################
    MoveInterface = Node(
        name="move",
        package="ros2srrc_execution",
        executable="move",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"use_sim_time": True}, \
                    {"ROB_PARAM": tf_prefix + "ur10e"}, {"ENV_PARAM": "gazebo"},
                    ],
        remappings=[("/robot_description", robot_description_topic),
                    ("/joint_states", joint_state_topic)
            ]
    )
    SequenceInterface = Node(
        name="sequence",
        package="ros2srrc_execution",
        executable="sequence",
        output="screen",
        namespace=robot_name,
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"use_sim_time": True}, \
                    {"ROB_PARAM": tf_prefix + "ur10e"}, {"EE_PARAM": tf_prefix + "robotiq_2f85"}, {"ENV_PARAM": "gazebo"}],
        remappings=[
                    ("/robot_description", robot_description_topic),
                    ("/joint_states", joint_state_topic)
                ]
    )

    MobileMXYWServer = Node(
        package='multi_robot_py',
        executable='mobile_motion_server.py',
        name='mobile_mxyw_server',
        namespace='robot_1',   # keep your node under robot_1 if you want
        output='screen',
        parameters=[{
            'odom_frame': 'robot_1/odom',
            'base_frame': 'robot_1/base_footprint',
            'cmd_vel_topic': '/robot_1/diff_drive_base_controller/cmd_vel',
            'xy_tolerance': 0.05,
            'yaw_tolerance': 0.05,
        }],
    )


    # Launch flow
    return [
        # slight stagger to let Gazebo settle before mobile spawn
        TimerAction(period=5.0, actions=[spawn_entity]),
        state_pub,
        RegisterEventHandler(OnProcessExit(target_action=spawn_entity, on_exit=[js_broadcaster, diff_drive_ctrl])),
        RegisterEventHandler(OnProcessExit(target_action=js_broadcaster, on_exit=[joint_traj_ctrl])),
        RegisterEventHandler(OnProcessExit(target_action=joint_traj_ctrl,on_exit=[
                    TimerAction(period=10.0, actions=[run_move_group, rviz_node]),
                    TimerAction(period=20.0, actions=[MoveInterface]),
                    # TimerAction(period=10.0, actions=[MobileMXYWServer ]),
                ],
            )
        ),
    ]

def generate_launch_description():
    multi_robot_sim = get_package_share_directory('multi_robot_sim')

    # Gazebo resources
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[str(Path('multi_robot_sim').parent.resolve())]
    )

    ### SimensTUM case study (world file)   
    world = os.path.join(multi_robot_sim, 'worlds', 'wind_v2.sdf')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(multi_robot_sim, "launch", "gz_sim.launch.py")]),
        launch_arguments=[('gz_args', [world, ' -v 4 -r'])],
    )

    # Bridges (clock + mobile bridges file; add your ad-hoc cmd_vel/tf bridges if you want)
    gz_ros2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )


    ##################
    # Bridge ROS ign #
    ##################

    tc_bridge_params = os.path.join(multi_robot_sim, 'config', 'ros_bridge',
        'tc_ros_gz_bridge.yaml'
    )
    # Bridge ROS
    gz_ros2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )


    ur_bridge_params = os.path.join(multi_robot_sim, 'config', 'ros_bridge',
        'ur_ros_gz_bridge.yaml'
    )
    # 1) parameter_bridge: bridge Image<->gz.msgs.Image, PointCloud2<->gz.msgs.PointCloudPacked
    node_zed_camera_gz_bridge = Node(
        name='zed_camera_gz_bridge',
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'config_file': ur_bridge_params,
        }],
    )

    # 2) Color image bridge
    node_zed_camera_gz_image_bridge = Node(
        name='zed_camera_gz_image_bridge',
        package='ros_gz_image',
        executable='image_bridge',
        output='screen',
        arguments=[
            '/sensors/robot_1/zed_mini_camera/image',
        ],
        remappings=[
            ('/sensors/robot_1/zed_mini_camera/image',
            '/sensors/robot_1/zed_mini_camera/color/image'),
            ('/sensors/robot_1/zed_mini_camera/image/compressed',
            '/sensors/robot_1/zed_mini_camera/color/compressed'),
            ('/sensors/robot_1/zed_mini_camera/image/compressedDepth',
            '/sensors/robot_1/zed_mini_camera/color/compressedDepth'),
            ('/sensors/robot_1/zed_mini_camera/image/theora',
            '/sensors/robot_1/zed_mini_camera/color/theora'),
        ],
        parameters=[{'use_sim_time': True}],
    )

    # 3) Depth image bridge
    node_zed_camera_gz_depth_bridge = Node(
        name='zed_camera_gz_depth_bridge',
        package='ros_gz_image',
        executable='image_bridge',
        output='screen',
        arguments=[
            '/sensors/robot_1/zed_mini_camera/depth_image',
        ],
        remappings=[
            ('/sensors/robot_1/zed_mini_camera/depth_image',
            '/sensors/robot_1/zed_mini_camera/depth/image'),
            ('/sensors/robot_1/zed_mini_camera/depth_image/compressed',
            '/sensors/robot_1/zed_mini_camera/depth/compressed'),
            ('/sensors/robot_1/zed_mini_camera/depth_image/compressedDepth',
            '/sensors/robot_1/zed_mini_camera/depth/compressedDepth'),
            ('/sensors/robot_1/zed_mini_camera/depth_image/theora',
            '/sensors/robot_1/zed_mini_camera/depth/theora'),
        ],
        parameters=[{'use_sim_time': True}],
    )
    

    print("Formwork installation - Husky-ur10e with towercrane")
    print("")


    mobile_bridge_params = os.path.join(multi_robot_sim, 'config', 'ros_bridge', 'mobile_ros_gz_bridge.yaml')
    diff_control_gz_bridge = Node(
        name='diff_control_gz_bridge', package='ros_gz_bridge', executable='parameter_bridge',
        output='screen', parameters=[{'use_sim_time': True, 'config_file': mobile_bridge_params}],
    )

    # Robot xacros
    xacro_mobile = os.path.join(multi_robot_sim, 'urdf', 'robot_1_mobile_ur10_gz_v3.urdf.xacro')

    # Build action lists
    nodes = [
        gazebo_resource_path,
        gazebo,
        gz_ros2_bridge,
 
    ]

    nodes.extend(spawn_mobile(xacro_mobile, robot_name="robot_1"))

    return LaunchDescription(nodes)
