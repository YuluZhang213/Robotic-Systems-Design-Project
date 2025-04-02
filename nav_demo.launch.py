from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import SetParameter, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
import os



def generate_launch_description():
    ld = LaunchDescription()

    # Parameters, Nodes and Launch files go here

    # Declare package directory
    pkg_nav = get_package_share_directory('t1_nav')

    # Define nav_to_pose behaviour tree
    bt_xml_navtopose_file = PathJoinSubstitution([pkg_nav, 'behavior_tree_xml', 'bt_simple_nav.xml'])

    # Declare the argument for use_sim_time
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Whether to run in simulation mode'
    )

    # Necessary fixes
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behaviour_server',
        'bt_navigator',
        #'map_server',
        #'amcl',
        #'waypoint_follower',
    ]

    # LOAD PARAMETERS FROM YAML FILES
    config_bt_nav = PathJoinSubstitution([pkg_nav, 'config', 'bt_nav.yaml'])
    config_planner = PathJoinSubstitution([pkg_nav, 'config', 'planner.yaml'])
    config_controller = PathJoinSubstitution([pkg_nav, 'config', 'controller.yaml'])
    config_params = PathJoinSubstitution([pkg_nav, 'config', 'params.yaml'])

    # Declare the use_sim_time LaunchConfiguration
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Include t1_rover to spawn the robot and launch rviz, pass through use sim time argument
    launch_rover = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory('t1_rover'), '/launch', '/rover.launch.py']),
    launch_arguments={
        'use_sim_time': use_sim_time
    }.items(),
    )

    # Include SLAM Toolbox standard launch file
    launch_slamtoolbox = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory('slam_toolbox'), '/launch', '/online_async_launch.py']),
    launch_arguments={
        'use_sim_time': use_sim_time
    }.items(),
    )

    # Behaviour Tree Navigator
    node_bt_nav = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            config_bt_nav,
            #{'default_nav_through_poses_bt_xml' : bt_xml_navtopose_file},
            {'use_sim_time': use_sim_time},
        ], 

        remappings=remappings,
    )

    # Behaviour Tree Server
    node_behaviour = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behaviour_server',
        output='screen',
        parameters=[
            config_bt_nav,
            #{'default_nav_through_poses_bt_xml' : bt_xml_navtopose_file},
            {'use_sim_time': use_sim_time},
        ], 
        remappings=remappings,
    )

    # Planner Server Node
    node_planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
         parameters=[
            config_planner,
            {'use_sim_time': use_sim_time},
        ], 
        remappings=remappings,
    )

    # Controller Server Node
    node_controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
         parameters=[
            config_controller,
            {'use_sim_time': use_sim_time},
        ], 
        remappings=remappings,
    )

    # Map Server Node
    node_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[config_params]  # Make sure 'config_params' includes map_server parameters
    )

    # AMCL Node
    node_amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[config_params]  # Ensure 'config_params' includes AMCL parameters
    )

    # Waypoint Follower Node
    node_waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[config_params]  # Ensure 'config_params' includes waypoint_follower parameters
    )

    # Lifecycle Node Manager to automatically start lifecycles nodes (from list)
    node_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'autostart': True}, {'node_names': lifecycle_nodes}],
    )

     # Lifecycle Node Manager to automatically start lifecycles nodes (from list)
    node_explore = Node(
        package='t1_nav',
        executable='explore',
        name='explore',
        output='screen',
    )

    node_simple_nav = Node(
        package='t1_nav',
        executable='simple_nav',
        name='simple_nav',
        output='screen',
    )

    node_ui_nav = Node(
        package='t1_ui',
        executable='main',
        name='main',
        output='screen',
    )


    # Add actions to LaunchDescription
    ld.add_action(use_sim_time_arg)
    ld.add_action(launch_rover)
    ld.add_action(launch_slamtoolbox)
    ld.add_action(node_bt_nav)
    ld.add_action(node_behaviour)
    ld.add_action(node_planner)
    ld.add_action(node_controller)
    ld.add_action(node_lifecycle_manager)
    ld.add_action(node_ui_nav)
    #ld.add_action(node_map_server)
    #ld.add_action(node_amcl)
    #ld.add_action(node_waypoint_follower)
    #ld.add_action(node_map_saver)
    #ld.add_action(node_explore)
    #ld.add_action(node_simple_nav)

    return ld