import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    launch_description = LaunchDescription()

    # get pkg path
    laser_line_pkg = get_package_share_directory("laserline")
    
    # create launch configuration variables
    params_file_path = LaunchConfiguration('laser_line_params_files', default=os.path.join(laser_line_pkg, 'params', 'config.yaml'))
    laser_line_log_level = LaunchConfiguration('laser_line_log_level', default="info")
    line_path_compare_log_level = LaunchConfiguration('line_path_compare_log_level', default="info")
    
    # laserline segement node
    laser_line_node = Node(
        executable='laserline',
        package='laserline',
        # name='laser',
        namespace='',
        output='screen',
        parameters=[params_file_path],
        arguments=['--ros-args', '--log-level', ['laser_line_detection:=', laser_line_log_level]],        
    )

    # line_path_compare node
    line_path_compare_node = Node(
        executable='line_path_compare',
        package='laserline',
        name='line_path_compare_node',
        namespace='',
        output='screen',
        parameters=[params_file_path],
        arguments=['--ros-args', '--log-level', ['line_path_compare_log_level:=', line_path_compare_log_level]],        
    )

   
    launch_description.add_action(laser_line_node)
    # launch_description.add_action(line_path_compare_node)

    return launch_description
