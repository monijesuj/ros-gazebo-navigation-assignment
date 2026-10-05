import os
from datetime import datetime
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share=Path(get_package_share_directory('robot_demo'))
    gui, rviz, autostart = [LaunchConfiguration(n) for n in ('gui','rviz','autostart')]
    mission=LaunchConfiguration('mission_file'); report=LaunchConfiguration('report_file')
    args=[DeclareLaunchArgument('record_bag',default_value='false'),DeclareLaunchArgument('gui',default_value='true'),DeclareLaunchArgument('rviz',default_value='true'),DeclareLaunchArgument('autostart',default_value='true'),DeclareLaunchArgument('mission_file',default_value=str(share/'config/mission.json')),DeclareLaunchArgument('report_file',default_value=str(Path.cwd()/'mission_report.json'))]
    common={'use_sim_time':True}
    simulator=ExecuteProcess(cmd=['ign','gazebo','-r',PythonExpression(["'' if '",gui,"' == 'true' else '-s'"]),str(share/'worlds/navigation.sdf')],output='screen')
    # Distinct sensor bridges give every ROS stream its correct URDF frame.
    def bridge(name, topics, frame=None):
        params={**common}
        if frame:params['override_frame_id']=frame
        return Node(package='ros_gz_bridge',executable='parameter_bridge',name=name,arguments=topics,parameters=[params],output='screen')
    bridges=[bridge('motion_bridge',['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock','/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist','/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry','/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V','/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model','/world/navigation/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V','/contacts/base_link@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts','/contacts/left_wheel@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts','/contacts/right_wheel@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts']),bridge('lidar_bridge',['/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],'lidar_link'),bridge('camera_bridge',['/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image','/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image','/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo','/camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'],'camera_optical_link'),bridge('gps_bridge',['/gps/fix@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat'],'gps_link')]
    return LaunchDescription(args+[SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH',str(share/'models')+os.pathsep+os.environ.get('IGN_GAZEBO_RESOURCE_PATH','')),simulator]+bridges+[
      Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{**common,'robot_description':(share/'urdf/rover.urdf').read_text()}]),
      Node(package='robot_demo',executable='navigator',parameters=[{**common,'mission_file':mission,'autostart':ParameterValue(autostart,value_type=bool)}],output='screen'),
      Node(package='robot_demo',executable='monitor',parameters=[{**common,'mission_file':mission,'report_file':report}],output='screen'),
      ExecuteProcess(cmd=['ros2','bag','record','-o',str(Path.cwd()/'evidence'/('bag_launch_'+datetime.now().strftime('%Y%m%d_%H%M%S'))),'/clock','/tf','/tf_static','/odom','/scan','/gps/fix','/camera/depth_image','/camera/camera_info','/cmd_vel','/map','/planned_path','/travelled_path','/mission/status','/mission/verification'],condition=IfCondition(LaunchConfiguration('record_bag')),output='screen'),
      Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'config/demo.rviz')],parameters=[common],condition=IfCondition(rviz))])
