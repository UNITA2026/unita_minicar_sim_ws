import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    sim_pkg_name = 'minicar_simulation'
    sim_pkg_share = get_package_share_directory(sim_pkg_name)
    
    # [주의] YOLO 모델 파일 (.pt) 경로
    workspace_model_path = os.path.join(os.getcwd(), 'lane.pt')

    print(f"\n[Launch] Loading YOLO model from: {workspace_model_path}\n")

    spawn_car_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg_share, 'launch', 'spawn_car.launch.py')
        ),
        launch_arguments={
            'x': '1.126491',  
            'y': '5.710308',
            'z': '0.1',     
            'yaw': '2.929481'
        }.items()
    )

    # [camera_perception_pkg] 1. YOLOv8 객체 인식
    yolo_node = Node(
        package='camera_perception_pkg',
        executable='yolov8_node',
        name='yolov8_node',
        output='screen',
        parameters=[{
            'model': workspace_model_path,
            'device': 'cpu',
            'threshold': 0.5,
            'enable': True,
        }],
        remappings=[
            ('camera/image_raw', '/front_camera_sensor/image_raw')
        ]
    )

    # [camera_perception_pkg] 2. 차선 정보 추출
    lane_info_node = Node(
        package='camera_perception_pkg',
        executable='lane_info_extractor_node',
        name='lane_info_extractor_node',
        output='screen'
    )

    # [decision_making_pkg] 3. 경로 계획 (Path Planner)
    path_planner_node = Node(
        package='decision_making_pkg',
        executable='path_planner_node',
        name='path_planner_node',
        output='screen'
    )

    # [decision_making_pkg] 4. 모션 계획 (Motion Planner)
    motion_planner_node = Node(
        package='decision_making_pkg',
        executable='motion_planner_node',
        name='motion_planner_node',
        output='screen'
    )

    # [minicar_simulation] 5. 시뮬레이션 송신 (제어기)
    sim_sender_node = Node(
        package='minicar_simulation', 
        executable='simulation_sender.py', 
        name='simulation_sender_node',
        output='screen',
        parameters=[{
            'sub_topic': '/topic_control_signal',
            'pub_topic': '/cmd_vel'
        }]
    )

    # [debug_pkg] 6. 시각화 노드 (선택)
    yolo_viz_node = Node(
        package='debug_pkg',
        executable='yolov8_visualizer_node',
        name='yolov8_visualizer_node',
        output='screen',
        remappings=[
            ('camera/image_raw', '/front_camera_sensor/image_raw'),
            ('detections', '/detections') 
        ]
    )
    
    path_viz_node = Node(
        package='debug_pkg',
        executable='path_visualizer_node',
        name='path_visualizer_node',
        output='screen'
    )

    return LaunchDescription([
        spawn_car_launch,
        
        TimerAction(
            period=2.0,
            actions=[
                yolo_node,
                lane_info_node,
                path_planner_node,
                motion_planner_node,
                sim_sender_node,
                yolo_viz_node,
                path_viz_node
            ]
        )
    ])