from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    open_camera = Node(
        package='brobot_controller',
        executable='brobot_camera.py'
    )

    detect_face = Node(
        package='brobot_controller',
        executable='brobot_face_detection.py'
    )

    detect_hand = Node(
        package='brobot_controller',
        executable='brobot_hand_gesture_recognition.py'
    )


    launch_description = LaunchDescription()
    launch_description.add_action(open_camera)
    launch_description.add_action(detect_face)
    launch_description.add_action(detect_hand)
    return launch_description
