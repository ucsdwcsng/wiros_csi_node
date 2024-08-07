import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='wiros_csi',
            executable='csi_node',
            name='wiros_csi_node',
            parameters = [{
            }]),
  ])
