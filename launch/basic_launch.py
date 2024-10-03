import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='wiros_csi',
            executable='csi_node',
            name='wiros_csi_node',
            parameters = [{
                'asus_host': 'wcsng',
                'asus_pwd': 'robot123!',
                'asus_ip': '192.168.44.1',
                'channel': 157,
                'bw': 20,
                'mac_filter': "AC:37:*:*:*:*",
                'beacon_rate': 0,
                
                
                
            }]),
  ])
