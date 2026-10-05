from setuptools import setup

package_name = 'amr_navigation_runtime'
setup(
    name=package_name,
    version='0.1.0',
    packages=['amr_navigation_runtime'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/navigation.launch.py',
            'launch/simulation_imu.launch.py', 'launch/three_amr.launch.py',
            'launch/sim_gazebo.launch.py', 'launch/sim_external.launch.py',
            'launch/sim_webots.launch.py']),
        ('share/' + package_name + '/config', [
            'config/nav2_params.yaml', 'config/velocity_gate.yaml',
            'config/warehouse_map.yaml', 'config/warehouse_map.pgm']),
        ('share/' + package_name + '/rviz', [
            'rviz/amr.rviz', 'rviz/fleet_view.rviz']),
        ('share/' + package_name + '/models/amr_robot_imu', [
            'models/amr_robot_imu/model.sdf', 'models/amr_robot_imu/model.config']),
        ('share/' + package_name + '/models', ['models/dynamic_pallet.sdf']),
    ],
    entry_points={'console_scripts': [
        'velocity_gate = amr_navigation_runtime.velocity_gate:main',
        'fleet_goal_bridge = amr_navigation_runtime.fleet_goal_bridge:main',
        'initial_pose_publisher = amr_navigation_runtime.initial_pose_publisher:main',
        'collision_recorder = amr_navigation_runtime.collision_recorder:main',
        'experiment_recorder = amr_navigation_runtime.experiment_recorder:main',
        'gazebo_tf_relay = amr_navigation_runtime.tf_relay:main',
    ]},
)
