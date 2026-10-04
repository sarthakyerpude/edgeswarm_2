from setuptools import setup

package_name = 'webots_warehouse_sim'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/webots_world.launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/warehouse.wbt']),
        ('share/' + package_name + '/protos', ['protos/EdgeSwarmAMR.proto']),
        ('share/' + package_name + '/resource', [
            'resource/amr_webots.urdf', 'resource/item_manager.urdf',
            'resource/cyclonedds.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Webots backend for the EdgeSwarm warehouse fleet.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'scan_flip = webots_warehouse_sim.scan_flip:main',
    ]},
)
