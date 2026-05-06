from setuptools import setup

package_name = 'semantic_path_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CougnetDefaux Gabin',
    maintainer_email='gabin.cougnet-defaux@telecomnancy.eu',
    description='Replay a semantic path JSON as velocity commands for Unitree Go2 teleoperation in Gazebo.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'semantic_path_teleop = semantic_path_teleop.semantic_path_teleop:main',
            'spawn_path_markers = semantic_path_teleop.spawn_path_markers:main',
        ],
    },
)
