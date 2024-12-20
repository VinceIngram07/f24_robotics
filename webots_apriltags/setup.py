from setuptools import setup, find_packages

package_name = 'webots_apriltags'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/webots_apriltags.launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/turtlebot3_apriltags.wbt']))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/resource', [
    'resource/turtlebot_webots.urdf',
    'resource/ros2control.yml',
]))
data_files.append(('share/' + package_name + '/rviz', ['rviz/turtlebot3_apriltags.rviz']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),  # Automatically find packages in this directory
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    maintainer='Monica Anderson-UA',
    maintainer_email='anderson@ua.edu',
    description='Webots Apriltags',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tag_logger = webots_apriltags.scripts.tag_logger:main',
            'random_movement = webots_apriltags.scripts.random:main',
        ],
        'launch.frontend.launch_extension': ['launch_ros = launch_ros'],
    },
)
