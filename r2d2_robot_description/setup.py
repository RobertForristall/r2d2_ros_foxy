from struct import pack
from setuptools import setup

config_files = [
    'urdf/r2d2_body.urdf.xacro', 
    'urdf/r2d2_leg.urdf.xacro', 
    'urdf/r2d2.urdf.xacro', 
    'urdf/general_macros.xacro', 
    'urdf/materials.xacro',
    'urdf/r2d2_control.urdf.xacro',
    'urdf/r2d2_rviz.urdf.xacro',
    'urdf/r2d2.gazebo.xacro'
]

package_name = 'r2d2_robot_description'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/urdf', config_files))
data_files.append(('share/' + package_name + '/launch', ['launch/rviz.py', 'launch/gazebo.py']))
data_files.append(('share/' + package_name + '/config', ['config/r2d2.rviz', 'config/ros2_controllers.yaml', 'config/rviz_controllers.yaml']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robert',
    maintainer_email='robert.forristall@knights.ucf.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'head_driver = r2d2_robot_description.head_driver:main',
            'movement_driver = r2d2_robot_description.movement_driver:main',
            'spawn_entity = r2d2_robot_description.spawn_entity:main'
        ],
    },
)
