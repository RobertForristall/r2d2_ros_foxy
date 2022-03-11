from struct import pack
from setuptools import setup

package_name = 'r2d2_robot_description'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/urdf', ['urdf/r2d2_body.urdf.xacro', 'urdf/r2d2_leg.urdf.xacro', 'urdf/r2d2.urdf.xacro']))
data_files.append(('share/' + package_name + '/launch', ['launch/rviz.py']))
data_files.append(('share/' + package_name + '/config', ['config/r2d2.rviz']))
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
        ],
    },
)
