from setuptools import setup

package_name = 'my_ros2_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jiamingzhang',
    maintainer_email='wqfx65zjm@gmail.com',
    description='test out calibration subscription to lidar pointcloud and image',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibration_subscriber = my_ros2_package.calib_test_node:main',
        ],
    },
)
