from setuptools import setup

package_name = 'pt_selection_pkg'

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
    description='select 3d point interactively using rviz',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pt_selection_node = pt_selection_pkg.rviz_pt_selection:main',
            'pt_collection_node = pt_selection_pkg.rviz_pt_collection:main',
            'undistort_img_node = pt_selection_pkg.export_undistort_img:main'
        ],
    },
)
