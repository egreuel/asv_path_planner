import os
from glob import glob
from setuptools import setup

package_name = 'asv_path_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[pxy][yma]*')))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ergr',
    maintainer_email='ey.greuel@web.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "vel_obst_node = asv_path_planner.master_vel_obst_node:main",
            "draw_circle = asv_path_planner.draw_circle:main",
            "pose_subs = asv_path_planner.pose_subs:main",
            "ros_script = asv_path_planner.ros_script:main",
            "velobst = asv_path_planner.velobst:main"
        ],
    },
)
