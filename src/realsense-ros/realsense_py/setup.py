from setuptools import setup
import os
from glob import glob

package_name = 'realsense_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rvc',
    maintainer_email='751915341@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'apriltag_detection  = realsense_py.apriltag_detection:main',
        'view_images         = realsense_py.view_images:main',
        'cam2cam_cal         = realsense_py.cam2cam_cal:main',
        'test_yaml_params    = realsense_py.test_yaml_params:main',
        'my_depth_subscriber = realsense_py.my_depth_subscriber:main',
        'test                = realsense_py.apriltag_detection_new:main',
        'concatenate_images  = realsense_py.concatenate_images:main',
        'arucotag_detection  = realsense_py.arucotag_detection:main'
        ],
    },
)
