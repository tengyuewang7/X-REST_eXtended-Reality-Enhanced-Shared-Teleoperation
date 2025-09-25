import os 
from glob import glob
from setuptools import setup

package_name = 'to_unity_py'
share_dir = os.path.join("share", package_name)

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rvc',
    maintainer_email='tengyue.21@intl.zju.edu.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rizon_frame_visual = to_unity_py.rizon_frame_visual:main',
            'rizon_visual_test = to_unity_py.rizon_visual_test:main',
            'service = to_unity_py.test_service:main',
            'decision_maker = to_unity_py.decision_maker:main',
            'stupid_test = to_unity_py.stupid_test:main',
            'ur3e_control = to_unity_py.ur3e_control:main',
            'rizon_control = to_unity_py.rizon_control:main',
            'rizon_control_2 = to_unity_py.rizon_control_2:main',
            'test = to_unity_py.test:main',
        ],
    },
)
