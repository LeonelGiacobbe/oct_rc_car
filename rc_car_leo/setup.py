from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rc_car_leo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*.[pxy][yma]*')))
    ],
    install_requires=['setuptools', 'numpy', 'sensor_msgs', 'rospy'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='lg1221@msstate.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ackermann_conversion = rc_car_leo.ackermann_conversion:main',
            'openrtk_node = ros_openrtk.openrtk_node:main',
            'pointcloud_filter_node = rc_car_leo.pointcloud_filter_node:main',  # Adjusted name
            'filter_2 = rc_car_leo.filter_2:main',
        ],
    },
)
