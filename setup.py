import os
import glob
from setuptools import find_packages, setup

package_name = 'geopose_ros_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), ['launch/launch.py']),
        (os.path.join('share', package_name, "resource"), glob.glob('resource/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jclarke',
    maintainer_email='james.clarke@os.uk',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publish_geopose = geopose_ros_tools.publish_geopose:main',
            'geopose_to_osgb36 = geopose_ros_tools.geopose_to_osgb36:main',
            'publish_ply_pointcloud = geopose_ros_tools.publish_ply_pointcloud:main',
        ],
    },
)