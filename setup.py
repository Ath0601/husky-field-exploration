from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'fieldrobo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.*')),
        (os.path.join('share', package_name, 'meshes', 'accessories'), glob('meshes/accessories/*.*')),
        (os.path.join('share', package_name, 'config', 'rviz'), glob('config/rviz/*.rviz')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'world'), glob('world/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='atharva',
    maintainer_email='atharva.abhi.kulkarni@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pose_control = fieldrobo.pose_controller:main',
            'traj_generate = fieldrobo.trajectory_generator:main',
            'frontier_explorer = fieldrobo.frontier_explorer:main',
        ],
    },
)
