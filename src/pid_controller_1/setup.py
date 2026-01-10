from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pid_controller_1'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
         



        ('share/ament_index/resource_index/packages', ['resource/pid_controller_1']),
        ('share/pid_controller_1', ['package.xml']),
        ('share/pid_controller_1', ['pid_controller_1/waypoints.yaml']),
        ('share/pid_controller_1/worlds', ['worlds/buoyant_lrauv.sdf']),
        ('share/pid_controller_1/launch', ['launch/pid_controller_1.launch.py']),


        (os.path.join('share', package_name, 'pid_controller_1'), ['pid_controller_1/waypoints.yaml']),
        (os.path.join('share', package_name, 'launch'), ['launch/pid_controller_1.launch.py']),
        (os.path.join('share', package_name, 'urdf'), ['urdf/my_lrauv.urdf']),
        (os.path.join('share', package_name, 'worlds'), ['worlds/buoyant_lrauv.sdf']),
        (os.path.join('share', package_name, 'models'), ['models/my_lrauv/model.sdf']),
        (os.path.join('share', package_name, 'bridge'), ['bridge/bridge_config.yaml']),



    
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thiru',
    maintainer_email='na22b078@smail.iitm.ac.in',
    description='waypoint tracking using pid controller for LRAUV',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add your executable scripts here, e.g.:
            'pid_controller = pid_controller_1.pid_controller:main',
            'trajectory_plotter = pid_controller_1.trajectory_plotter:main',
        ],
    },
)