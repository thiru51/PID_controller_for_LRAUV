from setuptools import setup, find_packages
import os
from glob import glob


package_name = 'lrauv_zigzag_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name]), # <--- MUST match inner folder
    data_files=[
    
    
    ('share/ament_index/resource_index/packages',
     ['resource/lrauv_zigzag_controller']),
    ('share/lrauv_zigzag_controller', ['package.xml']),
            ('share/lrauv_zigzag_controller', ['lrauv_zigzag_controller/waypoints.yml']),


         
        (os.path.join('share', package_name, 'launch'), ['launch/zigzag_test.launch.py']),
        (os.path.join('share', package_name, 'urdf'), ['urdf/my_lrauv.urdf']),
        (os.path.join('share', package_name, 'worlds'), ['worlds/buoyant_lrauv.sdf']),
        (os.path.join('share', package_name, 'models'), ['models/my_lrauv/model.sdf']),
        (os.path.join('share', package_name, 'bridge'), ['bridge/bridge_config.yaml']),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thiru',
    maintainer_email='na22b078@smail.iitm.ac.in',
    description='Zigzag maneuver controller for LRAUV',
    license='MIT',
    tests_require=['pytest'],
   entry_points={
    'console_scripts': [
        
        'mav_pid_controller = lrauv_zigzag_controller.mav_pid_controller:main',
        'waypoints_publisher = lrauv_zigzag_controller.waypoints_publisher:main',

    	'imu_odom_logger = lrauv_zigzag_controller.imu_odom_logger:main',
    	'imu_odom_dvl_logger = lrauv_zigzag_controller.imu_odom_dvl_logger:main',
        'zigzag_node = lrauv_zigzag_controller.zigzag_node:main'
    ],
},
)





