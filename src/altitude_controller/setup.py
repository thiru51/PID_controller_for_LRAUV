from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'altitude_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # ('share/' + package_name, ['altitude_controller/waypoints.yml']),
 
 
        (os.path.join('share', package_name, 'launch'), ['launch/hybrid_altitude_controller.launch.py']),

        
        (os.path.join('share', package_name, 'worlds'), ['worlds/buoyant_lrauv.sdf']),
        (os.path.join('share', package_name, 'worlds'), ['worlds/sydney_regatta_lrauv.sdf']),
        (os.path.join('share', package_name, 'worlds'), ['worlds/buoyant_lrauv_simple.sdf']),
        (os.path.join('share', package_name, 'models'), ['models/my_lrauv/model.sdf']),

        (os.path.join('share', package_name, 'bridge'), ['bridge/bridge_config.yaml']),

        # Install config and results folder so plotter can save/locate files
        (os.path.join('share', package_name, 'config'), ['altitude_controller/waypoints.yaml']),
        (os.path.join('share', package_name, 'results_and_plots'), glob('results_and_plots/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thiru',
    maintainer_email='na22b078@smail.iitm.ac.in',
    description='altitude_controller for LRAUV',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lrauv_3d_los_controller = altitude_controller.lrauv_3d_los_controller:main',
            'hybrid_altitude_controller = altitude_controller.hybrid_altitude_controller:main',
            'trajectory_plotter = altitude_controller.trajectory_plotter:main',
            'trajectory_paraview = altitude_controller.trajectory_paraview:main',
            'sonar_image_node = altitude_controller.sonar_image_node:main',
            'open_loop_controller = altitude_controller.open_loop_controler:main',
        ],
    },
)
