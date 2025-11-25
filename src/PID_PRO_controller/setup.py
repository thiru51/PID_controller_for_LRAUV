from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'PID_PRO_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # ('share/' + package_name, ['PID_PRO_controller/waypoints.yml']),

        (os.path.join('share', package_name, 'launch'), ['launch/PID_PRO_controller.launch.py']),

        (os.path.join('share', package_name, 'worlds'), ['worlds/buoyant_lrauv.sdf']),

        (os.path.join('share', package_name, 'models'), ['models/my_lrauv/model.sdf']),

        (os.path.join('share', package_name, 'bridge'), ['bridge/bridge_config.yaml']),

        # Install config and results folder so plotter can save/locate files
        (os.path.join('share', package_name, 'config'), ['PID_PRO_controller/waypoints.yaml']),
        (os.path.join('share', package_name, 'results_and_plots'), glob('results_and_plots/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thiru',
    maintainer_email='na22b078@smail.iitm.ac.in',
    description='PID_PRO_controller for LRAUV',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lrauv_3d_los_controller = PID_PRO_controller.lrauv_3d_los_controller:main',
            'trajectory_plotter = PID_PRO_controller.trajectory_plotter:main',
            
        ],
    },
)
