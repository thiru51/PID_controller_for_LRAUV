from setuptools import setup, find_packages
import os

package_name = 'pid_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # ('share/' + package_name, ['pid_controller/waypoints.yml']),

        (os.path.join('share', package_name, 'launch'), ['launch/pid_controller.launch.py']),

        (os.path.join('share', package_name, 'worlds'), ['worlds/buoyant_lrauv.sdf']),

        (os.path.join('share', package_name, 'models'), ['models/my_lrauv/model.sdf']),

        (os.path.join('share', package_name, 'bridge'), ['bridge/bridge_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thiru',
    maintainer_email='na22b078@smail.iitm.ac.in',
    description='pid_controller for LRAUV',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            
        ],
    },
)
