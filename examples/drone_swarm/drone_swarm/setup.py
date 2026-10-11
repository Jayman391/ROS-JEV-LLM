from glob import glob

from setuptools import find_packages, setup

package_name = 'drone_swarm'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jason Lobell',
    maintainer_email='jasonlobell123@gmail.com',
    description='Drone swarm reference example (Part B): search & rescue on jev_ai + Gazebo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'environment = drone_swarm.environment:main',
        'drone_agent = drone_swarm.drone_agent:main',
        'mission_coordinator = drone_swarm.coordinator:main',
        'mission_scorer = drone_swarm.scorer:main',
        'swarm_world = drone_swarm.scenario:main',
    ]},
)
