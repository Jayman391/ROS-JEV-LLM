from glob import glob

from setuptools import find_packages, setup

package_name = 'jev_ai_examples'

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
    description='jev_ai examples: toy_patrol',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['toy_patrol = jev_ai_examples.toy_patrol:main',
                                      'toy_supervisor = jev_ai_examples.toy_supervisor:main',
                                      'turtle_agent = jev_ai_examples.turtle_agent:main',
                                      'turtle_arena = jev_ai_examples.turtle_agent:arena_main']},
)
