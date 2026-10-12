from glob import glob

from setuptools import find_packages, setup

package_name = 'assembly_line'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jason Lobell',
    maintainer_email='jasonlobell123@gmail.com',
    description='Assembly line example: five arms, three products, QA, on jev_ai',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'line_supervisor = assembly_line.supervisor:main',
        'arm_agent = assembly_line.arm_agent:main',
        'qa_agent = assembly_line.qa_agent:main',
        'line_scorer = assembly_line.scorer:main',
        'line_world = assembly_line.gz_world:main',
    ]},
)
