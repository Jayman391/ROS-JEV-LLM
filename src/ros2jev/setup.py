from setuptools import find_packages, setup

package_name = 'ros2jev'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['ros2cli'],
    zip_safe=True,
    maintainer='Jason Lobell',
    maintainer_email='jasonlobell123@gmail.com',
    description='ros2 jev: status, ask, report, calibrate, replay, ope, search',
    license='Apache-2.0',
    entry_points={
        'ros2cli.command': ['jev = ros2jev.command.jev:JevCommand'],
        'ros2jev.verb': [
            'status = ros2jev.verb.status:StatusVerb',
            'ask = ros2jev.verb.ask:AskVerb',
            'report = ros2jev.verb.report:ReportVerb',
            'calibrate = ros2jev.verb.calibrate:CalibrateVerb',
            'replay = ros2jev.verb.replay:ReplayVerb',
            'ope = ros2jev.verb.ope:OpeVerb',
            'search = ros2jev.verb.search:SearchVerb',
        ],
    },
)
