import os
from setuptools import setup

package_name = 'my_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    # Files we want to install, specifically launch files
    # to be modified with os.path.join
    data_files=[
        # marker file
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        # xml file
        ('share/' + package_name, ['package.xml']),
        # launch file
        ('share/' + package_name + '/launch/', ['launch/' + 'my_controller.launch.py']),
        # yaml file
        ('share/' + package_name + '/config/', ['config/' + 'publisher_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Opal Leung',
    maintainer_email='opalleunglch@mailbox.org',
    description='UR demo skill: move to the goals listed in a yaml file',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_controller = my_controller.publisher_joint_trajectory_controller:main',
            'joint_subscriber = my_controller.subscriber_joint_state:main'
        ],
    },
)