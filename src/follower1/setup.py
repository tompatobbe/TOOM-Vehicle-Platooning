from setuptools import find_packages, setup

package_name = 'follower1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/follower1.launch.py']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tobbe',
    maintainer_email='thorgrentobias@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'servo_driver = follower1.servo_driver:main',
            'distance = follower1.distance:main',
            'distance3 = follower1.distance3:main',
            'motor_driver = follower1.motor_driver:main',
            'controller = follower1.controller:main',
            'motor_teleop = follower1.motor_teleop:main',
            'motor_test = follower1.motor_test:main',
            'mpc = follower1.mpc:main',
            'camera = follower1.camera:main',
            'pixy2_driver = follower1.pixy2_driver:main',
            'speed = follower1.speed:main',
        ],
    },
)
