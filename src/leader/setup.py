from setuptools import find_packages, setup

package_name = 'leader'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/leader.launch.py']),
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
            'axis = leader.axis:main',
            'ds4_print = leader.ds4_print:main',
            'ds4_pub = leader.ds4_pub:main',
            'motor_driver = leader.motor_driver:main',
            'motor_tester = leader.motor_tester:main',
            'servo_driver = leader.servo_driver:main',
            'speed = leader.speed:main',
        ],
    },
)
