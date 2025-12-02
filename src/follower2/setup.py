from setuptools import find_packages, setup

package_name = 'follower2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'controller = follower2.controller:main',
            'distance = follower2.distance:main',
            'motor_driver = follower2.motor_driver:main',
            'servo_driver = follower2.servo_driver:main',
        ],
    },
)
