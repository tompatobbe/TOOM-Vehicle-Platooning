from setuptools import find_packages, setup

package_name = 'ds4'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ds4_launch.launch.py']),    
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
            'ds4_print=ds4.ds4_print:main',
            'servo_driver=ds4.servo_driver:main',
            'axis=ds4.axis:main',
            'motor_driver=ds4.motor_driver:main',
            'ds4_pub=ds4.ds4_pub:main',
            'motor_tester=ds4.motor_tester:main',
        ],
    },
)
