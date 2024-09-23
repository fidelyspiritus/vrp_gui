from setuptools import find_packages, setup

package_name = 'virtual_robot'

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
    maintainer='Anastasiia Markova',
    maintainer_email='anastasiia.a.markova@gmail.com',
    description='Robot simulation for testing',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motors = virtual_robot.motors:main',
            'gps = virtual_robot.gps:main',
            'imu = virtual_robot.imu:main',
            'led_left = virtual_robot.led_left:main',
            'led_right = virtual_robot.led_right:main'
        ],
    },
)
