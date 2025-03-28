from setuptools import find_packages, setup

package_name = 'esp32_serial'

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
    maintainer='svr',
    maintainer_email='swarag@svrrobotics.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "serial_publisher = esp32_serial.serial_publisher:main",
        'ultrasonic_sensor_node = esp32_serial.ultrasonic_sensor_node:main',
    ],
    },
)
