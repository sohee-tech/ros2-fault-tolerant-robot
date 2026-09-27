from setuptools import find_packages, setup

package_name = 'fault_injector'

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
    maintainer='sohee',
    maintainer_email='sohee@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_fault_injector = fault_injector.lidar_fault_injector:main',
            'nav2_fault_injector = fault_injector.nav2_fault_injector:main',
            'odom_fault_injector = fault_injector.odom_fault_injector:main',
            'control_delay_injector = fault_injector.control_delay_injector:main',
            'nav_fault_injector = fault_injector.nav_fault_injector:main',
        ],
    },
)
