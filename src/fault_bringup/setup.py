import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'fault_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *[(os.path.join('share', package_name, os.path.dirname(f)), [f])
          for d in ('launch', 'config', 'worlds', 'models')
          for f in glob(os.path.join(d, '**', '*'), recursive=True) if os.path.isfile(f)],
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
            'fault1_test = fault_bringup.fault1_test:main',
        ],
    },
)
