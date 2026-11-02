import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'fault_web'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'requirements.txt', 'requirements-test.txt']),
        (os.path.join('share', package_name, 'static'), glob('static/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sohee',
    maintainer_email='sohee@todo.todo',
    description='Read-only administrator alerts and QR mobile monitor',
    license='Apache-2.0',
    # no console_scripts: the server needs the .venv_web interpreter (see run_web_monitor.sh)
)
