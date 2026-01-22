from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'openarm_test'

def get_data_files(src_dir, dest_dir):
    data_files = []
    for root, dirs, files in os.walk(src_dir):
        dest = os.path.join(dest_dir, os.path.relpath(root, src_dir))
        data_files.append((dest, [os.path.join(root, f) for f in files]))
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/openarm_test.yaml']),
        ('share/' + package_name + '/launch', ['launch/openarm_test.launch.py']),
    ] + get_data_files('data', 'share/' + package_name + '/data'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='labor',
    maintainer_email='3110379921@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_executor = openarm_test.openarm_test:main',
        ],
    },
)
