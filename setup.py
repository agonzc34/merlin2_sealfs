from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'merlin2_sealfs'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.launch.py'))),
        ('share/' + package_name + '/config', glob(os.path.join('config', '*.yaml'))),
        ('share/' + package_name + '/scripts', glob(os.path.join('scripts', '*.sh'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='agonzc34',
    maintainer_email='agonzc34@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'remapper = merlin2_sealfs.sealfs_remapper:main',
            'plan = merlin2_sealfs.prueba:main',
            'talker = merlin2_sealfs.talker:main'
        ],
    },
)
