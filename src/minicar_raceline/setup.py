from glob import glob
from setuptools import find_packages, setup

package_name = 'minicar_raceline'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryosuke',
    maintainer_email='merondebu0715@gmail.com',
    description='Minicar raceline package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'raceline_manager_node = minicar_raceline.raceline_manager_node:main',
    ]},
)
