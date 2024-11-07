from setuptools import find_packages, setup
from glob import glob

package_name = 'mini_tof'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # copy everything in the "readers" directory to the install location, so that we can
        # access them with , e.g. 'from readers import tmf8820_reader".
        ('lib/' + package_name + '/readers', glob(package_name + '/readers/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Carter Sifferman',
    maintainer_email='cpsiff@gmail.com',
    description='Package for interfacing with miniature ToF sensors',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tof_publisher = mini_tof.tof_publisher:main',
            'tof_visualizer = mini_tof.tof_visualizer:main',
        ],
    },
)
