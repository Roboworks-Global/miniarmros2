from setuptools import find_packages, setup
from glob import glob

package_name = 'pick_and_place'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='darylleesy',
    maintainer_email='darylleesy@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'colour_detector = pick_and_place.colour_detector_node:main',
            'block_detector = pick_and_place.blockdetector_tester:main',
            'og_colour_detector= pick_and_place.base_colour_detector_node:main',
            'controller = pick_and_place.controller:main',
            'controller_v2 = pick_and_place.controller_v2:main',
            'arm_tester =  pick_and_place.arm_tester:main',
            'nav2_tester = pick_and_place.nav2_tester:main'
        ],
    },
)
