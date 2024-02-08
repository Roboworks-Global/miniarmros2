from setuptools import find_packages, setup

package_name = 'stepper_arm_urdf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),\
        ('share/' + package_name, ['urdf/']),
        ('share/' + package_name + '/meshes', ['meshes/*.stl']),
        ('share/' + package_name + '/config', ['config/*.yaml']),
        ('share/' + package_name + '/launch', ['launch/*.launch.py']),
        ('share/' + package_name + '/scripts', ['scripts/*.py']),
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
        ],
    },
)
