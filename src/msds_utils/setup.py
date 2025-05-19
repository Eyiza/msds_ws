from setuptools import find_packages, setup

package_name = 'msds_utils'

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
    maintainer='eyiza',
    maintainer_email='precious.michael2002@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_stop = msds_utils.safety_stop:main',
            'twist_relay = msds_utils.twist_relay:main',
            'standoff = msds_utils.standoff:main',
            'laser_filter = msds_utils.laser_filter:main',
            'mecanum_odometry = msds_utils.mecanum_odometry:main',
        ],
    },
)
