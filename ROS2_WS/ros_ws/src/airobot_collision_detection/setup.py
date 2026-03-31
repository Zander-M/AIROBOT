from setuptools import find_packages, setup

package_name = 'airobot_collision_detection'

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
    maintainer='Zander Mao',
    maintainer_email='zma40@sfu.ca',
    description='Collision Detection',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'collision_detection=airobot_collision_detection.collision_detection:main',
            'airobot_collision_detection=airobot_collision_detection.collision_detection:main',
        ],
    },
)
