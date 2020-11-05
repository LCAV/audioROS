from setuptools import setup

package_name = 'audio_simulation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emilia-szymanska',
    maintainer_email='emiliaszym@gmail.com',
    description='Simulating audio data received by microphones',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crazyflie = audio_simulation.crazyflie:main',
            'constant_pose_publisher = audio_simulation.constant_pose_publisher:main',
        ],
    },
)
