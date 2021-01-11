import os
from glob import glob
from setuptools import setup

package_name = 'audio_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='duembgen',
    maintainer_email='frederike.duembgen@epfl.ch',
    description='Launch files for audio processing',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'measurement_pipeline = audio_bringup.measurement_pipeline:main',
            'convert_bag_to_csv = audio_bringup.convert_bag_to_csv:main'
        ],
    },
)
