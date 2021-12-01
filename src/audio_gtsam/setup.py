from setuptools import setup

package_name = 'audio_gtsam'

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
    maintainer='Frederike Duembgen',
    maintainer_email='frederike.duembgen@gmail.com',
    description='Package for running custom GTSAM pipeline ',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_mapper = audio_gtsam.wall_mapper:main'
        ],
    },
)
