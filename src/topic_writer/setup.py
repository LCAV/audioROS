from setuptools import setup

package_name = 'topic_writer'

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
    maintainer='duembgen',
    maintainer_email='frederike.duembgen@epfl.ch',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'csv_writer = topic_writer.csv_writer:main',
            'convert_all = topic_writer.convert_all:main'
        ],
    },
)
