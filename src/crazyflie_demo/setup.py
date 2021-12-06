from setuptools import setup

package_name = "crazyflie_demo"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Frederike Dümbgen",
    maintainer_email="frederike.duembgen@gmail.com",
    description="Run algorithms live while the Crazyflie is flying",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["wall_detection = crazyflie_demo.wall_detection:main"],
    },
)
