from setuptools import setup

package_name = "topic_plotter"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="duembgen",
    maintainer_email="frederike.duembgen@epfl.ch",
    description="Utilities to plot data from topics",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "audio = topic_plotter.audio:main",
            "doa = topic_plotter.doa:main",
            "geometry = topic_plotter.geometry:main",
            "motors = topic_plotter.motors:main",
            "status = topic_plotter.status:main",
            "wall = topic_plotter.wall:main",
            "distribution = topic_plotter.distributions:main",
        ],
    },
)
