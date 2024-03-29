import os
from setuptools import setup

package_name = "audio_stack"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [os.path.join("resource", package_name)],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Frederike Duembgen",
    maintainer_email="frederike.duembgen@gmail.com",
    description="Processing and plotting of live audio data",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "processor = audio_stack.processor:main",
            "doa_estimator = audio_stack.doa_estimator:main",
            "spectrum_estimator = audio_stack.spectrum_estimator:main",
            "pose_synch = audio_stack.pose_synch:main",
        ],
    },
)
