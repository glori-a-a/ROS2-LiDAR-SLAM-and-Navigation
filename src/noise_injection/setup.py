from glob import glob

from setuptools import find_packages, setup


package_name = "noise_injection"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Project Maintainer",
    maintainer_email="todo@example.com",
    description="Python LaserScan noise injection node for 2D LiDAR data.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "noise_injection_node = noise_injection.noise_injection_node:main",
        ],
    },
)
