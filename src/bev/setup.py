import os
from glob import glob
from setuptools import find_packages, setup

package_name = "bev"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # ADD THIS LINE TO INSTALL THE LAUNCH FILE:
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yjlee",
    maintainer_email="yjlee@todo.todo",
    description="Advanced Bird-Eye View Navigation",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bev_node = bev.bev_node:main",
            "bev_adv_node = bev.bev_adv_node:main",
        ],
    },
)
