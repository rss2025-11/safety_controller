from setuptools import find_packages, setup
import os
from glob import glob

package_name = "safety_controller"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "safety_controller/params.yaml"]),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="racecar",
    maintainer_email="suchitha.channa@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["safety_controller = safety_controller.safety_check:main"],
    },
)
