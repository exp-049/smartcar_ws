from glob import glob
from os.path import join

from setuptools import find_packages, setup


package_name = "smartcar_gazebo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/worlds", glob(join("worlds", "*.sdf"))),
        (f"share/{package_name}/config", glob(join("config", "*.yaml"))),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="student",
    maintainer_email="student@example.com",
    description="Gazebo Sim differential-drive smartcar and ROS 2 bridge exercise",
    license="Apache-2.0",
)
