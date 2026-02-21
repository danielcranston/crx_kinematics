from glob import glob

from setuptools import find_packages, setup

package_name = "crx_kinematics_py"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/demo.launch.py"]),
        (f"share/{package_name}/config", glob("config/*")),
        (f"share/{package_name}/meshes/crx10ia/visual", glob("meshes/crx10ia/visual/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Daniel",
    maintainer_email="cranston.daniel@gmail.com",
    description="Forward and inverse kinematics for Fanuc CRX series cobots",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": ["demo_node = crx_kinematics_py.demo_node:main"],
    },
)
