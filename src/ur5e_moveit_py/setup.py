import os

from setuptools import setup

package_name = "ur5e_moveit_py"
share_dir = os.path.join("share", package_name)

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        # ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (share_dir, ["package.xml"]),
        # (os.path.join(share_dir, "launch"), ["launch/endpoint.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Raoul Zebisch",
    maintainer_email="raoul.zebisch@uni-a.de",
    description="The ur5e_moveit python scripts package",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "trajectory_subscriber = ur5e_moveit_py.trajectory_subscriber:main",
            "mover = ur5e_moveit_py.mover:main"
        ]
    },
)
