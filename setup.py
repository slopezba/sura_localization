from setuptools import setup
import os
from glob import glob


package_name = "sura_localization"


setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="usuario",
    maintainer_email="iedo@uji.es",
    description="SURA localization and sensor fusion launch package",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "enu_to_ned_odometry = sura_localization.enu_to_ned_odometry:main",
            "ned_to_enu_imu = sura_localization.ned_to_enu_imu:main",
        ],
    },
)
