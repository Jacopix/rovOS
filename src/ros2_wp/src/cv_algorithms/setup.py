import os
from glob import glob
from setuptools import find_packages, setup

package_name = "cv_algorithms"

setup(
    name = package_name,
    version = "0.1.0",
    packages = find_packages(exclude=["test"]),
    data_files = [
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        #("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("package.xml")),
        #("share/" + package_name + "/launch", ["launch/cv_algorithms_launch.py"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires = ["setuptools"],
    zip_safe = True,
    maintainer = "paz",
    maintainer_email = "niccolo.pasetto@studenti.unipd.it",
    description = "Package containing all of the computer vision algorithms used.",
    license = "Apache-2.0",
    tests_require = ["pytest"],
    entry_points = {
        "console_scripts": [
            "mono_ir_node = " + package_name + ".lc_node_mono_ir:main",
            "stereo_ir_node = " + package_name + ".lc_node_stereo_ir:main",
            "stereo_vo_node = " + package_name + ".lc_node_stereo_vo:main",
        ],
    },
)
