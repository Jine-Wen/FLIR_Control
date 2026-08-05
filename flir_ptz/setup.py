import os
from glob import glob

from setuptools import find_packages, setup

package_name = "flir_ptz"


def _web_data_files():
    """Recursively map flir_ptz/web/** onto share/<pkg>/web/** for install.

    Individual files are listed (not a directory copy) so that
    `colcon build --symlink-install` symlinks each asset back to the
    source tree -- editing web/ files then needs no rebuild.
    """
    web_src_root = os.path.join(package_name, "web")
    data_files = []
    for dirpath, _dirnames, filenames in os.walk(web_src_root):
        if not filenames:
            continue
        rel_dir = os.path.relpath(dirpath, web_src_root)
        dest = os.path.join("share", package_name, "web")
        if rel_dir != ".":
            dest = os.path.join(dest, rel_dir)
        data_files.append((dest, [os.path.join(dirpath, f) for f in filenames]))
    return data_files


setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        *_web_data_files(),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="FLIR PTZ Maintainers",
    maintainer_email="maintainers@example.invalid",
    description=(
        "ROS 2 control package for FLIR PTZ cameras: state publishing, "
        "closed-loop motion, auto-scan, and a web control console."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "flir_ptz_node = flir_ptz.nodes.ptz_node:main",
            "flir_ptz_web = flir_ptz.nodes.web_node:main",
            "flir_joy_bridge = flir_ptz.nodes.joy_bridge:main",
        ],
    },
)
