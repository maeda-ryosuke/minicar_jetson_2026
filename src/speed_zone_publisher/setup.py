from glob import glob

from setuptools import find_packages, setup


package_name = "speed_zone_publisher"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ryosuke",
    maintainer_email="ryosuke@users.noreply.github.com",
    description="自己位置とエリア定義から区間の目標速度を /target_speed に出す",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "speed_zone_publisher = speed_zone_publisher.speed_zone_publisher_node:main",
        ],
    },
)
