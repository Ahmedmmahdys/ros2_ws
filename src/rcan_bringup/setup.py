from setuptools import setup

package_name = "rcan_bringup"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/rcan_bringup.launch.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Crane Panel Builder",
    maintainer_email="maintainer@example.com",
    description="Launch configuration for the RCAN executor and panel publishers.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={},
)
