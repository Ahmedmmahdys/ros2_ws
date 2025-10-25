from setuptools import setup

package_name = 'rcan_nodes'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_data={
        package_name: [
            'templates/*.j2',
        ],
    },
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='RCAN Team',
    maintainer_email='team@example.com',
    description='RCAN generated ROS2 nodes for RCAN activities',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'rcan-run-install-panel = rcan_nodes.launch.run_install_panel:main',
        ],
    },
)
