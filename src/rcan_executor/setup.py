from setuptools import find_packages, setup

package_name = 'rcan_executor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Example Maintainer',
    maintainer_email='example@example.com',
    description='Simulated RCAN executor that publishes task status for crane operations.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rcan_executor = rcan_executor.executor:main',
        ],
    },
)
