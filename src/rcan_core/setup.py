from setuptools import setup

package_name = 'rcan_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.api', f'{package_name}.services'],
    package_data={package_name: ['py.typed']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/rcan_core']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='RCAN Team',
    maintainer_email='team@example.com',
    description='RCAN core orchestration services and APIs',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'rcan-core = rcan_core.main_rcan:main',
            'rcan = rcan_core.cli:main',
        ],
    },
)
