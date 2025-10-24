from setuptools import setup

package_name = 'rcan_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.api', f'{package_name}.services'],
    package_data={package_name: ['py.typed']},
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
