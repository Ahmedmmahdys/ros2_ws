from setuptools import find_packages, setup

package_name = 'crane_builder'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/example_panels.yaml']),
    ],
    install_requires=['setuptools', 'neo4j'],
    zip_safe=True,
    maintainer='Example Maintainer',
    maintainer_email='example@example.com',
    description='Executes pick-and-place chains for wall and column panels using IFC GUID sequencing.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'panel_chain_executor = crane_builder.panel_chain_executor:main',
            'neo4j_panel_chain_executor = crane_builder.neo4j_panel_chain_executor:main',
        ],
    },
)
