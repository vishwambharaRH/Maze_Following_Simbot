from setuptools import find_packages, setup


package_name = 'maze_solver_logic'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Systems Architect',
    maintainer_email='systems.architect@example.com',
    description='Predictive wall-following control logic for a ROS2 Humble maze solver.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'logic_node = maze_solver_logic.controller_node:main',
        ],
    },
)
