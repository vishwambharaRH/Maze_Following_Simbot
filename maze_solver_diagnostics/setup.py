from setuptools import find_packages, setup


package_name = 'maze_solver_diagnostics'


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
    description='Diagnostics, watchdog, and solve-time logger for the maze solver.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diagnostics_node = maze_solver_diagnostics.diagnostics_node:main',
        ],
    },
)
