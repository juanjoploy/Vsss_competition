from setuptools import find_packages, setup

package_name = 'deteccion_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juan',
    maintainer_email='juan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'camera_node = deteccion_pkg.camera_node:main',
        'viewer_node = deteccion_pkg.viewer_node:main',
        'trajectory_node = deteccion_pkg.trajectory_node:main',
        'position_analyzer_node = deteccion_pkg.position_analyzer_node:main',
        'trajectory_graph_node = deteccion_pkg.trajectory_graph_node:main',
        'trajectory_plotter_node = deteccion_pkg.trajectory_plotter_node:main',

        ],
    },
)
