from setuptools import setup
import os
from glob import glob

package_name = 'mi_robot_urdf'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),


        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tu Nombre',
    maintainer_email='tu_email@example.com',
    description='Paquete ROS 2 para visualizar un modelo URDF en Rviz',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'display_rviz = mi_robot_urdf.display_rviz:main',
        ],
    },
)
