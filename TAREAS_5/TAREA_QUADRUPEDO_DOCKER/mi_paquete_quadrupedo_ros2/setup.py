from setuptools import find_packages, setup

package_name = 'mi_paquete_quadrupedo_ros2'

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
    maintainer='Tu Nombre',
    maintainer_email='tu@email.com',
    description='Control de un Minitaur en PyBullet con ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_quadrupedo = scripts.sim_quadrupedo:main',
            'teleop_quadrupedo = scripts.teleop_quadrupedo:main',
        ],
    },
)
