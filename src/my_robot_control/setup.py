from setuptools import find_packages, setup

package_name = 'my_robot_control'

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
    maintainer='daun',
    maintainer_email='clankyrocky1994@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'move_a_to_b = my_robot_control.move_a_to_b:main',
        'trajectory_visualizer = my_robot_control.trajectory_visualizer:main',
        'planned_trajectory = my_robot_control.planned_trajectory:main',
        'welding_line_ik = my_robot_control.welding_line_ik:main',
        ],
    },
)
