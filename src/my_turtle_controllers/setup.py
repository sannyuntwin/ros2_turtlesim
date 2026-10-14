import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_turtle_controllers'

# Resolve workspace root so the rviz/ directory at the repo root is installed
# regardless of where colcon invokes setup.py from.
_here = os.path.dirname(os.path.abspath(__file__))
_rviz_dir = os.path.join(_here, '..', '..', 'rviz')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/my_turtle_controllers']),
        ('share/my_turtle_controllers', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join(_rviz_dir, '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='you@example.com',
    description='Controller package for driving turtlesim motions.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'draw_circle = my_turtle_controller.draw_circle:main',
            'draw_circle_gazebo = my_turtle_controller.draw_circle_gazebo:main',
        ],
    },
)
