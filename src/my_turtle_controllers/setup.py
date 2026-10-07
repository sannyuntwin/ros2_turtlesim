from setuptools import find_packages, setup

package_name = 'my_turtle_controllers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/my_turtle_controllers']),
        ('share/my_turtle_controllers', ['package.xml']),
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
        ],
    },
)
