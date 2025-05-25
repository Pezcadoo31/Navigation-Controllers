from setuptools import find_packages, setup

package_name = 'navigation_controllers'

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
    maintainer='avans',
    maintainer_email='avans@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "fuzzy_logic_controller = navigation_controllers.fuzzy_logic_controller:main",
            "odometry = navigation_controllers.odometry:main",
            "pure_pursuit_controller = navigation_controllers.pure_pursuit_controller:main",
            "trayectory = navigation_controllers.trayectory:main",
            "turn_and_go = navigation_controllers.turn_and_go:main",
            "turn_while_go = navigation_controllers.turn_while_go:main",
        ],
    },
)
