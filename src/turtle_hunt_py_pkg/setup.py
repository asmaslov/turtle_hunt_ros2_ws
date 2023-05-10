from setuptools import setup

package_name = 'turtle_hunt_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='a.maslov',
    maintainer_email='asmaslov4@gmail.com',
    description='Turtle hunt',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'world_node = turtle_hunt_py_pkg.world:main',
            'hunter_node = turtle_hunt_py_pkg.hunter:main',
            'detector_node = turtle_hunt_py_pkg.detector:main',
            'operator_node = turtle_hunt_py_pkg.operator:main'
        ],
    },
)
