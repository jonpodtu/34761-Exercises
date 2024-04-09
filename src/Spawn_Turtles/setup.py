from setuptools import find_packages, setup

package_name = 'Spawn_Turtles'

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
    maintainer='jonpo',
    maintainer_email="jonpo@dtu.dk",
    description='Package that spawns too many turtles...',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = Spawn_Turtles.my_node:main',
            'turtle_spawner = Spawn_Turtles.turtle_spawn:main'
        ],
    },
)
