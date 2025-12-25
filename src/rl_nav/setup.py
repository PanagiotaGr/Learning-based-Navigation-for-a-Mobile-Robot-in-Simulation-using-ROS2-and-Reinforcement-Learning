from setuptools import setup

package_name = 'rl_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],

    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/random_walk.launch.py']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='panagiotagrosd',
    maintainer_email='panagiotagrosdouli@gmail.com',
    description='RL-friendly navigation nodes and tools for TurtleBot3.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'random_walk = rl_nav.random_walk_node:main',
        ],
    },
)

