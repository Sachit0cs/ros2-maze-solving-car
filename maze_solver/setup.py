import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'maze_solver'


def data_dir(sub):
    return (os.path.join('share', package_name, sub),
            [f for f in glob(os.path.join(sub, '*')) if os.path.isfile(f)])


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        data_dir('launch'),
        data_dir('description'),
        data_dir('config'),
    ],
    # the control panel's HTML/JS live beside its module and are located via
    # __file__, so they must travel with the package
    package_data={'maze_solver.ui': ['index.html', 'app.js']},
    include_package_data=True,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sachit',
    maintainer_email='info@sagedel.com',
    description='Maze solving with classical graph search on a lidar car.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner       = maze_solver.planner:main',
            'path_driver   = maze_solver.path_driver:main',
            'wall_follower = maze_solver.wall_follower:main',
            'mapper        = maze_solver.mapper:main',
            'maze_manager  = maze_solver.maze_manager:main',
            'maze_gen      = maze_solver.maze_gen:main',
            'make_mazes    = maze_solver.make_mazes:main',
            'control_panel = maze_solver.ui.server:main',
        ],
    },
)
