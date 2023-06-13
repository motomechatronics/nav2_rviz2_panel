from setuptools import setup
import os
from glob import glob

package_name = 'rooms_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/meshes', glob('meshes/*.*')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='salvatore.volpe@akara.ai',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rooms_server_exe = rooms_pkg.rooms_server:main',
            'visualization_markers_exe = rooms_pkg.visualization_markers:main',
            'visualization_markers_array_exe = rooms_pkg.visualization_markers_array:main',
            'visualization_pictograms_array_exe = rooms_pkg.visualization_pictograms_array:main',
            'visualization_markers_text_array_exe = rooms_pkg.visualization_markers_text_array:main',
            'visualization_markers_mesh_array_exe = rooms_pkg.visualization_markers_mesh_array:main',
            'visualization_markers_orientation_array_exe = rooms_pkg.visualization_markers_orientation_array:main',
            'visualization_waypoint_routine_array_exe = rooms_pkg.visualization_waypoint_routine_array:main',
            'visualization_waypoint_routine_text_array_exe = rooms_pkg.visualization_waypoint_routine_text_array:main',
            'visualization_waypoint_routine_text2_array_exe = rooms_pkg.visualization_waypoint_routine_text2_array:main',
            'visualization_waypoint_routine_text_dir_array_exe = rooms_pkg.visualization_waypoint_routine_text_dir_array:main',
            'navroutes_visualization_exe = rooms_pkg.navroutes_visualization:main',
            'navroutes_visualization_server_exe = rooms_pkg.navroutes_visualization_server:main',
            'navroutes_visualization_client_exe = rooms_pkg.navroutes_visualization_client:main',
            'navroutes_manager_exe = rooms_pkg.navroutes_manager:main',
        ],
    },
)
