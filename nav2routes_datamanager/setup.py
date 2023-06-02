from setuptools import setup
import os
from glob import glob

package_name = 'nav2routes_datamanager'

def copy_yaml_files():
    source_path = 'config/hospitals'
    destination_path = os.path.join('share', package_name, 'config', 'hospitals')

    # Copia i file YAML nelle sottocartelle corrispondenti
    yaml_files = glob('config/hospitals/**/*.yaml', recursive=True)
    data_files = []
    for yaml_file in yaml_files:
        rel_path = os.path.relpath(yaml_file, source_path)
        destination_folder = os.path.join(destination_path, os.path.dirname(rel_path))
        data_files.append((os.path.join('share', package_name, 'config', 'hospitals', os.path.dirname(rel_path)), [yaml_file]))

    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        *copy_yaml_files(),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
         'hospitals_datareader_exe = nav2routes_datamanager.hospitals_datareader:main',
         'hospitals_datareader_server_exe = nav2routes_datamanager.hospitals_datareader_server:main',
        ],
    },
)
