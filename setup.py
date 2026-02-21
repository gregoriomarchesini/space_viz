from setuptools import setup
from glob import glob

package_name = 'space_viz'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.py')),
  	('share/' + package_name+'/urdf/', glob('urdf/*')),
  	('share/' + package_name+'/rviz/', glob('rviz/*')),
  	('share/' + package_name+'/meshes/ur5e/collision/', glob('meshes/ur5e/collision/*')),
  	('share/' + package_name+'/meshes/ur5e/visual/', glob('meshes/ur5e/visual/*')),
    ('share/' + package_name+'/meshes/inspire_hand/', glob('meshes/inspire_hand/*')),
    ('share/' + package_name+'/meshes/spacecraft/', glob('meshes/spacecraft/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='',
    maintainer_email='gremar@kth.se',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rigid_body_attitude = space_viz.rigid_body_attitude:main',
            'agents = space_viz.agents:main',
        ],
    },
)
