from glob import glob

from setuptools import find_packages, setup

package_name = 'brazo_kinematics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ifacello',
    maintainer_email='ignacio.facello@mi.unc.edu.ar',
    description='Cinematica inversa del brazo de paralelogramo',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'ik_node = brazo_kinematics.ik_node:main',
        ],
    },
)
