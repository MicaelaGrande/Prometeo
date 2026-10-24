from setuptools import find_packages, setup

package_name = 'huggin_vision'

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
    maintainer='ifacello',
    maintainer_email='ignacio.facello@mi.unc.edu.ar',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_stream = huggin_vision.camera_stream:main',
            'object_recognition = huggin_vision.object_recognition:main',
        ],
    },
)
