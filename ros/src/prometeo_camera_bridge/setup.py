from setuptools import setup

package_name = 'prometeo_camera_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Prometeo Team',
    maintainer_email='team@example.invalid',
    description='ESP32-CAM HTTP/MJPEG to ROS 2 Image bridge',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'camera_bridge = prometeo_camera_bridge.camera_bridge:main',
        ],
    },
)
