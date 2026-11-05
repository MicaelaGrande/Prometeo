# Explica como se instala este paquete de Python.
# colcon lo lee al compilar y copia cada cosa a la carpeta install/.

from setuptools import find_packages, setup

nombre_paquete = 'prometeo_navigation'

setup(
    name=nombre_paquete,
    version='0.0.0',

    # Busca las carpetas con codigo. Saltea test/, porque las pruebas
    # no forman parte de lo que se instala.
    packages=find_packages(exclude=['test']),

    # Otros archivos que ROS necesita encontrar: a la izquierda la carpeta
    # de destino dentro de install/, y a la derecha los archivos a copiar.
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + nombre_paquete]),
        ('share/' + nombre_paquete, ['package.xml']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='micaela',
    maintainer_email='micaela.grande@mi.unc.edu.ar',
    description='Navegacion autonoma del robot Prometeo: libreta del recorrido y exploracion',
    license='TODO: License declaration',

    # Los programas que se pueden correr con "ros2 run": nombre = archivo:funcion.
    entry_points={
        'console_scripts': [
            'selector_modo = prometeo_navigation.selector_modo:main',
            'joystick_modo = prometeo_navigation.joystick_modo:main',
        ],
    },
)
