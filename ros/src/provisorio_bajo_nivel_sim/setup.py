# PROVISORIO: este paquete entero se borra cuando la ESP32 se centre sola (T9).
# Ver LEEME_BORRAR.md
from glob import glob

from setuptools import setup

paquete = 'provisorio_bajo_nivel_sim'

setup(
    name=paquete,
    version='0.0.0',
    packages=[paquete],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + paquete]),
        ('share/' + paquete, ['package.xml']),
        # Los parametros del nodo, para que el launch los encuentre
        ('share/' + paquete + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='micaela',
    maintainer_email='micaela.grande@mi.unc.edu.ar',
    description='PROVISORIO: imita en Gazebo el centrado y el freno que va a hacer la ESP32',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'centrado_simulado = provisorio_bajo_nivel_sim.centrado_simulado:main',
        ],
    },
)
