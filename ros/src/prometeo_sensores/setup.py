from setuptools import setup

paquete = 'prometeo_sensores'

setup(
    name=paquete,
    version='0.0.0',
    packages=[paquete],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + paquete]),
        ('share/' + paquete, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='micaela',
    maintainer_email='micaela.grande@mi.unc.edu.ar',
    description='Nodos de sensores que corren igual en el robot real y en la simulacion',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'distancias_a_range = prometeo_sensores.distancias_a_range:main',
        ],
    },
)
