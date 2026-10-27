"""Install the HAL launch and parameter interface."""
from glob import glob
from setuptools import setup

setup(
    name='smartcar_hal', version='0.1.0', packages=['smartcar_hal'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/smartcar_hal']),
        ('share/smartcar_hal', ['package.xml', 'README.md']),
        ('share/smartcar_hal/launch', glob('launch/*.py')),
        ('share/smartcar_hal/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
)
