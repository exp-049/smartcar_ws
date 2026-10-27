from glob import glob
from os.path import join
from setuptools import setup
package_name = 'smartcar_bringup'
setup(name=package_name, version='0.1.0', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                  ('share/' + package_name, ['package.xml']),
                  ('share/' + package_name + '/launch', glob('launch/*.py'))],
      install_requires=['setuptools'], zip_safe=True)
