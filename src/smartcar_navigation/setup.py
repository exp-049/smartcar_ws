from glob import glob
from os.path import join
from setuptools import setup
package_name = 'smartcar_navigation'
setup(name=package_name, version='0.1.0', packages=[],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                  ('share/' + package_name, ['package.xml']),
                  ('share/' + package_name + '/launch', glob('launch/*.py')),
                  ('share/' + package_name + '/config', glob(join('config', '*.yaml'))),
                  ('share/' + package_name + '/maps', glob(join('maps', '*'))),
                  ('share/' + package_name + '/behavior_trees', glob(join('behavior_trees', '*.xml')))],
      install_requires=['setuptools'], zip_safe=True)
