from setuptools import setup
package_name = 'smartcar_tools'
setup(name=package_name, version='0.1.0', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]), ('share/' + package_name, ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': ['navigate_to_pose = smartcar_tools.navigate_to_pose:main', 'save_map = smartcar_tools.save_map:main', 'health_check = smartcar_tools.health_check:main', 'path_guard = smartcar_tools.path_guard:main', 'wait_for_ready = smartcar_tools.wait_for_ready:main', 'command_guard = smartcar_tools.command_guard:main', 'cartographer_tf_guard = smartcar_tools.cartographer_tf_guard:main']})
